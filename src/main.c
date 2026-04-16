
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"

// PINES

#define PIN_BTN_DER      15
#define PIN_BTN_IZQ      35

#define PIN_LED_VERDE    14
#define PIN_LED_ROJO     13

#define PIN_SEG_A        16
#define PIN_SEG_B        17
#define PIN_SEG_C        18
#define PIN_SEG_D        19
#define PIN_SEG_E        21
#define PIN_SEG_F        22
#define PIN_SEG_G        23

#define PIN_DIG_CEN      25
#define PIN_DIG_DEC      33
#define PIN_DIG_UNI      32

// Puente H

#define PIN_HS_LEFT      4    // IRF9630 izquierdo
#define PIN_LS_LEFT      26   // IRLZ44N izquierdo
#define PIN_HS_RIGHT     5    // IRF9630 derecho
#define PIN_LS_RIGHT     27   // IRLZ44N derecho

#define ADC_CHANNEL      ADC1_CHANNEL_6  


// PWM

#define PWM_FREQ_HZ      500
#define PWM_RESOLUTION   LEDC_TIMER_8_BIT
#define PWM_MODE         LEDC_HIGH_SPEED_MODE
#define PWM_TIMER        LEDC_TIMER_0
#define PWM_CHANNEL_HL   LEDC_CHANNEL_0   // PWM en HS_LEFT
#define PWM_CHANNEL_HR   LEDC_CHANNEL_1   // PWM en HS_RIGHT
#define PWM_DUTY_MAX     255


// TIEMPOS

#define SAFE_REVERSE_MS  300
#define DISPLAY_DELAY_MS 2
#define BUTTON_DELAY_MS  20
#define MOTOR_DELAY_MS   10

typedef enum

{

    DIR_RIGHT = 0,
    DIR_LEFT

} motor_dir_t;


// LÓGICA
// IRF9630: 1 = ON, 0 = OFF
// IRLZ44N: 0 = ON, 1 = OFF

#define HS_ON   1
#define HS_OFF  0
#define LS_ON   0
#define LS_OFF  1

// VARIABLES GLOBALES

static volatile int percent_power = 0;
static volatile int pwm_value = 0;
static volatile motor_dir_t current_dir = DIR_RIGHT;
static volatile motor_dir_t requested_dir = DIR_RIGHT;
static volatile bool direction_change_pending = false;

// DISPLAY
// ÁNODO COMÚN:
// 0 = segmento encendido
// 1 = segmento apagado

static const uint8_t digits_map[10][7] = {

    {0,0,0,0,0,0,1}, 

    {1,0,0,1,1,1,1}, 

    {0,0,1,0,0,1,0}, 

    {0,0,0,0,1,1,0}, 

    {1,0,0,1,1,0,0}, 

    {0,1,0,0,1,0,0}, 

    {0,1,0,0,0,0,0}, 

    {0,0,0,1,1,1,1}, 

    {0,0,0,0,0,0,0}, 

    {0,0,0,0,1,0,0}  

};

// GPIO

static void gpio_init_all(void)

{

    gpio_config_t out_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << PIN_LED_VERDE) |
            (1ULL << PIN_LED_ROJO)  |
            (1ULL << PIN_SEG_A)     |
            (1ULL << PIN_SEG_B)     |
            (1ULL << PIN_SEG_C)     |
            (1ULL << PIN_SEG_D)     |
            (1ULL << PIN_SEG_E)     |
            (1ULL << PIN_SEG_F)     |
            (1ULL << PIN_SEG_G)     |
            (1ULL << PIN_DIG_CEN)   |
            (1ULL << PIN_DIG_DEC)   |
            (1ULL << PIN_DIG_UNI)   |
            (1ULL << PIN_LS_LEFT)   |
            (1ULL << PIN_LS_RIGHT),

        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&out_conf);

    gpio_config_t in_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_BTN_DER) | (1ULL << PIN_BTN_IZQ),

        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE

    };

    gpio_config(&in_conf);
}

// ADC

static void adc_init_all(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
}

// PWM EN LOS P-MOS

static void pwm_init_all(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK

    };

    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_hl = {
        .gpio_num   = PIN_HS_LEFT,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL_HL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    ledc_channel_config(&ch_hl);

    ledc_channel_config_t ch_hr = {
        .gpio_num   = PIN_HS_RIGHT,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL_HR,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch_hr);
}

static void all_digits_off(void)
{
    gpio_set_level(PIN_DIG_CEN, 1);
    gpio_set_level(PIN_DIG_DEC, 1);
    gpio_set_level(PIN_DIG_UNI, 1);
}

static void enable_digit(int digit)
{
    all_digits_off();
    if (digit == 0) gpio_set_level(PIN_DIG_CEN, 0);
    if (digit == 1) gpio_set_level(PIN_DIG_DEC, 0);
    if (digit == 2) gpio_set_level(PIN_DIG_UNI, 0);
}

static void set_segments(int number)
{
    if (number < 0 || number > 9) number = 0;
    gpio_set_level(PIN_SEG_A, digits_map[number][0]);
    gpio_set_level(PIN_SEG_B, digits_map[number][1]);
    gpio_set_level(PIN_SEG_C, digits_map[number][2]);
    gpio_set_level(PIN_SEG_D, digits_map[number][3]);
    gpio_set_level(PIN_SEG_E, digits_map[number][4]);
    gpio_set_level(PIN_SEG_F, digits_map[number][5]);
    gpio_set_level(PIN_SEG_G, digits_map[number][6]);
}

static void display_task(void *pvParameters)
{
    int digit = 0;
    while (1)
    {
        int centenas = percent_power / 100;
        int decenas  = (percent_power / 10) % 10;
        int unidades = percent_power % 10;
        switch (digit)
        {
            case 0: set_segments(centenas); enable_digit(0); break;
            case 1: set_segments(decenas);  enable_digit(1); break;
            case 2: set_segments(unidades); enable_digit(2); break;
        }
        digit++;
        if (digit > 2) digit = 0;
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_DELAY_MS));
    }
}

static void update_leds(motor_dir_t dir)
{
    if (dir == DIR_RIGHT)
    {
        gpio_set_level(PIN_LED_VERDE, 1);
        gpio_set_level(PIN_LED_ROJO, 0);
    }
    else
    {
        gpio_set_level(PIN_LED_VERDE, 0);
        gpio_set_level(PIN_LED_ROJO, 1);
    }
}

// MOTOR

static void pwm_all_off(void)
{
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_HL, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_HL);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_HR, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_HR);
}

static void motor_all_off(void)
{
    // Ambos P-MOS OFF
    pwm_all_off();
    // Ambos N-MOS OFF
    gpio_set_level(PIN_LS_LEFT,  LS_OFF);
    gpio_set_level(PIN_LS_RIGHT, LS_OFF);
}

static void motor_drive_right(int pwm)
{    motor_all_off();
    //P izquierdo PWM + N derecho ON
    gpio_set_level(PIN_LS_RIGHT, LS_ON);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_HL, pwm);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_HL);
}

static void motor_drive_left(int pwm)
{    motor_all_off();
    // P derecho PWM + N izquierdo ON
    gpio_set_level(PIN_LS_LEFT, LS_ON);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_HR, pwm);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_HR);
}

static void motor_task(void *pvParameters)
{    while (1)    {
        if (direction_change_pending){
            motor_all_off();
            vTaskDelay(pdMS_TO_TICKS(SAFE_REVERSE_MS));
            current_dir = requested_dir;
            update_leds(current_dir);
            direction_change_pending = false;
        }

        if (pwm_value <= 0){
            motor_all_off();
        }
        else{
            if (current_dir == DIR_RIGHT){
                motor_drive_right(pwm_value);
            }
            else{
                motor_drive_left(pwm_value);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_DELAY_MS));
    }

}

// ADC

static void adc_task(void *pvParameters)
{
    while (1){
        int raw = adc1_get_raw(ADC_CHANNEL);
        if (raw < 0) raw = 0;
        if (raw > 4095) raw = 4095;
        percent_power = (raw * 100) / 4095;
        pwm_value = (percent_power * PWM_DUTY_MAX) / 100;

        vTaskDelay(pdMS_TO_TICKS(20));
    }

}

static void button_task(void *pvParameters){
    bool last_der = true;
    bool last_izq = true;

    while (1){
        bool der = gpio_get_level(PIN_BTN_DER);
        bool izq = gpio_get_level(PIN_BTN_IZQ);

        if (last_der == 1 && der == 0){
            requested_dir = DIR_RIGHT;
            if (requested_dir != current_dir)
            {
                direction_change_pending = true;
            }
        }
        if (last_izq == 1 && izq == 0)
        {
            requested_dir = DIR_LEFT;
            if (requested_dir != current_dir)
            {
                direction_change_pending = true;
            }
        }

        last_der = der;
        last_izq = izq;

        vTaskDelay(pdMS_TO_TICKS(BUTTON_DELAY_MS));
    }
}

// MAIN

void app_main(void){

    gpio_init_all();
    adc_init_all();
    pwm_init_all();

    current_dir = DIR_RIGHT;
    requested_dir = DIR_RIGHT;
    direction_change_pending = false;
    update_leds(current_dir);
    motor_all_off();
    all_digits_off();

    xTaskCreate(display_task, "display_task", 2048, NULL, 1, NULL);
    xTaskCreate(adc_task, "adc_task", 2048, NULL, 1, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 1, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 1, NULL);
}
 