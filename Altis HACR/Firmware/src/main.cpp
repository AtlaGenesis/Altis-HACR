#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <algorithm>
#include <cstdlib>  // For std::abs (integers)

constexpr uint PWM_PIN = 16;
constexpr uint BUTTON_PIN = 15;
constexpr uint POT_ADC_INPUT = 0;
constexpr uint PWM_FREQ_HZ = 50;
constexpr uint16_t PWM_MIN_US = 1000;
constexpr uint16_t PWM_MAX_US = 2000;
constexpr uint32_t LONG_PRESS_MS = 1000;
constexpr uint32_t ESC_ARM_DELAY_MS = 2000;
constexpr uint16_t ADC_DEADBAND = 5;
constexpr uint32_t PWM_WRAP = 20000;  // 20ms period at 1MHz clock
constexpr uint16_t RAMP_STEP_US = 10; // Max adjustment per loop for safer ramping
constexpr uint32_t DEBOUNCE_MS = 10;  // Button debounce time

// Initializes PWM for 50Hz with 1us resolution.
// FIX: Return type changed from void to uint so pwm_slice can be returned.
uint init_pwm_50hz() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    auto pwm_slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_clkdiv(pwm_slice, 125.0f);  // Assumes 125MHz sys clock -> 1MHz PWM clock
    pwm_set_wrap(pwm_slice, PWM_WRAP);
    pwm_set_enabled(pwm_slice, true);
    return pwm_slice;
}

// Sets PWM duty cycle in microseconds, clamped to min/max.
void set_pwm_us(uint16_t us) {
    us = std::clamp(us, PWM_MIN_US, PWM_MAX_US);
    pwm_set_gpio_level(PWM_PIN, us);  // 1 tick = 1us
}

// Reads potentiometer via ADC and maps to PWM range (1000-2000us).
uint16_t read_pot_us() {
    uint16_t raw = adc_read();
    // Linear mapping: MIN + (raw / 4095.0) * (MAX - MIN), integer math for speed
    uint16_t mapped = PWM_MIN_US + (raw * (PWM_MAX_US - PWM_MIN_US)) / 4095;
    return std::clamp(mapped, PWM_MIN_US, PWM_MAX_US);
}

int main() {
    stdio_init_all();
    auto pwm_slice = init_pwm_50hz();  // Unused here, but available for expansion
    (void)pwm_slice;                   // Suppress unused variable warning

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_down(BUTTON_PIN);  // Button to VCC; high when pressed

    adc_init();
    adc_gpio_init(26 + POT_ADC_INPUT);  // GPIO26 for ADC0
    adc_select_input(POT_ADC_INPUT);

    set_pwm_us(PWM_MIN_US);
    sleep_ms(ESC_ARM_DELAY_MS);  // Arm ESC by holding min PWM

    struct SystemState {
        bool is_on = false;
        bool is_pressed = false;
        bool last_btn = false;              // FIX: tracks last stable button state for debounce
        absolute_time_t press_start = nil_time;
        uint16_t current_pwm = PWM_MIN_US;
        absolute_time_t last_button_time = nil_time;
    } state;

    while (true) {
        bool current_btn = gpio_get(BUTTON_PIN);
        auto now = get_absolute_time();

        // FIX: Reset debounce timer on any state change, only act after stable for DEBOUNCE_MS
        if (current_btn != state.last_btn) {
            state.last_button_time = now;
            state.last_btn = current_btn;
        }

        if (absolute_time_diff_us(state.last_button_time, now) >= DEBOUNCE_MS * 1000) {
            // Rising edge (press)
            if (current_btn && !state.is_pressed) {
                state.is_pressed = true;
                state.press_start = now;
            }
            // Falling edge (release)
            if (!current_btn && state.is_pressed) {
                state.is_pressed = false;
                uint32_t held_ms = static_cast<uint32_t>(
                    absolute_time_diff_us(state.press_start, now) / 1000);
                if (held_ms >= LONG_PRESS_MS) {
                    state.is_on = !state.is_on;
                }
            }
        }

        // PWM control: Ramp towards target with deadband and step limit
        if (state.is_on) {
            uint16_t target = read_pot_us();
            int16_t diff = static_cast<int16_t>(target) - static_cast<int16_t>(state.current_pwm);
            if (std::abs(diff) > ADC_DEADBAND) {
                uint16_t step = std::min(static_cast<uint16_t>(std::abs(diff)), RAMP_STEP_US);
                state.current_pwm += (diff > 0) ? step : -step;
                set_pwm_us(state.current_pwm);
            }
        } else {
            state.current_pwm = PWM_MIN_US;
            set_pwm_us(PWM_MIN_US);
        }

        sleep_ms(1);  // ~1kHz loop rate
    }
}