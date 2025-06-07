/*
  ioports.c - driver code for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "grbl/protocol.h"

static io_ports_data_t digital;
static input_signal_t *aux_in;
static output_signal_t *aux_out;
static volatile uint32_t event_bits;

static void digital_out_pwm (struct xbar *output, float value)
{
    if(output->id < digital.out.n_ports) {

        uint_fast16_t pwm_value = ioports_compute_pwm_value(&aux_out[output->id].pwm->data, value);
        const pwm_signal_t *pwm = aux_out[output->id].pwm->port;

        aux_out[output->id].pwm->value = value;

        if(pwm_value == aux_out[output->id].pwm->data.off_value) {
            if(aux_out[output->id].pwm->data.always_on) {
                *pwm->ccr = aux_out[output->id].pwm->data.off_value;
                if(IS_TIMER_BDTR(pwm->timer))
                    pwm->timer->BDTR |= TIM_BDTR_MOE;
                *pwm->ccr = 0;
            } else {
                if(IS_TIMER_BDTR(pwm->timer))
                    pwm->timer->BDTR |= TIM_BDTR_MOE;
                *pwm->ccr = 0;
            }
        } else {
            *pwm->ccr = pwm_value;
            if(IS_TIMER_BDTR(pwm->timer))
                pwm->timer->BDTR |= TIM_BDTR_MOE;
        }
    }
}

static float digital_out_pwm_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < digital.out.n_ports)
        value = aux_out[output->id].pwm->value;

    return value;
}

static bool digital_out_pwm_cfg (xbar_t *output, pwm_config_t *config, bool persistent)
{
    bool ok;

    if((ok = !!aux_out[output->id].pwm)) {

        uint32_t prescaler = 0, clock_hz = pwm_get_clock_hz(aux_out[output->id].pwm->port);

        do {
            prescaler++;
            ok = ioports_precompute_pwm_values(config, &aux_out[output->id].pwm->data, clock_hz / prescaler);
        } while(ok && aux_out[output->id].pwm->data.period > 65530);

        if(ok) {

            pwm_config(aux_out[output->id].pwm->port, prescaler, aux_out[output->id].pwm->data.period, config->invert);

            aux_out[output->id].mode.pwm = !config->servo_mode;
            aux_out[output->id].mode.servo_pwm = config->servo_mode;

            digital_out_pwm(output, config->min);
        }
    }

    return ok;
}

static bool digital_out_cfg (xbar_t *output, gpio_out_config_t *config, bool persistent)
{
    if(output->id < digital.out.n_ports && aux_out[output->id].pwm == NULL) {

        if(config->pwm && output->mode.claimed) {

            pwm_out_t *pwm;

            if((pwm = calloc(sizeof(pwm_out_t), 1))) {
                if((pwm->port = pwm_claim((GPIO_TypeDef *)output->port, output->pin))) {
                    pwm_enable(pwm->port);
                    aux_out[output->id].pwm = pwm;
                } else
                    free(pwm);
            }

            if(aux_out[output->id].pwm == NULL)
                return false;
        }

        if(config->inverted != aux_out[output->id].mode.inverted) {
            aux_out[output->id].mode.inverted = config->inverted;
            DIGITAL_OUT(aux_out[output->id].port, aux_out[output->id].bit, !DIGITAL_IN(aux_out[output->id].port, aux_out[output->id].bit));
        }

        if(config->open_drain != aux_out[output->id].mode.open_drain) {
            if((aux_out[output->id].mode.open_drain = config->open_drain))
                aux_out[output->id].port->OTYPER |= (GPIO_OTYPER_OT0 << aux_out[output->id].pin);
            else
                aux_out[output->id].port->OTYPER &= ~(GPIO_OTYPER_OT0 << aux_out[output->id].pin);
        }

        if(persistent)
            ioport_save_output_settings(output, config);
    }

    return output->id < digital.out.n_ports;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < digital.out.n_ports)
        DIGITAL_OUT(aux_out[port].port, aux_out[port].bit, aux_out[port].mode.inverted ? !on : on);
}

static float digital_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < digital.out.n_ports)
        value = (float)(DIGITAL_IN(aux_out[output->id].port, aux_out[output->id].bit) ^ aux_out[output->id].mode.inverted);

    return value;
}

static bool digital_in_cfg (xbar_t *input, gpio_in_config_t *config, bool persistent)
{
    if(input->id < digital.in.n_ports && config->pull_mode != PullMode_UpDown) {

        aux_in[input->id].mode.inverted = config->inverted;
        aux_in[input->id].mode.debounce = config->debounce;
        aux_in[input->id].mode.pull_mode = config->pull_mode;
        aux_in[input->id].port->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (input->pin << 1));
        aux_in[input->id].port->PUPDR |= (config->pull_mode << (input->pin << 1));

        if(persistent)
            ioport_save_input_settings(input, config);
    }

    return input->id < digital.in.n_ports;
}

static float digital_in_state (xbar_t *input)
{
    float value = -1.0f;

    if(input->id < digital.in.n_ports)
        value = (float)(DIGITAL_IN(aux_in[input->id].port, aux_in[input->id].bit) ^ aux_in[input->id].mode.inverted);

    return value;
}

inline static __attribute__((always_inline)) int32_t get_input (const input_signal_t *input, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return DIGITAL_IN(input->port, input->bit) ^ input->mode.inverted;

    int32_t value = -1;
    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    if(wait_mode == WaitMode_Rise || wait_mode == WaitMode_Fall) {

        pin_irq_mode_t irq_mode = wait_mode == WaitMode_Rise ? IRQ_Mode_Rising : IRQ_Mode_Falling;

        if(input->cap.irq_mode & irq_mode) {

            event_bits &= ~input->bit;
            gpio_irq_enable(input, irq_mode);

            do {
                if(event_bits & input->bit) {
                    value = DIGITAL_IN(input->port, input->bit) ^ input->mode.inverted;
                    break;
                }
                if(delay) {
                    protocol_execute_realtime();
                    hal.delay_ms(50, NULL);
                } else
                    break;
            } while(--delay && !sys.abort);

            gpio_irq_enable(input, input->mode.irq_mode);    // Restore pin interrupt status
        }

    } else {

        bool wait_for = wait_mode != WaitMode_Low;

        do {
            if((DIGITAL_IN(input->port, input->bit) ^ input->mode.inverted) == wait_for) {
                value = DIGITAL_IN(input->port, input->bit) ^ input->mode.inverted;
                break;
            }
            if(delay) {
                protocol_execute_realtime();
                hal.delay_ms(50, NULL);
            } else
                break;
        } while(--delay && !sys.abort);

    }

    return value;
}

static int32_t wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(port < digital.in.n_ports)
        value = get_input(&aux_in[port], wait_mode, timeout);

    return value;
}

void ioports_event (input_signal_t *input)
{
    if(input) {
        event_bits |= input->bit;
        if(input->interrupt_callback)
            input->interrupt_callback(input->user_port, DIGITAL_IN(input->port, input->bit));
    }
}

static bool register_interrupt_handler (uint8_t port, uint8_t user_port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    bool ok;

    if((ok = port < digital.in.n_ports && aux_in[port].cap.irq_mode != IRQ_Mode_None)) {

        input_signal_t *input = &aux_in[port];

        if((ok = (irq_mode & input->cap.irq_mode) == irq_mode && interrupt_callback != NULL)) {
            input->user_port = user_port;
            input->mode.irq_mode = irq_mode;
            input->interrupt_callback = interrupt_callback;
            gpio_irq_enable(input, irq_mode);
        }

        if(irq_mode == IRQ_Mode_None || !ok) {
            hal.irq_disable();
            EXTI->IMR1 &= ~input->bit;     // Disable pin interrupt
            input->mode.irq_mode = IRQ_Mode_None;
            input->interrupt_callback = NULL;
            hal.irq_enable();
        }
    }

    return ok;
}

static bool set_function (xbar_t *port, pin_function_t function)
{
    if(port->mode.input)
        aux_in[port->id].id = function;
    else
        aux_out[port->id].id = function;

    return true;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    pin.set_function = set_function;

    if(dir == Port_Input && port < digital.in.n_ports) {
        XBAR_SET_DIN_INFO(pin, port, aux_in[port], digital_in_cfg, digital_in_state);
        info = &pin;
    }

    if(dir == Port_Output && port < digital.out.n_ports) {
        XBAR_SET_DOUT_INFO(pin, port, aux_out[port], digital_out_cfg, digital_out_state);
        if((pin.cap.pwm = !!aux_out[port].pwm)) {
            pin.config = digital_out_pwm_cfg;
            pin.get_value = digital_out_pwm_state;
            pin.set_value = digital_out_pwm;
        } else
            pin.cap.pwm = !pin.mode.claimed && pwm_is_available(aux_out[pin.id].port, aux_out[pin.id].pin);
        info = &pin;
    }

    return info;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *s)
{
    if(dir == Port_Input && port < digital.in.n_ports)
        aux_in[port].description = s;

    if(dir == Port_Output && port < digital.out.n_ports)
        aux_out[port].description = s;
}

void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    aux_in = aux_inputs->pins.inputs;
    aux_out = aux_outputs->pins.outputs;

    digital.in.n_ports = aux_inputs->n_pins;
    digital.out.n_ports = aux_outputs->n_pins;

    io_digital_t ports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = get_pin_info,
        .wait_on_input = wait_on_input,
        .set_pin_description = set_pin_description,
        .register_interrupt_handler = register_interrupt_handler
    };

    ioports_add_digital(&ports);
}
