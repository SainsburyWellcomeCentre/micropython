/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Gordon L. Mills
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/runtime.h"
#include "py/mphal.h"

#include "hardware/irq.h"
#include "hardware/uart.h"

#define HARPSYNC_UART_BAUDRATE (100000)
#define HARPSYNC_UART_BITS (8)
#define HARPSYNC_UART_STOP (1)

// UART 0 default pins
#if !defined(MICROPY_HW_UART0_TX)
#define MICROPY_HW_UART0_TX (0)
#define MICROPY_HW_UART0_RX (1)
#endif

// UART 1 default pins
#if !defined(MICROPY_HW_UART1_TX)
#define MICROPY_HW_UART1_TX (4)
#define MICROPY_HW_UART1_RX (5)
#endif

#define IS_VALID_PERIPH(uart, pin)  (((((pin) + 4) & 8) >> 3) == (uart))
#define IS_VALID_TX(uart, pin)      (((pin) & 3) == 0 && IS_VALID_PERIPH(uart, pin))
#define IS_VALID_RX(uart, pin)      (((pin) & 3) == 1 && IS_VALID_PERIPH(uart, pin))

#define UART_INVERT_TX (1)
#define UART_INVERT_RX (2)
#define UART_INVERT_MASK (UART_INVERT_TX | UART_INVERT_RX)

const mp_obj_type_t harpsync_type;

typedef struct _harpsync_obj_t {
    mp_obj_base_t base;
    uart_inst_t *const uart;
    uint8_t uart_id;
    uint32_t baudrate;
    uint8_t tx;
    uint8_t rx;
    uint8_t invert;
    int64_t time_us;
} harpsync_obj_t;

STATIC harpsync_obj_t harpsync_obj[] = {
    {{&harpsync_type}, uart0, 0, 0, MICROPY_HW_UART0_TX, MICROPY_HW_UART0_RX, 0, 0},
    {{&harpsync_type}, uart1, 1, 0, MICROPY_HW_UART1_TX, MICROPY_HW_UART1_RX, 0, 0},
};

STATIC const char *_invert_name[] = {"None", "INV_TX", "INV_RX", "INV_TX|INV_RX"};

/******************************************************************************/
// IRQ handling

STATIC inline void uart_service_interrupt(harpsync_obj_t *self) {
    static const int64_t calib_us = 10 - 572;   // IRQ latency = 10 us, sync end 572 us early.
    static uint8_t rx_buffer[6] = {0xaa, 0xaf, 0, 0 ,0 ,0};
    static uint8_t rx_buf_idx = 0;
    if (uart_get_hw(self->uart)->mis & UART_UARTMIS_RXMIS_BITS) { // rx interrupt?
        // clear interrupt bit
        uart_get_hw(self->uart)->icr = UART_UARTMIS_RXMIS_BITS;
        // Parse harp sync message bytes.
        while (uart_is_readable(self->uart)) {
            uint8_t rx_byte = uart_get_hw(self->uart)->dr;
            if (rx_buf_idx < 2) {
                rx_buf_idx = (rx_buffer[rx_buf_idx] == rx_byte) ? rx_buf_idx + 1 : 0;
            } else {
                rx_buffer[rx_buf_idx++] = rx_byte;
                if (rx_buf_idx == sizeof(rx_buffer)) {
                    int64_t time_s = *((uint32_t*)(&rx_buffer[2])) + 1;
                    self->time_us = 1000000 * time_s + calib_us - time_us_64();
                    rx_buf_idx = 0;
                }
            }
        }
    }
}

STATIC void uart0_irq_handler(void) {
    uart_service_interrupt(&harpsync_obj[0]);
}

STATIC void uart1_irq_handler(void) {
    uart_service_interrupt(&harpsync_obj[1]);
}

/******************************************************************************/
// MicroPython bindings for harpsync

STATIC void harpsync_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    harpsync_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "harpsync(%u, baudrate=%u, tx=%d, rx=%d, invert=%s)",
        self->uart_id, self->baudrate, self->tx, self->rx, _invert_name[self->invert]);
}

STATIC void harpsync_init_helper(harpsync_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_tx, ARG_rx, ARG_invert};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_tx, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_rx, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_invert, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Set TX/RX pins if configured.
    if (args[ARG_tx].u_obj != mp_const_none) {
        int tx = mp_hal_get_pin_obj(args[ARG_tx].u_obj);
        if (!IS_VALID_TX(self->uart_id, tx)) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad TX pin"));
        }
        self->tx = tx;
    }
    if (args[ARG_rx].u_obj != mp_const_none) {
        int rx = mp_hal_get_pin_obj(args[ARG_rx].u_obj);
        if (!IS_VALID_RX(self->uart_id, rx)) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad RX pin"));
        }
        self->rx = rx;
    }

    // Set line inversion if configured.
    if (args[ARG_invert].u_int >= 0) {
        if (args[ARG_invert].u_int & ~UART_INVERT_MASK) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad inversion mask"));
        }
        self->invert = args[ARG_invert].u_int;
    }

    // Initialise the UART peripheral if any arguments given, or it was not initialised previously.
    if (n_args > 0 || kw_args->used > 0 || self->baudrate == 0) {
        self->baudrate = uart_init(self->uart, HARPSYNC_UART_BAUDRATE);
        uart_set_format(self->uart, HARPSYNC_UART_BITS, HARPSYNC_UART_STOP, UART_PARITY_NONE);
        uart_set_fifo_enabled(self->uart, false);
        gpio_set_function(self->tx, GPIO_FUNC_UART);
        gpio_set_function(self->rx, GPIO_FUNC_UART);
        if (self->invert & UART_INVERT_RX) {
            gpio_set_inover(self->rx, GPIO_OVERRIDE_INVERT);
        }
        if (self->invert & UART_INVERT_TX) {
            gpio_set_outover(self->tx, GPIO_OVERRIDE_INVERT);
        }

        // Set the irq handler.
        if (self->uart_id == 0) {
            irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
            irq_set_enabled(UART0_IRQ, true);
        } else {
            irq_set_exclusive_handler(UART1_IRQ, uart1_irq_handler);
            irq_set_enabled(UART1_IRQ, true);
        }

        // Enable the uart rx irq; this macro sets the rx irq level to 4.
        uart_set_irq_enables(self->uart, true, false);
    }
}

STATIC mp_obj_t harpsync_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // Get UART bus.
    int uart_id = mp_obj_get_int(args[0]);
    if (uart_id < 0 || uart_id >= MP_ARRAY_SIZE(harpsync_obj)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("UART(%d) doesn't exist"), uart_id);
    }

    // Get static peripheral object.
    harpsync_obj_t *self = (harpsync_obj_t *)&harpsync_obj[uart_id];

    // Initialise the UART peripheral.
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    harpsync_init_helper(self, n_args - 1, args + 1, &kw_args);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t harpsync_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    // Initialise the UART peripheral.
    harpsync_init_helper(args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(harpsync_init_obj, 1, harpsync_init);

STATIC mp_obj_t harpsync_deinit(mp_obj_t self_in) {
    harpsync_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uart_deinit(self->uart);
    if (self->uart_id == 0) {
        irq_set_enabled(UART0_IRQ, false);
    } else {
        irq_set_enabled(UART1_IRQ, false);
    }
    self->baudrate = 0;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(harpsync_deinit_obj, harpsync_deinit);

STATIC mp_obj_t harpsync_read(mp_obj_t self_in) {
    harpsync_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int64_t us = self->time_us + time_us_64();
    mp_obj_t timestamp[2];
    timestamp[0] = mp_obj_new_int(us / 1000000);
    timestamp[1] = mp_obj_new_int((us % 1000000) / 32);
    return mp_obj_new_tuple(2, timestamp);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(harpsync_read_obj, harpsync_read);

STATIC mp_obj_t harpsync_write(mp_obj_t self_in, mp_obj_t seconds) {
    harpsync_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int64_t time_s = mp_obj_get_int(seconds);
    self->time_us = 1000000 * time_s - time_us_64();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(harpsync_write_obj, harpsync_write);

STATIC const mp_rom_map_elem_t harpsync_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&harpsync_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&harpsync_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&harpsync_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&harpsync_write_obj) },

    { MP_ROM_QSTR(MP_QSTR_INV_TX), MP_ROM_INT(UART_INVERT_TX) },
    { MP_ROM_QSTR(MP_QSTR_INV_RX), MP_ROM_INT(UART_INVERT_RX) },
};
STATIC MP_DEFINE_CONST_DICT(harpsync_locals_dict, harpsync_locals_dict_table);

const mp_obj_type_t harpsync_type = {
    { &mp_type_type },
    .name = MP_QSTR_harpsync,
    .print = harpsync_print,
    .make_new = harpsync_make_new,
    .locals_dict = (mp_obj_dict_t *)&harpsync_locals_dict,
};

STATIC const mp_rom_map_elem_t harpsync_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_harpsync) },
    { MP_ROM_QSTR(MP_QSTR_harpsync), MP_ROM_PTR(&harpsync_type) },
};
STATIC MP_DEFINE_CONST_DICT(harpsync_module_globals, harpsync_module_globals_table);

const mp_obj_module_t mp_module_harpsync = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&harpsync_module_globals,
};
