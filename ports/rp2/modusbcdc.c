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
#include "py/stream.h"
#include "py/mphal.h"

#include "tusb.h"


const mp_obj_type_t usbcdc_type;

typedef struct _usbcdc_obj_t {
    mp_obj_base_t base;
    uint8_t itf_id;
    uint16_t timeout;       // timeout waiting for first char (in ms)
    uint16_t timeout_char;  // timeout waiting between chars (in ms)
} usbcdc_obj_t;

STATIC usbcdc_obj_t usbcdc_obj[] = {
    {{&usbcdc_type}, 0, 0, 0},
    {{&usbcdc_type}, 1, 0, 0},
};

STATIC const char *_parity_name[] = {"None", "Odd", "Even", "Mark", "Space"};

/******************************************************************************/
// MicroPython bindings for USBCDC

STATIC void usbcdc_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    usbcdc_obj_t *self = MP_OBJ_TO_PTR(self_in);
    cdc_line_coding_t linec;
    tud_cdc_n_get_line_coding(self->itf_id, &linec);
    mp_printf(print, "USBCDC(%u, baudrate=%u, bits=%u, parity=%s, stop=%u, timeout=%u, timeout_char=%u)",
        self->itf_id, linec.bit_rate, linec.data_bits, _parity_name[linec.parity], linec.stop_bits,
        self->timeout, self->timeout_char);
}

STATIC void usbcdc_init_helper(usbcdc_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_timeout, ARG_timeout_char};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_timeout_char, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Set timeout if configured.
    if (args[ARG_timeout].u_int >= 0) {
        self->timeout = args[ARG_timeout].u_int;
    }

    // Set timeout_char if configured.
    if (args[ARG_timeout_char].u_int >= 0) {
        self->timeout_char = args[ARG_timeout_char].u_int;
    }
}

STATIC mp_obj_t usbcdc_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // Get CDC interface.
    int itf_id = mp_obj_get_int(args[0]);
    if (itf_id < 0 || itf_id >= MP_ARRAY_SIZE(usbcdc_obj)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("USBCDC(%d) doesn't exist"), itf_id);
    }

    // Get static peripheral object.
    usbcdc_obj_t *self = (usbcdc_obj_t *)&usbcdc_obj[itf_id];

    // Initialise the peripheral object.
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    usbcdc_init_helper(self, n_args - 1, args + 1, &kw_args);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t usbcdc_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    // Initialise the peripheral object.
    usbcdc_init_helper(args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(usbcdc_init_obj, 1, usbcdc_init);

STATIC mp_obj_t usbcdc_any(mp_obj_t self_in) {
    usbcdc_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return MP_OBJ_NEW_SMALL_INT(tud_cdc_n_available(self->itf_id));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(usbcdc_any_obj, usbcdc_any);

STATIC const mp_rom_map_elem_t usbcdc_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&usbcdc_init_obj) },

    { MP_ROM_QSTR(MP_QSTR_any), MP_ROM_PTR(&usbcdc_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_stream_write_obj) },
};
STATIC MP_DEFINE_CONST_DICT(usbcdc_locals_dict, usbcdc_locals_dict_table);

STATIC mp_uint_t usbcdc_read(mp_obj_t self_in, void *buf_in, mp_uint_t size, int *errcode) {
    usbcdc_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint64_t t = time_us_64() + (uint64_t)self->timeout * 1000;
    uint64_t timeout_char_us = (uint64_t)self->timeout_char * 1000;
    uint8_t *dest = buf_in;

    for (size_t i = 0; i < size; i++) {
        // Wait for the first/next character
        while (tud_cdc_n_available(self->itf_id) == 0) {
            if (time_us_64() > t) {  // timed out
                if (i <= 0) {
                    *errcode = MP_EAGAIN;
                    return MP_STREAM_ERROR;
                } else {
                    return i;
                }
            }
            MICROPY_EVENT_POLL_HOOK
        }
        *dest++ = tud_cdc_n_read_char(self->itf_id);
        t = time_us_64() + timeout_char_us;
    }
    return size;
}

STATIC mp_uint_t usbcdc_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode) {
    usbcdc_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint64_t t = time_us_64() + (uint64_t)self->timeout * 1000;
    uint64_t timeout_char_us = (uint64_t)self->timeout_char * 1000;
    const uint8_t *src = buf_in;
    size_t i = 0;

    // Put as many bytes as possible into the transmit buffer.
    while (i < size && tud_cdc_n_write_available(self->itf_id) > 0) {
        tud_cdc_n_write_char(self->itf_id, *src++);
        ++i;
    }

    // Send the next characters while busy waiting.
    while (i < size) {
        // Wait for the first/next character to be sent.
        while (tud_cdc_n_write_available(self->itf_id) == 0) {
            if (time_us_64() > t) {  // timed out
                if (i <= 0) {
                    *errcode = MP_EAGAIN;
                    return MP_STREAM_ERROR;
                } else {
                    return i;
                }
            }
            MICROPY_EVENT_POLL_HOOK
        }
        tud_cdc_n_write_char(self->itf_id, *src++);
        ++i;
        t = time_us_64() + timeout_char_us;
    }
    tud_cdc_n_write_flush(self->itf_id);

    return size;
}

STATIC mp_uint_t usbcdc_ioctl(mp_obj_t self_in, mp_uint_t request, mp_uint_t arg, int *errcode) {
    usbcdc_obj_t *self = self_in;
    mp_uint_t ret;
    if (request == MP_STREAM_POLL) {
        uintptr_t flags = arg;
        ret = 0;
        if ((flags & MP_STREAM_POLL_RD) && tud_cdc_n_available(self->itf_id) > 0) {
            ret |= MP_STREAM_POLL_RD;
        }
        if ((flags & MP_STREAM_POLL_WR) && tud_cdc_n_write_available(self->itf_id) > 0) {
            ret |= MP_STREAM_POLL_WR;
        }
    } else {
        *errcode = MP_EINVAL;
        ret = MP_STREAM_ERROR;
    }
    return ret;
}

STATIC const mp_stream_p_t cdc_stream_p = {
    .read = usbcdc_read,
    .write = usbcdc_write,
    .ioctl = usbcdc_ioctl,
    .is_text = false,
};

const mp_obj_type_t usbcdc_type = {
    { &mp_type_type },
    .name = MP_QSTR_usbcdc,
    .print = usbcdc_print,
    .make_new = usbcdc_make_new,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &cdc_stream_p,
    .locals_dict = (mp_obj_dict_t *)&usbcdc_locals_dict,
};

STATIC const mp_rom_map_elem_t usbcdc_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_usbcdc) },
    { MP_ROM_QSTR(MP_QSTR_usbcdc), MP_ROM_PTR(&usbcdc_type) },
};
STATIC MP_DEFINE_CONST_DICT(usbcdc_module_globals, usbcdc_module_globals_table);

const mp_obj_module_t mp_module_usbcdc = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&usbcdc_module_globals,
};
