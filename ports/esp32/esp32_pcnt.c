/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021-22 Jonathan Hogg
 * Copyright (c) 2026 OpenAI
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
#include "py/obj.h"
#include "py/objexcept.h"

#if MICROPY_PY_ESP32_PCNT

#include "shared/runtime/mpirq.h"

#include "driver/pulse_cnt.h"
#include "esp_idf_version.h"
#include "hal/pcnt_ll.h"
#include "modesp32.h"

#if !MICROPY_ENABLE_FINALISER
#error "esp32.PCNT requires MICROPY_ENABLE_FINALISER."
#endif

#define ESP32_PCNT_CHANNEL_COUNT PCNT_LL_GET(CHANS_PER_UNIT)
#define ESP32_PCNT_UNIT_COUNT    PCNT_LL_GET(UNITS_PER_INST)

// 与旧版 driver/pcnt.h 兼容的事件位定义，保持 Python 侧可按位或组合触发条件。
#define ESP32_PCNT_EVT_THRES_1 (1u << 0)
#define ESP32_PCNT_EVT_THRES_0 (1u << 1)
#define ESP32_PCNT_EVT_L_LIM   (1u << 2)
#define ESP32_PCNT_EVT_H_LIM   (1u << 3)
#define ESP32_PCNT_EVT_ZERO    (1u << 4)
#define ESP32_PCNT_EVT_ALL     (ESP32_PCNT_EVT_THRES_1 | ESP32_PCNT_EVT_THRES_0 | ESP32_PCNT_EVT_L_LIM | ESP32_PCNT_EVT_H_LIM | ESP32_PCNT_EVT_ZERO)

typedef struct _esp32_pcnt_irq_obj_t {
    mp_irq_obj_t base;
    uint32_t flags;
    uint32_t trigger;
} esp32_pcnt_irq_obj_t;

typedef struct _esp32_pcnt_obj_t {
    mp_obj_base_t base;
    mp_int_t unit_id; // Python 暴露的逻辑 unit id，不等同于底层实际分配顺序。
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channels[ESP32_PCNT_CHANNEL_COUNT];
    int pin[ESP32_PCNT_CHANNEL_COUNT];
    int mode_pin[ESP32_PCNT_CHANNEL_COUNT];
    uint8_t rising[ESP32_PCNT_CHANNEL_COUNT];
    uint8_t falling[ESP32_PCNT_CHANNEL_COUNT];
    uint8_t mode_low[ESP32_PCNT_CHANNEL_COUNT];
    uint8_t mode_high[ESP32_PCNT_CHANNEL_COUNT];
    int low_limit;
    int high_limit;
    bool threshold_enabled[2];
    int threshold[2];
    uint16_t filter_cycles;
    bool unit_enabled;
    esp32_pcnt_irq_obj_t *irq;
    struct _esp32_pcnt_obj_t *next;
} esp32_pcnt_obj_t;

// Linked list of PCNT units.
MP_REGISTER_ROOT_POINTER(struct _esp32_pcnt_obj_t *esp32_pcnt_obj_head);

static mp_obj_t esp32_pcnt_deinit(mp_obj_t self_in);

static inline pcnt_channel_edge_action_t esp32_pcnt_get_edge_action(mp_int_t value) {
    if (value < PCNT_CHANNEL_EDGE_ACTION_HOLD || value > PCNT_CHANNEL_EDGE_ACTION_DECREASE) {
        mp_raise_ValueError(MP_ERROR_TEXT("rising/falling"));
    }
    return (pcnt_channel_edge_action_t)value;
}

static inline pcnt_channel_level_action_t esp32_pcnt_get_level_action(mp_int_t value) {
    if (value < PCNT_CHANNEL_LEVEL_ACTION_KEEP || value > PCNT_CHANNEL_LEVEL_ACTION_HOLD) {
        mp_raise_ValueError(MP_ERROR_TEXT("mode_low/mode_high"));
    }
    return (pcnt_channel_level_action_t)value;
}

static void esp32_pcnt_disable_events_for_unit(esp32_pcnt_obj_t *self) {
    if (!self->irq) {
        return;
    }
    self->irq->flags = 0;
    self->irq->base.handler = mp_const_none;
    self->irq->trigger = 0;
}

static void esp32_pcnt_release_unit(esp32_pcnt_obj_t *self) {
    if (self->unit == NULL) {
        return;
    }

    if (self->unit_enabled) {
        esp_err_t err = pcnt_unit_stop(self->unit);
        if (err != ESP_ERR_INVALID_STATE) {
            check_esp_err(err);
        }
        check_esp_err(pcnt_unit_disable(self->unit));
        self->unit_enabled = false;
    }

    for (size_t channel = 0; channel < ESP32_PCNT_CHANNEL_COUNT; ++channel) {
        if (self->channels[channel] != NULL) {
            check_esp_err(pcnt_del_channel(self->channels[channel]));
            self->channels[channel] = NULL;
        }
    }

    check_esp_err(pcnt_del_unit(self->unit));
    self->unit = NULL;
}

static void esp32_pcnt_add_watch_point(esp32_pcnt_obj_t *self, int value) {
    esp_err_t err = pcnt_unit_add_watch_point(self->unit, value);
    // 相同值的监视点在新驱动下会返回 INVALID_STATE，这里按“已存在”处理。
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        check_esp_err(err);
    }
}

static void esp32_pcnt_apply_filter(esp32_pcnt_obj_t *self) {
    if (self->filter_cycles == 0) {
        check_esp_err(pcnt_unit_set_glitch_filter(self->unit, NULL));
        return;
    }

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = ((uint32_t)self->filter_cycles * 1000 + 79) / 80,
    };
    if (filter_cfg.max_glitch_ns == 0) {
        filter_cfg.max_glitch_ns = 1;
    }
    check_esp_err(pcnt_unit_set_glitch_filter(self->unit, &filter_cfg));
}

static void esp32_pcnt_apply_channel(esp32_pcnt_obj_t *self, size_t channel) {
    if (self->channels[channel] != NULL) {
        check_esp_err(pcnt_del_channel(self->channels[channel]));
        self->channels[channel] = NULL;
    }

    if (self->pin[channel] < 0) {
        return;
    }

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num = self->pin[channel],
        .level_gpio_num = self->mode_pin[channel],
    };
    check_esp_err(pcnt_new_channel(self->unit, &chan_cfg, &self->channels[channel]));
    check_esp_err(pcnt_channel_set_edge_action(
        self->channels[channel],
        (pcnt_channel_edge_action_t)self->rising[channel],
        (pcnt_channel_edge_action_t)self->falling[channel]));
    check_esp_err(pcnt_channel_set_level_action(
        self->channels[channel],
        (pcnt_channel_level_action_t)self->mode_high[channel],
        (pcnt_channel_level_action_t)self->mode_low[channel]));
}

static bool IRAM_ATTR esp32_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    (void)unit;
    esp32_pcnt_obj_t *self = (esp32_pcnt_obj_t *)user_ctx;
    if (self->irq == NULL) {
        return false;
    }

    uint32_t status = 0;
    int value = edata->watch_point_value;

    if (value == self->high_limit) {
        status |= ESP32_PCNT_EVT_H_LIM;
    }
    if (value == self->low_limit) {
        status |= ESP32_PCNT_EVT_L_LIM;
    }
    if (value == 0) {
        status |= ESP32_PCNT_EVT_ZERO;
    }
    if (self->threshold_enabled[0] && value == self->threshold[0]) {
        status |= ESP32_PCNT_EVT_THRES_0;
    }
    if (self->threshold_enabled[1] && value == self->threshold[1]) {
        status |= ESP32_PCNT_EVT_THRES_1;
    }

    if (status == 0) {
        return false;
    }

    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    self->irq->flags |= status;
    MICROPY_END_ATOMIC_SECTION(atomic_state);

    if (self->irq->base.handler != mp_const_none && (status & self->irq->trigger)) {
        mp_irq_handler(&self->irq->base);
    }

    return false;
}

static void esp32_pcnt_rebuild_unit(esp32_pcnt_obj_t *self) {
    esp32_pcnt_release_unit(self);

    pcnt_unit_config_t unit_cfg = {
        .low_limit = self->low_limit,
        .high_limit = self->high_limit,
        .intr_priority = 0,
    };
    check_esp_err(pcnt_new_unit(&unit_cfg, &self->unit));

    pcnt_event_callbacks_t callbacks = {
        .on_reach = esp32_pcnt_on_reach,
    };
    check_esp_err(pcnt_unit_register_event_callbacks(self->unit, &callbacks, self));

    for (size_t channel = 0; channel < ESP32_PCNT_CHANNEL_COUNT; ++channel) {
        esp32_pcnt_apply_channel(self, channel);
    }

    esp32_pcnt_apply_filter(self);
    esp32_pcnt_add_watch_point(self, self->low_limit);
    esp32_pcnt_add_watch_point(self, self->high_limit);
    esp32_pcnt_add_watch_point(self, 0);
    if (self->threshold_enabled[0]) {
        esp32_pcnt_add_watch_point(self, self->threshold[0]);
    }
    if (self->threshold_enabled[1]) {
        esp32_pcnt_add_watch_point(self, self->threshold[1]);
    }

    if (self->irq) {
        self->irq->flags = 0;
    }
    self->unit_enabled = false;
}

void esp32_pcnt_deinit_all(void) {
    esp32_pcnt_obj_t **pcnt = &MP_STATE_PORT(esp32_pcnt_obj_head);
    while (*pcnt != NULL) {
        esp32_pcnt_deinit(MP_OBJ_FROM_PTR(*pcnt));
        *pcnt = (*pcnt)->next;
    }
}

static void esp32_pcnt_init_defaults(esp32_pcnt_obj_t *self) {
    self->unit = NULL;
    self->unit_enabled = false;
    self->low_limit = -32768;
    self->high_limit = 32767;
    self->threshold_enabled[0] = false;
    self->threshold_enabled[1] = false;
    self->threshold[0] = 0;
    self->threshold[1] = 0;
    self->filter_cycles = 0;
    for (size_t channel = 0; channel < ESP32_PCNT_CHANNEL_COUNT; ++channel) {
        self->channels[channel] = NULL;
        self->pin[channel] = -1;
        self->mode_pin[channel] = -1;
        self->rising[channel] = PCNT_CHANNEL_EDGE_ACTION_HOLD;
        self->falling[channel] = PCNT_CHANNEL_EDGE_ACTION_HOLD;
        self->mode_low[channel] = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
        self->mode_high[channel] = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
    }
}

static void esp32_pcnt_init_helper(esp32_pcnt_obj_t *self, size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_channel,
        ARG_pin,
        ARG_rising,
        ARG_falling,
        ARG_mode_pin,
        ARG_mode_low,
        ARG_mode_high,
        ARG_min,
        ARG_max,
        ARG_filter,
        ARG_threshold0,
        ARG_threshold1,
        ARG_value,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel,     MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int = 0} },
        { MP_QSTR_pin,         MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_rising,      MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_falling,     MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mode_pin,    MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mode_low,    MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mode_high,   MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_min,         MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_max,         MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_filter,      MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_threshold0,  MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_threshold1,  MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_value,       MP_ARG_KW_ONLY | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_uint_t channel = args[ARG_channel].u_int;
    if (channel >= ESP32_PCNT_CHANNEL_COUNT) {
        mp_raise_ValueError(MP_ERROR_TEXT("channel"));
    }

    if (args[ARG_mode_pin].u_obj != MP_OBJ_NULL && args[ARG_pin].u_obj == MP_OBJ_NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("pin"));
    }

    if (args[ARG_pin].u_obj != MP_OBJ_NULL) {
        if (args[ARG_pin].u_obj == mp_const_none) {
            self->pin[channel] = -1;
            self->mode_pin[channel] = -1;
        } else {
            self->pin[channel] = (int)mp_hal_get_pin_obj(args[ARG_pin].u_obj);
            if (args[ARG_mode_pin].u_obj == MP_OBJ_NULL || args[ARG_mode_pin].u_obj == mp_const_none) {
                self->mode_pin[channel] = -1;
            } else {
                self->mode_pin[channel] = (int)mp_hal_get_pin_obj(args[ARG_mode_pin].u_obj);
            }
        }
    }

    if (args[ARG_rising].u_obj != MP_OBJ_NULL) {
        self->rising[channel] = esp32_pcnt_get_edge_action(mp_obj_get_int(args[ARG_rising].u_obj));
    }
    if (args[ARG_falling].u_obj != MP_OBJ_NULL) {
        self->falling[channel] = esp32_pcnt_get_edge_action(mp_obj_get_int(args[ARG_falling].u_obj));
    }
    if (args[ARG_mode_low].u_obj != MP_OBJ_NULL) {
        self->mode_low[channel] = esp32_pcnt_get_level_action(mp_obj_get_int(args[ARG_mode_low].u_obj));
    }
    if (args[ARG_mode_high].u_obj != MP_OBJ_NULL) {
        self->mode_high[channel] = esp32_pcnt_get_level_action(mp_obj_get_int(args[ARG_mode_high].u_obj));
    }

    if (args[ARG_filter].u_obj != MP_OBJ_NULL) {
        mp_int_t filter = mp_obj_get_int(args[ARG_filter].u_obj);
        if (filter < 0 || filter > 1023) {
            mp_raise_ValueError(MP_ERROR_TEXT("filter"));
        }
        self->filter_cycles = (uint16_t)filter;
    }

    if (args[ARG_min].u_obj != MP_OBJ_NULL) {
        mp_int_t minimum = mp_obj_get_int(args[ARG_min].u_obj);
        if (minimum < -32768 || minimum > 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("minimum"));
        }
        self->low_limit = minimum;
    }

    if (args[ARG_max].u_obj != MP_OBJ_NULL) {
        mp_int_t maximum = mp_obj_get_int(args[ARG_max].u_obj);
        if (maximum < 0 || maximum > 32767) {
            mp_raise_ValueError(MP_ERROR_TEXT("maximum"));
        }
        self->high_limit = maximum;
    }

    if (args[ARG_threshold0].u_obj != MP_OBJ_NULL) {
        self->threshold_enabled[0] = true;
        self->threshold[0] = mp_obj_get_int(args[ARG_threshold0].u_obj);
    }

    if (args[ARG_threshold1].u_obj != MP_OBJ_NULL) {
        self->threshold_enabled[1] = true;
        self->threshold[1] = mp_obj_get_int(args[ARG_threshold1].u_obj);
    }

    if (args[ARG_value].u_obj != MP_OBJ_NULL && mp_obj_get_int(args[ARG_value].u_obj) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("value"));
    }

    if (self->low_limit == 0 && self->high_limit == 0) {
        // 新驱动要求 limit 覆盖正负两个方向，兼容旧接口时给出最小有效窗口。
        self->low_limit = -1;
        self->high_limit = 1;
    } else {
        if (self->low_limit >= 0) {
            self->low_limit = -1;
        }
        if (self->high_limit <= 0) {
            self->high_limit = 1;
        }
    }

    esp32_pcnt_rebuild_unit(self);
}

static mp_obj_t esp32_pcnt_make_new(const mp_obj_type_t *type, size_t n_pos_args, size_t n_kw_args, const mp_obj_t *args) {
    if (n_pos_args < 1) {
        mp_raise_TypeError(MP_ERROR_TEXT("id"));
    }

    mp_int_t unit_id = mp_obj_get_int(args[0]);
    if (unit_id < 0 || unit_id >= ESP32_PCNT_UNIT_COUNT) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid id"));
    }

    esp32_pcnt_obj_t *self = MP_STATE_PORT(esp32_pcnt_obj_head);
    while (self) {
        if (self->unit_id == unit_id) {
            break;
        }
        self = self->next;
    }

    if (!self) {
        self = mp_obj_malloc(esp32_pcnt_obj_t, &esp32_pcnt_type);
        self->unit_id = unit_id;
        self->irq = NULL;
        self->next = MP_STATE_PORT(esp32_pcnt_obj_head);
        MP_STATE_PORT(esp32_pcnt_obj_head) = self;
        esp32_pcnt_init_defaults(self);
    }

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw_args, args + n_pos_args);
    esp32_pcnt_init_helper(self, 0, args + n_pos_args, &kw_args);

    return MP_OBJ_FROM_PTR(self);
}

static void esp32_pcnt_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "PCNT(%d)", (int)self->unit_id);
}

static mp_obj_t esp32_pcnt_init(size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    esp32_pcnt_init_helper(self, n_pos_args - 1, pos_args + 1, kw_args);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_pcnt_init_obj, 1, esp32_pcnt_init);

static mp_obj_t esp32_pcnt_deinit(mp_obj_t self_in) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(self_in);
    esp32_pcnt_release_unit(self);
    esp32_pcnt_disable_events_for_unit(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_pcnt_deinit_obj, esp32_pcnt_deinit);

static mp_obj_t esp32_pcnt_value(size_t n_args, const mp_obj_t *pos_args) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    if (self->unit == NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialised"));
    }

    if (n_args == 2 && mp_obj_get_int(pos_args[1]) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("value"));
    }

    int value;
    while (true) {
        check_esp_err(pcnt_unit_get_count(self->unit, &value));
        if (self->irq && self->irq->flags && self->irq->base.handler != mp_const_none) {
            mp_call_function_1(self->irq->base.handler, self->irq->base.parent);
        } else {
            break;
        }
    }

    if (n_args == 2) {
        check_esp_err(pcnt_unit_clear_count(self->unit));
    }

    return MP_OBJ_NEW_SMALL_INT(value);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp32_pcnt_value_obj, 1, 2, esp32_pcnt_value);

static mp_uint_t esp32_pcnt_irq_trigger(mp_obj_t self_in, mp_uint_t new_trigger) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->irq->trigger = new_trigger & ESP32_PCNT_EVT_ALL;
    return 0;
}

static mp_uint_t esp32_pcnt_irq_info(mp_obj_t self_in, mp_uint_t info_type) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (info_type == MP_IRQ_INFO_FLAGS) {
        mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
        mp_uint_t flags = self->irq->flags;
        self->irq->flags = 0;
        MICROPY_END_ATOMIC_SECTION(atomic_state);
        return flags;
    } else if (info_type == MP_IRQ_INFO_TRIGGERS) {
        return self->irq->trigger;
    }
    return 0;
}

static const mp_irq_methods_t esp32_pcnt_irq_methods = {
    .trigger = esp32_pcnt_irq_trigger,
    .info = esp32_pcnt_irq_info,
};

static mp_obj_t esp32_pcnt_irq(size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_handler, ARG_trigger };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler,  MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_trigger,  MP_ARG_INT, {.u_int = ESP32_PCNT_EVT_ZERO} },
    };

    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!self->irq) {
        self->irq = mp_obj_malloc(esp32_pcnt_irq_obj_t, &mp_irq_type);
        self->irq->base.methods = (mp_irq_methods_t *)&esp32_pcnt_irq_methods;
        self->irq->base.parent = MP_OBJ_FROM_PTR(self);
        self->irq->base.ishard = false;
        self->irq->base.handler = mp_const_none;
        self->irq->flags = 0;
        self->irq->trigger = 0;
    }

    if (n_pos_args > 1 || kw_args->used != 0) {
        mp_obj_t handler = args[ARG_handler].u_obj;
        mp_uint_t trigger = args[ARG_trigger].u_int;

        if (handler != mp_const_none) {
            if (trigger == 0 || (trigger & ~ESP32_PCNT_EVT_ALL) != 0) {
                mp_raise_ValueError(MP_ERROR_TEXT("trigger"));
            }
            self->irq->base.handler = handler;
            self->irq->trigger = trigger;
        } else {
            esp32_pcnt_disable_events_for_unit(self);
        }
    }

    return MP_OBJ_FROM_PTR(self->irq);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(esp32_pcnt_irq_obj, 1, esp32_pcnt_irq);

static mp_obj_t esp32_pcnt_start(mp_obj_t self_in) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->unit == NULL) {
        esp32_pcnt_rebuild_unit(self);
    }
    if (!self->unit_enabled) {
        check_esp_err(pcnt_unit_enable(self->unit));
        self->unit_enabled = true;
    }
    check_esp_err(pcnt_unit_start(self->unit));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_pcnt_start_obj, esp32_pcnt_start);

static mp_obj_t esp32_pcnt_stop(mp_obj_t self_in) {
    esp32_pcnt_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->unit != NULL && self->unit_enabled) {
        check_esp_err(pcnt_unit_stop(self->unit));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_pcnt_stop_obj, esp32_pcnt_stop);

static const mp_rom_map_elem_t esp32_pcnt_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_init),            MP_ROM_PTR(&esp32_pcnt_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_value),           MP_ROM_PTR(&esp32_pcnt_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq),             MP_ROM_PTR(&esp32_pcnt_irq_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),           MP_ROM_PTR(&esp32_pcnt_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),            MP_ROM_PTR(&esp32_pcnt_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&esp32_pcnt_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__),         MP_ROM_PTR(&esp32_pcnt_deinit_obj) },

    // Constants
    { MP_ROM_QSTR(MP_QSTR_IGNORE),          MP_ROM_INT(PCNT_CHANNEL_EDGE_ACTION_HOLD) },
    { MP_ROM_QSTR(MP_QSTR_INCREMENT),       MP_ROM_INT(PCNT_CHANNEL_EDGE_ACTION_INCREASE) },
    { MP_ROM_QSTR(MP_QSTR_DECREMENT),       MP_ROM_INT(PCNT_CHANNEL_EDGE_ACTION_DECREASE) },
    { MP_ROM_QSTR(MP_QSTR_NORMAL),          MP_ROM_INT(PCNT_CHANNEL_LEVEL_ACTION_KEEP) },
    { MP_ROM_QSTR(MP_QSTR_REVERSE),         MP_ROM_INT(PCNT_CHANNEL_LEVEL_ACTION_INVERSE) },
    { MP_ROM_QSTR(MP_QSTR_HOLD),            MP_ROM_INT(PCNT_CHANNEL_LEVEL_ACTION_HOLD) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_ZERO),        MP_ROM_INT(ESP32_PCNT_EVT_ZERO) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_THRESHOLD0),  MP_ROM_INT(ESP32_PCNT_EVT_THRES_0) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_THRESHOLD1),  MP_ROM_INT(ESP32_PCNT_EVT_THRES_1) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_MIN),         MP_ROM_INT(ESP32_PCNT_EVT_L_LIM) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_MAX),         MP_ROM_INT(ESP32_PCNT_EVT_H_LIM) },
};
static MP_DEFINE_CONST_DICT(esp32_pcnt_locals_dict, esp32_pcnt_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    esp32_pcnt_type,
    MP_QSTR_PCNT,
    MP_TYPE_FLAG_NONE,
    make_new, esp32_pcnt_make_new,
    print, esp32_pcnt_print,
    locals_dict, &esp32_pcnt_locals_dict
    );

#endif // MICROPY_PY_ESP32_PCNT
