/*
 * Adjustable speed for keyboard-driven cursor movement and scrolling.
 *
 * The input processor scales REL_X/REL_Y by the cursor speed level,
 * REL_WHEEL/REL_HWHEEL by the scroll speed level, and both by its own mul/div
 * params, and flips REL_WHEEL when scroll reversal is on. The speed behavior
 * steps the cursor speed, scroll speed, jump step or jump delay level up or
 * down, or toggles scroll reversal. The jump behavior moves the cursor in
 * steps while held. All settings are kept per BLE profile and persisted.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/settings/settings.h>
#include <drivers/behavior.h>
#include <drivers/input_processor.h>

#include <zmk/behavior.h>
#include <zmk/ble.h>

#include <dt-bindings/zmk/mouse_speed.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mouse_speed, CONFIG_INPUT_LOG_LEVEL);

// Each step is roughly 1.4x the previous one
static const uint16_t level_percents[] = {25, 35, 50, 70, 100, 140, 200, 280, 400};
#define DEFAULT_LEVEL 4

// Vertical jump step in pixels; horizontal steps are scaled by the jump node
static const uint16_t jump_steps[] = {25, 50, 75, 100, 150, 200, 300, 400, 600};
#define DEFAULT_JUMP_LEVEL 4

// Pause between repeated jumps while the key stays held
static const uint16_t jump_delays_ms[] = {80, 120, 160, 200, 250, 300, 400, 500, 700};
#define DEFAULT_JUMP_DELAY_LEVEL 3

#if IS_ENABLED(CONFIG_ZMK_BLE)
#define PROFILE_COUNT ZMK_BLE_PROFILE_COUNT
#else
#define PROFILE_COUNT 1
#endif

static uint8_t levels[PROFILE_COUNT];
static uint8_t scroll_levels[PROFILE_COUNT];
static uint8_t jump_levels[PROFILE_COUNT];
static uint8_t jump_delay_levels[PROFILE_COUNT];
static uint8_t scroll_reversed[PROFILE_COUNT];

// Every per-profile setting with its number of valid values and default
static const struct {
    const char *name;
    uint8_t *values;
    uint8_t count;
    uint8_t def;
} stored[] = {
    {"levels", levels, ARRAY_SIZE(level_percents), DEFAULT_LEVEL},
    {"scroll_levels", scroll_levels, ARRAY_SIZE(level_percents), DEFAULT_LEVEL},
    {"jump_levels", jump_levels, ARRAY_SIZE(jump_steps), DEFAULT_JUMP_LEVEL},
    {"jump_delay_levels", jump_delay_levels, ARRAY_SIZE(jump_delays_ms),
     DEFAULT_JUMP_DELAY_LEVEL},
    {"scroll_reversed", scroll_reversed, 2, 0},
};

static int active_slot(void) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    int index = zmk_ble_active_profile_index();
    if (index >= 0 && index < PROFILE_COUNT) {
        return index;
    }
#endif
    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)

static void save_work_cb(struct k_work *work) {
    char key[40];
    for (int i = 0; i < ARRAY_SIZE(stored); i++) {
        snprintf(key, sizeof(key), "mouse_speed/%s", stored[i].name);
        settings_save_one(key, stored[i].values, PROFILE_COUNT);
    }
}

static K_WORK_DELAYABLE_DEFINE(save_work, save_work_cb);

static int mouse_speed_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    int index = -1;
    for (int i = 0; i < ARRAY_SIZE(stored); i++) {
        if (settings_name_steq(name, stored[i].name, NULL)) {
            index = i;
            break;
        }
    }
    if (index < 0) {
        return -ENOENT;
    }

    // Ignore stored data saved with a different profile count
    if (len != PROFILE_COUNT) {
        return 0;
    }

    uint8_t *target = stored[index].values;
    int err = read_cb(cb_arg, target, PROFILE_COUNT);
    if (err < 0) {
        LOG_ERR("Failed to read mouse speed setting %s (err %d)", name, err);
        return err;
    }

    for (int i = 0; i < PROFILE_COUNT; i++) {
        if (target[i] >= stored[index].count) {
            target[i] = stored[index].def;
        }
    }

    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mouse_speed, "mouse_speed", NULL, mouse_speed_settings_set, NULL,
                               NULL);

#endif // IS_ENABLED(CONFIG_SETTINGS)

static int mouse_speed_init(const struct device *dev) {
    for (int i = 0; i < ARRAY_SIZE(stored); i++) {
        memset(stored[i].values, stored[i].def, PROFILE_COUNT);
    }
    return 0;
}

// Input processor

// Indexed by axis: REL_X, REL_Y, REL_HWHEEL, REL_WHEEL
static int16_t remainders[4];

static int mouse_speed_handle_event(const struct device *dev, struct input_event *event,
                                    uint32_t param1, uint32_t param2,
                                    struct zmk_input_processor_state *state) {
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int axis;
    switch (event->code) {
    case INPUT_REL_X:
        axis = 0;
        break;
    case INPUT_REL_Y:
        axis = 1;
        break;
    case INPUT_REL_HWHEEL:
        axis = 2;
        break;
    case INPUT_REL_WHEEL:
        axis = 3;
        break;
    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int slot = active_slot();
    uint8_t *slot_levels = axis < 2 ? levels : scroll_levels;
    int32_t mul = level_percents[slot_levels[slot]] * param1;
    int32_t div = 100 * param2;

    if (axis == 3 && scroll_reversed[slot]) {
        mul = -mul;
    }

    int32_t value = event->value * mul + remainders[axis];
    int32_t scaled = value / div;
    remainders[axis] = value - scaled * div;

    event->value = CLAMP(scaled, INT16_MIN, INT16_MAX);

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api mouse_speed_processor_api = {
    .handle_event = mouse_speed_handle_event,
};

DEVICE_DT_DEFINE(DT_INST(0, zmk_input_processor_mouse_speed), mouse_speed_init, NULL, NULL, NULL,
                 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mouse_speed_processor_api);

// Speed behavior

static void schedule_save(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_reschedule(&save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
#endif
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    int slot = active_slot();
    int index;
    int step;

    switch (binding->param1) {
    case MSPD_INC:
    case MSPD_DEC:
        index = 0;
        step = binding->param1 == MSPD_INC ? 1 : -1;
        break;
    case MSPD_SCRL_INC:
    case MSPD_SCRL_DEC:
        index = 1;
        step = binding->param1 == MSPD_SCRL_INC ? 1 : -1;
        break;
    case MSPD_JUMP_INC:
    case MSPD_JUMP_DEC:
        index = 2;
        step = binding->param1 == MSPD_JUMP_INC ? 1 : -1;
        break;
    case MSPD_DELAY_INC:
    case MSPD_DELAY_DEC:
        index = 3;
        step = binding->param1 == MSPD_DELAY_INC ? 1 : -1;
        break;
    case MSPD_SCRL_REV:
        scroll_reversed[slot] = !scroll_reversed[slot];
        LOG_DBG("Profile %d scroll reversed %d", slot, scroll_reversed[slot]);
        schedule_save();
        return ZMK_BEHAVIOR_OPAQUE;
    default:
        LOG_ERR("Unknown mouse speed command: %d", binding->param1);
        return -ENOTSUP;
    }

    uint8_t *values = stored[index].values;
    int level = CLAMP(values[slot] + step, 0, stored[index].count - 1);
    if (level == values[slot]) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    values[slot] = level;
    LOG_DBG("Profile %d %s %d", slot, stored[index].name, level);

    schedule_save();

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api mouse_speed_behavior_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

BEHAVIOR_DT_DEFINE(DT_INST(0, zmk_behavior_mouse_speed), NULL, NULL, NULL, NULL, POST_KERNEL,
                   CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mouse_speed_behavior_api);

// Jump behavior

#if DT_HAS_COMPAT_STATUS_OKAY(zmk_behavior_mouse_jump)

#define JUMP_NODE DT_INST(0, zmk_behavior_mouse_jump)

// The behavior device doubles as the input device its listener reads from
static const struct device *jump_dev;
static uint8_t held_directions;

static k_timeout_t jump_delay(void) {
    return K_MSEC(jump_delays_ms[jump_delay_levels[active_slot()]]);
}

// Holding two directions adds them up into a diagonal step
static void jump_emit(uint8_t directions) {
    int32_t step_y = jump_steps[jump_levels[active_slot()]];
    int32_t step_x =
        step_y * DT_PROP(JUMP_NODE, horizontal_mul) / DT_PROP(JUMP_NODE, horizontal_div);
    int32_t dx = 0;
    int32_t dy = 0;

    if (directions & BIT(MJMP_LEFT)) {
        dx -= step_x;
    }
    if (directions & BIT(MJMP_RIGHT)) {
        dx += step_x;
    }
    if (directions & BIT(MJMP_UP)) {
        dy -= step_y;
    }
    if (directions & BIT(MJMP_DOWN)) {
        dy += step_y;
    }

    if (dx != 0) {
        input_report_rel(jump_dev, INPUT_REL_X, CLAMP(dx, INT16_MIN, INT16_MAX), dy == 0,
                         K_NO_WAIT);
    }
    if (dy != 0) {
        input_report_rel(jump_dev, INPUT_REL_Y, CLAMP(dy, INT16_MIN, INT16_MAX), true, K_NO_WAIT);
    }
}

static void jump_work_cb(struct k_work *work) {
    if (held_directions == 0) {
        return;
    }

    jump_emit(held_directions);
    k_work_schedule(k_work_delayable_from_work(work), jump_delay());
}

static K_WORK_DELAYABLE_DEFINE(jump_work, jump_work_cb);

static int mouse_jump_init(const struct device *dev) {
    jump_dev = dev;
    return 0;
}

static int on_jump_binding_pressed(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event) {
    if (binding->param1 > MJMP_RIGHT) {
        LOG_ERR("Unknown mouse jump direction: %d", binding->param1);
        return -ENOTSUP;
    }

    bool idle = held_directions == 0;
    held_directions |= BIT(binding->param1);

    // The first step goes out right away; repeats follow the shared timer
    jump_emit(BIT(binding->param1));
    if (idle) {
        k_work_reschedule(&jump_work, jump_delay());
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_jump_binding_released(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event) {
    if (binding->param1 > MJMP_RIGHT) {
        return -ENOTSUP;
    }

    held_directions &= ~BIT(binding->param1);
    if (held_directions == 0) {
        k_work_cancel_delayable(&jump_work);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api mouse_jump_behavior_api = {
    .binding_pressed = on_jump_binding_pressed,
    .binding_released = on_jump_binding_released,
};

BEHAVIOR_DT_DEFINE(JUMP_NODE, mouse_jump_init, NULL, NULL, NULL, POST_KERNEL,
                   CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mouse_jump_behavior_api);

#endif // DT_HAS_COMPAT_STATUS_OKAY(zmk_behavior_mouse_jump)
