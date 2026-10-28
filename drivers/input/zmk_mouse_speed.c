/*
 * Adjustable speed for keyboard-driven cursor movement and scrolling.
 *
 * The input processor scales REL_X/REL_Y by the cursor speed level,
 * REL_WHEEL/REL_HWHEEL by the scroll speed level, and both by its own mul/div
 * params. The behavior steps either level up or down. Levels are kept per BLE
 * profile and persisted in settings.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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

#if IS_ENABLED(CONFIG_ZMK_BLE)
#define PROFILE_COUNT ZMK_BLE_PROFILE_COUNT
#else
#define PROFILE_COUNT 1
#endif

static uint8_t levels[PROFILE_COUNT];
static uint8_t scroll_levels[PROFILE_COUNT];

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
    settings_save_one("mouse_speed/levels", levels, sizeof(levels));
    settings_save_one("mouse_speed/scroll_levels", scroll_levels, sizeof(scroll_levels));
}

static K_WORK_DELAYABLE_DEFINE(save_work, save_work_cb);

static int mouse_speed_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    uint8_t *target;
    if (settings_name_steq(name, "levels", NULL)) {
        target = levels;
    } else if (settings_name_steq(name, "scroll_levels", NULL)) {
        target = scroll_levels;
    } else {
        return -ENOENT;
    }

    // Ignore stored data saved with a different profile count
    if (len != PROFILE_COUNT) {
        return 0;
    }

    int err = read_cb(cb_arg, target, PROFILE_COUNT);
    if (err < 0) {
        LOG_ERR("Failed to read mouse speed levels (err %d)", err);
        return err;
    }

    for (int i = 0; i < PROFILE_COUNT; i++) {
        if (target[i] >= ARRAY_SIZE(level_percents)) {
            target[i] = DEFAULT_LEVEL;
        }
    }

    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mouse_speed, "mouse_speed", NULL, mouse_speed_settings_set, NULL,
                               NULL);

#endif // IS_ENABLED(CONFIG_SETTINGS)

static int mouse_speed_init(const struct device *dev) {
    for (int i = 0; i < PROFILE_COUNT; i++) {
        levels[i] = DEFAULT_LEVEL;
        scroll_levels[i] = DEFAULT_LEVEL;
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

    uint8_t *slot_levels = axis < 2 ? levels : scroll_levels;
    int32_t mul = level_percents[slot_levels[active_slot()]] * param1;
    int32_t div = 100 * param2;

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

// Behavior

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    int slot = active_slot();
    uint8_t *slot_levels;
    int step;

    switch (binding->param1) {
    case MSPD_INC:
        slot_levels = levels;
        step = 1;
        break;
    case MSPD_DEC:
        slot_levels = levels;
        step = -1;
        break;
    case MSPD_SCRL_INC:
        slot_levels = scroll_levels;
        step = 1;
        break;
    case MSPD_SCRL_DEC:
        slot_levels = scroll_levels;
        step = -1;
        break;
    default:
        LOG_ERR("Unknown mouse speed command: %d", binding->param1);
        return -ENOTSUP;
    }

    int level = CLAMP(slot_levels[slot] + step, 0, (int)ARRAY_SIZE(level_percents) - 1);
    if (level == slot_levels[slot]) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    slot_levels[slot] = level;
    LOG_DBG("Profile %d %s speed %d%%", slot, slot_levels == levels ? "cursor" : "scroll",
            level_percents[level]);

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_reschedule(&save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
#endif

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
