/*
 * Adjustable speed for keyboard-driven cursor movement.
 *
 * The input processor scales REL_X/REL_Y by the current speed level and by
 * its own mul/div params. The behavior steps the level up or down. Levels are
 * kept per BLE profile and persisted in settings.
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
}

static K_WORK_DELAYABLE_DEFINE(save_work, save_work_cb);

static int mouse_speed_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    if (!settings_name_steq(name, "levels", NULL)) {
        return -ENOENT;
    }

    // Ignore stored data saved with a different profile count
    if (len != sizeof(levels)) {
        return 0;
    }

    int err = read_cb(cb_arg, levels, sizeof(levels));
    if (err < 0) {
        LOG_ERR("Failed to read mouse speed levels (err %d)", err);
        return err;
    }

    for (int i = 0; i < PROFILE_COUNT; i++) {
        if (levels[i] >= ARRAY_SIZE(level_percents)) {
            levels[i] = DEFAULT_LEVEL;
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
    }
    return 0;
}

// Input processor

static int16_t remainders[2];

static int mouse_speed_handle_event(const struct device *dev, struct input_event *event,
                                    uint32_t param1, uint32_t param2,
                                    struct zmk_input_processor_state *state) {
    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int axis = event->code == INPUT_REL_X ? 0 : 1;
    int32_t mul = level_percents[levels[active_slot()]] * param1;
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
    int level = levels[slot];

    switch (binding->param1) {
    case MSPD_INC:
        level = MIN(level + 1, (int)ARRAY_SIZE(level_percents) - 1);
        break;
    case MSPD_DEC:
        level = MAX(level - 1, 0);
        break;
    default:
        LOG_ERR("Unknown mouse speed command: %d", binding->param1);
        return -ENOTSUP;
    }

    if (level == levels[slot]) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    levels[slot] = level;
    LOG_DBG("Profile %d mouse speed %d%%", slot, level_percents[level]);

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
