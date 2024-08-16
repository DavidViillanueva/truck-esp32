// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Ricardo Quesada
// http://retro.moe/unijoysticle2

// Unijoysticle platform

#include "platform/uni_platform_unijoysticle_800xl.h"

#include "sdkconfig.h"

#include "uni_common.h"
#include "uni_config.h"
#include "uni_log.h"

// Unijoysticle v2 800XL

static const struct uni_platform_unijoysticle_gpio_config gpio_config_800xl = {
    .port_a = {GPIO_NUM_26, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_14, GPIO_NUM_33, GPIO_NUM_16},
    .port_b = {GPIO_NUM_27, GPIO_NUM_25, GPIO_NUM_32, GPIO_NUM_17, GPIO_NUM_13, GPIO_NUM_21, GPIO_NUM_22},
    .leds = {GPIO_NUM_5, GPIO_NUM_12, GPIO_NUM_15},
    .push_buttons = {{
                         .gpio = GPIO_NUM_34,
                         .callback = uni_platform_unijoysticle_on_push_button_mode_pressed,
                     },
                     {
                         .gpio = GPIO_NUM_35,
                         .callback = uni_platform_unijoysticle_on_push_button_swap_pressed,
                     }},
    .sync_irq = {-1, -1},
};

//
// Variant overrides
//

//
const struct uni_platform_unijoysticle_variant* uni_platform_unijoysticle_800xl_create_variant(void) {
    const static struct uni_platform_unijoysticle_variant variant = {
        .name = "2 800XL",
        .gpio_config = &gpio_config_800xl,
        .flags = UNI_PLATFORM_UNIJOYSTICLE_VARIANT_FLAG_QUADRATURE_MOUSE,
        .supported_modes = UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_NORMAL |
                           UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_TWINSTICK |
                           UNI_PLATFORM_UNIJOYSTICLE_GAMEPAD_MODE_MOUSE,
        .default_mouse_emulation = UNI_PLATFORM_UNIJOYSTICLE_MOUSE_EMULATION_ATARIST,
        .preferred_seat_for_mouse = GAMEPAD_SEAT_B,
        .preferred_seat_for_joystick = GAMEPAD_SEAT_A,
    };

    return &variant;
}