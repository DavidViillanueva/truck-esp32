// Copyright 2021 - 2023, Ricardo Quesada, http://retro.moe
// SPDX-License-Identifier: Apache-2.0 or LGPL-2.1-or-later

#include "ArduinoController.h"

#include "sdkconfig.h"

#include "arduino_platform.h"

#include <uni_common.h>
#include <uni_log.h>

const struct Controller::controllerNames Controller::_controllerNames[] = {
    {Controller::CONTROLLER_TYPE_UnknownSteamController, "Unknown Steam"},
    {Controller::CONTROLLER_TYPE_SteamController, "Steam"},
    {Controller::CONTROLLER_TYPE_SteamControllerV2, "Steam V2"},

    {Controller::CONTROLLER_TYPE_XBox360Controller, "XBox 360"},
    {Controller::CONTROLLER_TYPE_XBoxOneController, "XBox One"},
    {Controller::CONTROLLER_TYPE_PS3Controller, "DualShock 3"},
    {Controller::CONTROLLER_TYPE_PS4Controller, "DualShock 4"},
    {Controller::CONTROLLER_TYPE_WiiController, "Wii"},
    {Controller::CONTROLLER_TYPE_AppleController, "Apple"},
    {Controller::CONTROLLER_TYPE_AndroidController, "Android"},
    {Controller::CONTROLLER_TYPE_SwitchProController, "Switch Pro"},
    {Controller::CONTROLLER_TYPE_SwitchJoyConLeft, "Switch JoyCon Left"},
    {Controller::CONTROLLER_TYPE_SwitchJoyConRight, "Switch JoyCon Right"},
    {Controller::CONTROLLER_TYPE_SwitchJoyConPair, "Switch JoyCon Pair"},
    {Controller::CONTROLLER_TYPE_SwitchInputOnlyController, "Switch Input Only"},
    {Controller::CONTROLLER_TYPE_MobileTouch, "Mobile Touch"},
    {Controller::CONTROLLER_TYPE_XInputSwitchController, "XInput Switch"},
    {Controller::CONTROLLER_TYPE_PS5Controller, "DualSense"},

    // Bluepad32 additions
    {Controller::CONTROLLER_TYPE_iCadeController, "iCade"},
    {Controller::CONTROLLER_TYPE_SmartTVRemoteController, "Smart TV Remote"},
    {Controller::CONTROLLER_TYPE_EightBitdoController, "8BitDo"},
    {Controller::CONTROLLER_TYPE_GenericController, "Generic"},
    {Controller::CONTROLLER_TYPE_NimbusController, "Nimbus"},
    {Controller::CONTROLLER_TYPE_OUYAController, "OUYA"},
    {Controller::CONTROLLER_TYPE_PSMoveController, "PSMove"},
    {Controller::CONTROLLER_TYPE_AtariJoystick, "Atari Joystick"},

    {Controller::CONTROLLER_TYPE_GenericKeyboard, "Keyboard"},
    {Controller::CONTROLLER_TYPE_GenericMouse, "Mouse"},
};

Controller::Controller() : _connected(false), _idx(-1), _data(), _properties() {}

bool Controller::isConnected() const {
    return _connected;
}

void Controller::disconnect() {
    if (!isConnected()) {
        loge("controller not connected");
        return;
    }

    arduino_disconnect_controller(_idx);
}

void Controller::setPlayerLEDs(uint8_t led) const {
    if (!isConnected()) {
        loge("controller not connected");
        return;
    }

    if (arduino_set_player_leds(_idx, led) == -1)
        loge("error setting player LEDs");
}

void Controller::setColorLED(uint8_t red, uint8_t green, uint8_t blue) const {
    if (!isConnected()) {
        loge("controller not connected");
        return;
    }

    if (arduino_set_lightbar_color(_idx, red, green, blue) == -1)
        loge("error setting lightbar color");
}

void Controller::playDualRumble(uint16_t delayedStartMs,
                                uint16_t durationMs,
                                uint8_t weakMagnitude,
                                uint8_t strongMagnitude) const {
    if (!isConnected()) {
        loge("controller not connected");
        return;
    }

    if (arduino_play_dual_rumble(_idx, delayedStartMs, durationMs, weakMagnitude, strongMagnitude) == -1)
        loge("error playing dual rumble");
}

String Controller::getModelName() const {
    for (int i = 0; i < ARRAY_SIZE(_controllerNames); i++) {
        if (_properties.type == _controllerNames[i].type)
            return _controllerNames[i].name;
    }
    return "Unknown";
}

//
// Keyboard functions
//
bool Controller::isKeyPressed(KeyboardKey key) const {
    // When querying for Modifiers, delegate to modifier function
    if (key >= Keyboard_LeftControl && key <= Keyboard_RightMeta) {
        return isModifierPressed(key);
    }

    for (int i = 0; i < UNI_KEYBOARD_PRESSED_KEYS_MAX; i++) {
        // Return early on error
        if (_data.keyboard.pressed_keys[i] <= HID_USAGE_KB_ERROR_UNDEFINED)
            return false;
        if (_data.keyboard.pressed_keys[i] == key)
            return true;
    }
    return false;
}

bool Controller::isAnyKeyPressed() const {
    for (unsigned char pressed_key : _data.keyboard.pressed_keys) {
        // Reserved for >= 0xe8
        if (pressed_key >= Keyboard_A && pressed_key < 0xe8)
            return true;
    }
    if (_data.keyboard.modifiers)
        return true;
    return false;
}

bool Controller::isModifierPressed(KeyboardKey key) const {
    static uint8_t convertion[] = {
        UNI_KEYBOARD_MODIFIER_LEFT_CONTROL,   //
        UNI_KEYBOARD_MODIFIER_LEFT_SHIFT,     //
        UNI_KEYBOARD_MODIFIER_LEFT_ALT,       //
        UNI_KEYBOARD_MODIFIER_LEFT_GUI,       //
        UNI_KEYBOARD_MODIFIER_RIGHT_CONTROL,  //
        UNI_KEYBOARD_MODIFIER_RIGHT_SHIFT,    //
        UNI_KEYBOARD_MODIFIER_RIGHT_ALT,      //
        UNI_KEYBOARD_MODIFIER_RIGHT_GUI,      //
    };

    // Safety check, out of range ?
    if (key < Keyboard_LeftControl || key > Keyboard_RightMeta)
        return false;

    int idx = key - Keyboard_LeftControl;
    uint8_t modifier = convertion[idx];

    // Safe to test for non-zero since we know that only one-bit is on in "modifier".
    return (_data.keyboard.modifiers & modifier);
}

// Private functions
void Controller::onConnected() {
    _connected = true;
    // Fetch properties, and have them cached.
    if (arduino_get_controller_properties(_idx, &_properties) != UNI_ARDUINO_ERROR_SUCCESS) {
        loge("failed to get controller properties");
    }
}

void Controller::onDisconnected() {
    _connected = false;
}
