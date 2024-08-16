// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include "driver/gpio.h"

#include <ESP32Servo.h>

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using, "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

Servo myServo;
int pos = 0;

int throttlePin = 21;
int brakePin = 22;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl)
{
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == nullptr)
        {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot)
    {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl)
{
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == ctl)
        {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController)
    {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl)
{
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),       // Controller Index
        ctl->dpad(),        // D-pad
        ctl->buttons(),     // bitmask of pressed buttons
        ctl->axisX(),       // (-511 - 512) left X Axis
        ctl->axisY(),       // (-511 - 512) left Y axis
        ctl->axisRX(),      // (-511 - 512) right X axis
        ctl->axisRY(),      // (-511 - 512) right Y axis
        ctl->brake(),       // (0 - 1023): brake button
        ctl->throttle(),    // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(), // bitmask of pressed "misc" buttons
        ctl->gyroX(),       // Gyro X
        ctl->gyroY(),       // Gyro Y
        ctl->gyroZ(),       // Gyro Z
        ctl->accelX(),      // Accelerometer X
        ctl->accelY(),      // Accelerometer Y
        ctl->accelZ()       // Accelerometer Z
    );
}

int rangeConvert(int number)
{
    return 180 - (45 * (number + 511) / 256);
}

void processGamepad(ControllerPtr ctl)
{
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    digitalWrite(throttlePin, LOW);
    digitalWrite(22, LOW);
    myServo.write(90);
    if (ctl->a())
    {
        Console.println("A");
    }

    if (ctl->b())
    {
        Console.println("B");
    }

    if (ctl->x())
    {
        Console.println("X");

        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        // ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
        //                    0x40 /* strongMagnitude */);
    }

    if (ctl->y())
    {
        Console.println("Y");
    }

    if (ctl->throttle() > 100)
    {
        Console.printf("Throttle %5d \n", ctl->throttle());
        analogWrite(throttlePin, ctl->throttle() / 4);
    }

    if (ctl->brake() > 100)
    {
        Console.printf("Brake %5d \n", ctl->brake());
        analogWrite(brakePin, ctl->brake() / 4);
    }

    if (ctl->axisX() > 50)
    {
        Console.printf("Rigth %5d \n", ctl->axisX());
        myServo.write(rangeConvert(ctl->axisX()));
    }

    if (ctl->axisX() < -50)
    {
        Console.printf("Left %5d \n", ctl->axisX());
        myServo.write(rangeConvert(ctl->axisX()));
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    // dumpGamepad(ctl);

    // See ArduinoController.h for all the available functions.
}

void processControllers()
{
    for (auto myController : myControllers)
    {
        if (myController && myController->isConnected() && myController->hasData())
        {
            if (myController->isGamepad())
            {
                processGamepad(myController);
            }
            else
            {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}
// Arduino setup function. Runs in CPU 1

void setup()
{

    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);

    pinMode(throttlePin, OUTPUT);
    pinMode(brakePin, OUTPUT);

    myServo.attach(GPIO_NUM_13);
    myServo.write(0);

    digitalWrite(throttlePin, HIGH);
    digitalWrite(brakePin, HIGH);

    delay(1000);

    digitalWrite(throttlePin, LOW);
    digitalWrite(brakePin, LOW);

    for (pos = 0; pos <= 180; pos += 1)
    {                       // goes from 0 degrees to 180 degrees
                            // in steps of 1 degree
        myServo.write(pos); // tell servo to go to position in variable 'pos'
        delay(15);          // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1)
    {                       // goes from 180 degrees to 0 degrees
        myServo.write(pos); // tell servo to go to position in variable 'pos'
        delay(15);          // waits 15ms for the servo to reach the position
    }

    myServo.write(90);
}

// Arduino loop function. Runs in CPU 1.
void loop()
{
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
