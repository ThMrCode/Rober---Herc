#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

namespace GamepadManager {

    // Definición de un solo controlador (Maximo 4)
    constexpr int MAX_CONTROLLERS = 1;
    extern ControllerPtr myControllers[MAX_CONTROLLERS];

    // Callbacks para conexión/desconexión de controladores
    void onConnectedController(ControllerPtr ctl);
    void onDisconnectedController(ControllerPtr ctl);

    // Procesamiento del gamepad
    void processFlechas(ControllerPtr ctl);
    void processJoystick_I(ControllerPtr ctl);
    void processJoystick_D(ControllerPtr ctl);
    void processBotones(ControllerPtr ctl);
    void processGatillos(ControllerPtr ctl);
    void processControllers();

    // Inicialización del sistema Bluepad32
    void start();

    // Función principal que debe llamarse en loop()
    void update();
}
