#include <GamepadManager.h>

void setup() {
    Serial.begin(115200);
    GamepadManager::start();
}

void loop() {
    GamepadManager::update();
}
