#include "GamepadManager.h"
#include <ESP32Servo.h>
#include <math.h> 

Servo servo1; 
Servo servo2;
Servo servo3;
Servo servo4; 

float ancho = 500;
float largo_1 = 200;
float largo_2 = 300;
float velocidad;

unsigned long previousMillis = 0;
const long interval = 150;

namespace GamepadManager {
    ControllerPtr myControllers[MAX_CONTROLLERS] = {nullptr};
}

void GamepadManager::start() {
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  servo1.attach(23); 
  servo2.attach(22); 
  servo3.attach(21); 
  servo4.attach(19); 
}

void GamepadManager::onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < MAX_CONTROLLERS; i++) {
    if (myControllers[i] == nullptr) {
      ControllerProperties properties = ctl->getProperties();
      myControllers[i] = ctl;
      return;
    }
  }
}

void GamepadManager::onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < MAX_CONTROLLERS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      return;
    }
  }
}

// Lectura joystick izquierdo del mando
void GamepadManager::processJoystick_I(ControllerPtr ctl){

  float Resultante_izquierda = sqrt(pow(ctl->axisX(),2) + pow(ctl->axisY(),2));
  //float alfa = atan((radio_de_giro - (ancho / 2)) / distancia_vertical) * 180.0 / PI;

  if (ctl->axisX() >= 100){
    float radio_de_giro = map(ctl->axisX(), 100, 512, 2000, 600);
    //float velocidad = map(ctl->brake(),0,1020,0,775);
    if (ctl->buttons() == 128){
      velocidad = ((ctl->throttle())*550)/1020;
    }
    else if (ctl->buttons() == 64) {
      velocidad = (-(ctl->brake())*550)/1020;
    }
    else {
      velocidad = 0;
    }

    float angular = velocidad/radio_de_giro;

    float alfa_1 = (atan(-largo_1 / (radio_de_giro - (ancho / 2))) * 180.0 / PI);
    float alfa_2 = (atan(largo_2 / (radio_de_giro - (ancho / 2))) * 180.0 / PI);
    float alfa_3 = (atan(largo_2 / (radio_de_giro + (ancho / 2))) * 180.0 / PI);
    float alfa_4 = (atan(-largo_1 / (radio_de_giro + (ancho / 2))) * 180.0 / PI);

    float motor_1 = (angular * sqrt(pow(largo_1,2) + pow(radio_de_giro + (ancho / 2),2)))/90;
    float motor_2 = (angular * (radio_de_giro + (ancho / 2)))/90; 
    float motor_3 = (angular * sqrt(pow(largo_2,2) + pow(radio_de_giro + (ancho / 2),2)))/90;
    float motor_4 = (angular * sqrt(pow(largo_2,2) + pow(radio_de_giro - (ancho / 2),2)))/90;
    float motor_5 = (angular * (radio_de_giro - (ancho / 2)))/90; 
    float motor_6 = (angular * sqrt(pow(largo_1,2) + pow(radio_de_giro - (ancho / 2),2)))/90;
    
    Serial.printf("%4f %4f %4f %4f %4f %4f ",motor_1,motor_2,motor_3,motor_4,motor_5,motor_6);
    Serial.printf("%4f \n",radio_de_giro);

    servo1.write(alfa_1);
    servo2.write(alfa_2);
    servo3.write(alfa_3);
    servo4.write(alfa_4);
  }
  else if (ctl->axisX() <= -100){
    float radio_de_giro = map(ctl->axisX(), -100, -512, 1600, 600);
    
    if (ctl->buttons() == 128){
      velocidad = ((ctl->throttle())*550)/1020;
    }
    else if (ctl->buttons() == 64) {
      velocidad = (-(ctl->brake())*550)/1020;
    }
    else {
      velocidad = 0;
    }

    float angular = velocidad/radio_de_giro;

    float alfa_1 = -(atan(largo_1 / (-radio_de_giro + (ancho / 2))) * 180.0 / PI);
    float alfa_2 = -(atan(largo_2 / (-radio_de_giro + (ancho / 2))) * 180.0 / PI);
    float alfa_3 = -(atan(largo_2 / (-radio_de_giro - (ancho / 2))) * 180.0 / PI);
    float alfa_4 = (atan(-largo_1 / (-radio_de_giro - (ancho / 2))) * 180.0 / PI);

    float motor_1 = (angular * sqrt(pow(largo_1,2) + pow(radio_de_giro - (ancho / 2),2)))/90;
    float motor_2 = (angular * (radio_de_giro - (ancho / 2)))/90;
    float motor_3 = (angular * sqrt(pow(largo_2,2) + pow(radio_de_giro - (ancho / 2),2)))/90;
    float motor_4 = (angular * sqrt(pow(largo_2,2) + pow(radio_de_giro + (ancho / 2),2)))/90;
    float motor_5 = (angular * (radio_de_giro + (ancho / 2)))/90;
    float motor_6 = (angular * sqrt(pow(largo_1,2) + pow(radio_de_giro + (ancho / 2),2)))/90;

    Serial.printf("%4f %4f %4f %4f %4f %4f ",motor_1,motor_2,motor_3,motor_4,motor_5,motor_6);
    Serial.printf("%4f \n",radio_de_giro);

    servo1.write(90 + alfa_1);
    servo2.write(alfa_2);
    servo3.write(90 - alfa_3);
    servo4.write(90 + alfa_4);
  }
}

// Lectura joystick derecho del mando
void GamepadManager::processJoystick_D(ControllerPtr ctl){

}

// Lectura de los botones en la parte derecha del mando
void GamepadManager::processBotones(ControllerPtr ctl){
  if (ctl->a()){
    //float alfa_1 = 90 + (atan(20 / (50 / 2)) * 180.0 / PI);
    // float alfa_2 = 90 - (atan(30 / (50 / 2)) * 180.0 / PI);
    // float alfa_3 = 90 + (atan(30 / (50 / 2)) * 180.0 / PI);
    // float alfa_4 = 90 - (atan(20 / (50 / 2)) * 180.0 / PI);
    float alfa_1 = 38.6598;
    float alfa_2 = 50.1944;
    float alfa_3 = 50.1944;
    float alfa_4 = 38.6598;
    
    Serial.printf("%d ",ctl->brake());
    Serial.printf("%d ",ctl->throttle());    
    Serial.printf("0 \n");

    servo1.write(90 + alfa_1);
    servo2.write(90 - alfa_2);
    servo3.write(90 + alfa_3);
    servo4.write(90 - alfa_4);
  }
  else if ((ctl->a() == false) && ((ctl->axisX() > -100) && (ctl->axisX() < 100))) {
    if (ctl->buttons() == 128){
      velocidad = ((ctl->throttle())*775)/1020;
    }
    else if (ctl->buttons() == 64) {
      velocidad = (-(ctl->brake())*775)/1020;
    }
    else {
      velocidad = 0;
    }

    float motor_1 = velocidad/90;
    float motor_2 = velocidad/90;
    float motor_3 = velocidad/90;
    float motor_4 = velocidad/90;
    float motor_5 = velocidad/90;
    float motor_6 = velocidad/90;

    Serial.printf("%4f %4f %4f %4f %4f %4f ",motor_1,motor_2,motor_3,motor_4,motor_5,motor_6);   
    Serial.printf("False \n");

    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);
  }
}

// Lectura de los gatillos del mando
void GamepadManager::processGatillos(ControllerPtr ctl) {
  // if (ctl->buttons() == 128) {
  //   Serial.printf("Gatillo derecho: %d \n",ctl->throttle());
  // }
  // else if (ctl->buttons() == 64) {
  //   Serial.printf("Gatillo izquierdo: %d \n",ctl->brake());
  // }
}

void GamepadManager::processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->isGamepad()) {
        processJoystick_I(myController);
        processJoystick_D(myController);
        processBotones(myController);
        processGatillos(myController);
    }
  }
}

void GamepadManager::update() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    BP32.update();
    processControllers();
  }
}