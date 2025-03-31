#include <DriverControl.h>

namespace DriverControl {
    namespace CONTROLERS {
        CONTROL FORWARD = CONTROL(&DriverEncoder::ENCODERS::FORWARD, &DriverMotor::MOTORS::FORWARD, 15.0, -10.0);
        CONTROL MIDDLE = CONTROL(&DriverEncoder::ENCODERS::MIDDLE, &DriverMotor::MOTORS::MIDDLE, 15.0, -10.0);
        CONTROL BACKWARD = CONTROL(&DriverEncoder::ENCODERS::BACKWARD, &DriverMotor::MOTORS::BACKWARD, 15.0, -10.0);
    }
}

void DriverControl::start()
{
    // De momento no es necesaria
}

void DriverControl::update(CONTROLERS::CONTROL *control)
{
    // Funcion de Loop que se llama con setpoint previamente definido en control

    // Proteger de set points muy bajos
    if(abs(control->set_point) < 0.5f) {
        DriverMotor::setDuty(control->motor, 0);
    }
    else {
        // Aniadir filtros de kalman
        float speed_Rad = DriverEncoder::getVelocity(control->encoder) * PI * 2.0f;

        float error = control->set_point - speed_Rad;
        float output = control->KP * error + control->KI * control->previous_error + control->previous_output;
        
        if (output > DriverMotor::CONFIG::RESOLUTION_MAX) output = DriverMotor::CONFIG::RESOLUTION_MAX;
        if (output < -DriverMotor::CONFIG::RESOLUTION_MAX) output = -DriverMotor::CONFIG::RESOLUTION_MAX;
    
        // Actualiza valores previous
        control->previous_output = output;
        control->previous_error = error;
       
        // Escribir la salida en el motor
        DriverMotor::setDuty(control->motor, output);

        //Codigo de Depuracion
        //Serial.println(error);
    }
}

float DriverControl::filter_kalman(float speed)
{
    float x_est = 0.0;      // Estimación inicial de velocidad
    float P_est = 1.0;      // Covarianza inicial
    const float Q = 0.02;   // Ruido del proceso
    const float R = 0.05;   // Ruido de medición
    
    float x_pred = x_est;  // Predicción de la velocidad
    float P_pred = P_est + Q;  // Predicción de la covarianza
    float K = P_pred / (P_pred + R);  // Cálculo de la ganancia de Kalman
    x_est = x_pred + K * (speed - x_pred);  // Actualización de la estimación
    P_est = (1 - K) * P_pred;  // Actualización de la covarianza
    speed = x_est;  // Guardar la velocidad filtrada
    return speed;
}

