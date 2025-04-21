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
        // Filtro de kalman
        DriverEncoder::getVelocity(control->encoder);
        float speed_rps = DriverControl::filter_kalman(control);
        float speed_Rad = speed_rps * PI * 2.0f;
        
        // Control
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

float DriverControl::filter_kalman(CONTROLERS::CONTROL *control)
{
    float x_pred = control->x_est;  // Predicción de la velocidad
    float P_pred = control->p_est + CONFIG::Q;  // Predicción de la covarianza
    float K = P_pred / (P_pred + CONFIG::R);  // Cálculo de la ganancia de Kalman
    control->x_est = x_pred + K * (control->encoder->speed - x_pred);  // Actualización de la estimación
    control->p_est = (1 - K) * P_pred;  // Actualización de la covarianza
    return control->x_est;
}

