#pragma once
#include <Precompile.h>
#include <DriverMotor.h>
#include <DriverEncoder.h>

namespace DriverControl {

    // CONFIG
    namespace CONFIG {
        constexpr float Q = 0.02f;
        constexpr float R = 0.05f;
    }

    // PINOUT MOTORS
    namespace CONTROLERS {
        struct CONTROL {
            DriverEncoder::ENCODERS::ENCODER* encoder;
            DriverMotor::MOTORS::MOTOR* motor;
            float KP;
            float KI;
            float previous_error;
            float previous_output;
            float x_est;
            float p_est;
            float set_point;        // relacion output pwm, control rads/s 
            CONTROL(DriverEncoder::ENCODERS::ENCODER* encoder_, DriverMotor::MOTORS::MOTOR* motor_, float KP_, float KI_) {
                this->encoder = encoder_; this->motor = motor_; this-> KP = KP_; this->KI = KI_;
                this->previous_error = 0; this-> previous_output = 0; this->set_point = 0;
            };
        };
        //extern CONTROL FORWARD_LEFT;
        extern CONTROL FORWARD;
        extern CONTROL MIDDLE;
        extern CONTROL BACKWARD;
    }

    // Start Driver
    void start();

    void update(CONTROLERS::CONTROL* control);

    float filter_kalman(CONTROLERS::CONTROL *control);

}
