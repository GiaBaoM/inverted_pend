#include "inverted_pendulum.h"


void setup() {
    Serial.begin(115200);
    //DC motor setup 
    pinMode(pin_motor_right,OUTPUT);
    pinMode(pin_motor_left, OUTPUT);

    // PID setup for balance control
    PID_pend_balance.SetOutputLimits(-255,255);
    PID_pend_balance.SetSampleTime(SAMPLE_TIME);
    PID_pend_balance.SetMode(AUTOMATIC);
  
    PID_cart_balance.SetOutputLimits(-30,30);
    PID_cart_balance.SetSampleTime(SAMPLE_TIME);
    PID_cart_balance.SetMode(AUTOMATIC);

    //PID setup for swingup control






    //Encoder setup
    enc_Setup();
    // set PWM frequency for DC motor approximately 980Hz
    setPwmFrequencyMEGA2560(6,2);
    setPwmFrequencyMEGA2560(8,2);
    // set PWM frequency for DC motor approximately 980Hz
    // InitTimersSafe();
    // SetPinFrequencySafe(pin_motor_right, 980);
    // SetPinFrequencySafe(pin_motor_left, 980);
    control_mode = balance_mode;
}

  

void loop() {
    //reading values form timer and encoders
    encoder_update();
    //mode selection
    mode_selection();
     //control mode selection
    if(control_mode == balance_mode) {
      balance_control();
    }
    else if(control_mode == swingup_mode) {
      //swingup_control_PD();
      pend_output = 0;
      //swingup_control_basic();
    }
    else if(control_mode == stop_mode) {
      //go_stop();
      pend_output = 0;
    }
    else {
      pend_output = 0;
    }
    // motor control
    motor_control(pend_output);

    //motor_control(60);
     // print values for debugging
    // Serial.print("pend_angle: ");
    // Serial.print(pend_angle);
    // Serial.print(",");
    // Serial.print("cart_position: ");
    // Serial.print(cart_position);
    // Serial.print(",");
    // Serial.print("pend_output: ");
    // Serial.println(pend_output);
    // Serial.print(",");
    
    // Serial.print("\n");
    
    enc_read();
    calculate_cart_velocity();
    calculate_pend_velocity();
    Serial.print(pend_angle);
    Serial.print(",");
    Serial.print(pend_setpoint);
    Serial.print(",");
    //Serial.print(pend_angle_in_degree);
    //Serial.print(",");

    //Serial.print(pend_setpoint);
    //Serial.print(",");
    Serial.print(control_mode);
    Serial.print(",");
    //Serial.print(pend_vel_in_degree_per_second);
    //Serial.print(",");

    //Serial.print(cart_position_in_centimeter);
    //Serial.print(",");
    //Serial.print(cart_vel_in_centimeter_per_second);
    //Serial.print(",");

    Serial.println(pend_output);
    //delay(100);


}

