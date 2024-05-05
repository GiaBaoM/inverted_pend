#include "inverted_pendulum.h"
#include "TimerOne.h"

void setup() {
    Serial.begin(115200);
    //Timer1.initialize (10000); //100Hz
    //Timer1.attachInterrupt(print_values);

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
    PID_pend_swingup.SetOutputLimits(-3000,3000);
    PID_pend_swingup.SetSampleTime(10);
    PID_pend_swingup.SetMode(AUTOMATIC);

    PID_cart_swingup.SetOutputLimits(-100,100);
    PID_cart_swingup.SetSampleTime(10);
    PID_cart_swingup.SetMode(AUTOMATIC);

    //Encoder setup
    enc_Setup();
    // set PWM frequency for DC motor approximately 4000Hz by using divisor 
    setPwmFrequencyMEGA2560(6,2);
    setPwmFrequencyMEGA2560(8,2);
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
      pwm_output = pend_output;
    }
    else if(control_mode == swingup_mode) {
      swingup_control_PD();
      //swingup_control_basic();
      pwm_output = pend_output_swingup;
      
    }
    else if(control_mode == stop_mode) {
      //go_stop();
      pwm_output = 0;
    }
    else {
      pwm_output = 0;
    }
    //motor control
    motor_control(pwm_output);
    enc_read();
    calculate_cart_velocity();
    calculate_pend_velocity();
    //print_values();
    // Serial.print(pend_angle);
    // Serial.print(',');
    // Serial.println(cart_position);
    print_values_labview();
    //print_values();

}

