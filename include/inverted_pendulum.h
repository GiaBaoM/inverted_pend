#include "PID_v1.h"
#include <Arduino.h>
#include "pwm_control.h"
#include "PWM.h"

//#define USE_ENCODER_h_LIB
#define USE_MY_ENCODER_h_LIB

#ifdef USE_MY_ENCODER_h_LIB
  #include "my_encoder.h"
#else
  #include "Encoder.h"
#endif

#define ENCODER_OPTIMIZE_INTERRUPTS
#define SAMPLE_TIME 3

//UART pins
  //UART0 0(RX), 1(TX) 
  //UART1 19(RX1), 18(TX1)
  //UART2 17(RX2), 16(TX2)
  //UART3 15(RX3), 14(TX3)

//Encdoer pins (interrupt pins)
#define pin_cart_enc1             3
#define pin_cart_enc2             2
#define pin_pend_enc1            18
#define pin_pend_enc2            19

#define pend_enc_slots                1000
#define cart_enc_slots                600
#define travel_slot                   13000
#define cart_middle_pos               0
#define cart_limit_max                6000
#define cart_limit_min                -6000
#define pend_max_positive_angle       550
#define pend_min_positive_angle       450
#define pend_max_negative_angle       -550
#define pend_min_negative_angle       -450

// DC motor PIN
#define pin_motor_right           6
#define pin_motor_left            8

//button pin
#define pin_button                7

//MODE
#define balance_mode              1
#define swingup_mode              2
#define stop_mode                3

void enc_Setup();
void enc_read();
bool is_controlable();
void go_right(double pwm_value);
void go_left(double pwm_value);
void go_stop();
void motor_control(int16_t pwm_value);
void swingup_control_basic();
void swingup_control_PD();
void balance_control();
void mode_selection();
void encoder_update();
void calculate_cart_velocity();
void calculate_pend_velocity();
void calculate_setpoint_for_pend_balance();
void print_values();

// parameters for PID balancing controllers
double cart_kp = 0.004;
double cart_ki = 0.0005;
double cart_kd = 0.0025;
double cart_setpoint = 0;
double cart_output = 0;  //output of cart is an offset for the setpoint of pendulum angle

double pend_kp = 4.32;
double pend_ki = 43.2;
double pend_kd = 0.108;
double pend_setpoint = 0;
double pend_output = 0;
//pwm signal for the motor
double pwm_output = 0;
// parameters for PD swing-up controllers
double pend_kp_swingup = 8;
double pend_kd_swingup = 1;
double cart_kp_swingup = 0.05;
double cart_kd_swingup = 0.03;
double cart_setpoint_swingup = 0;
double pend_setpoint_swingup = 0;
double pend_output_swingup = 0;

// velocity and position of the pendulum
double pend_angle = 0;
double opposite_pend_angle = 0;
double pend_angle_in_degree = 0;
double pend_vel = 0;
double pend_vel_in_degree_per_second = 0;
//velocity and position of the cart
double cart_position = 0;
double cart_vel = 0;
double cart_position_in_centimeter = 0;
double cart_vel_in_centimeter_per_second = 0;

//control mode:balance, swingup, stop
uint8_t control_mode = 0;
//variables for the calculation of the velocity
double timer;
const int array_length = 50;
double cart_position_array[array_length];
double pend_angle_array[array_length];
double timer_array[array_length];

// Encoder intialize
#ifdef USE_ENCODER_h_LIB
Encoder Enc_cart (pin_cart_enc1,pin_cart_enc2);
Encoder Enc_pend (pin_pend_enc1,pin_pend_enc2);
#endif

#ifdef USE_MY_ENCODER_h_LIB
Encoder * Encoder::instances[3] = {NULL, NULL, NULL};
Encoder Enc_cart(0, pin_cart_enc1,pin_cart_enc2, INTERRUPT_MODE);
Encoder Enc_pend(1, pin_pend_enc1,pin_pend_enc2, INTERRUPT_MODE);
#endif

//PID initialize for balancing control
PID PID_cart_balance(&cart_position, &cart_output, &cart_setpoint, cart_kp, cart_ki, cart_kd, P_ON_E, DIRECT);
PID PID_pend_balance(&pend_angle, &pend_output, &pend_setpoint, pend_kp, pend_ki, pend_kd, P_ON_E, DIRECT); 

//PID initialize for PD swing-up control
PID PID_pend_swingup(&opposite_pend_angle, &cart_setpoint_swingup, &pend_setpoint_swingup, pend_kp_swingup, 0, pend_kd_swingup, P_ON_E, DIRECT);
PID PID_cart_swingup(&cart_position, &pend_output_swingup, &cart_setpoint_swingup, cart_kp_swingup, 0, cart_kd_swingup, P_ON_E, REVERSE);





void enc_Setup() {
  //Initialize begin position for cart and pendulum
  Enc_cart.write(0);
  Enc_pend.write(0);
}

void enc_read() {
  pend_angle = Enc_pend.read();
  cart_position  = Enc_cart.read();
}
bool is_controlable() {
  if(((pend_angle < pend_max_positive_angle && pend_angle > pend_min_positive_angle)
    || (pend_angle > pend_max_negative_angle && pend_angle < pend_min_negative_angle)) 
    &&
    (cart_position < cart_limit_max && cart_position > cart_limit_min))
    return true;
   else return false;
}
void go_right(double pwm_value ) {
  analogWrite(pin_motor_left,0);
  analogWrite(pin_motor_right,pwm_value);
}

void go_left(double pwm_value ) {
  analogWrite(pin_motor_left,pwm_value);
  analogWrite(pin_motor_right,0);
}

void go_stop() {
  analogWrite(pin_motor_left,0);
  analogWrite(pin_motor_right,0);
}
void swingup_control_PD() {
  PID_pend_swingup.Compute();
  PID_cart_swingup.Compute();
}
void swingup_control_basic(){
  calculate_pend_velocity();
  pend_output_swingup = (pend_vel_in_degree_per_second >0)? 63:-63;
  for (int i = 0; i < 300; i++){
  if (is_controlable()) break;
  delay(1);
  }
};

void balance_control(){
    //PID_cart_balance.SetTunings(cart_kp,cart_ki,cart_kd);
    PID_cart_balance.Compute();
    calculate_setpoint_for_pend_balance();
    //PID_cart_balance.SetTunings(pend_kp,pend_ki,pend_kd);
    PID_pend_balance.Compute();
}
void mode_selection(){
  if (is_controlable()) {
    control_mode = balance_mode;
  }
  else if (digitalRead(pin_button) == LOW) {
    control_mode = swingup_mode;
  }
  else {
    control_mode = stop_mode;
  }
}
void motor_control(int16_t pwm_value) {
  if(pwm_value > 0) {go_left(pwm_value);}
  else if (pwm_value < 0) {go_right(abs(pwm_value));}
  else {go_stop();}
}

void enc_test() {
  enc_read();
  Serial.print("Cart Position: ");
  Serial.println(cart_position);
  // Serial.print(",");
   Serial.print(" Pendulum Angle: ");
   Serial.println(pend_angle);
}
void encoder_update() {
  // Calculate the velocity of the pendulum
  //Pushing the linear postion values one step down in the array to make place at element [0] for the present linear position  
    for (int i=array_length; i>1; i--){
      cart_position_array[i-1]=cart_position_array[i-2];
    }
  //Pushing the angle values one step down in the array to make place at element [0] for the present angle    
    for (int i=array_length; i>1; i--){
      pend_angle_array[i-1]=pend_angle_array[i-2];
    }
 
  //Pushing the timer values one step down in the array to make place at element [0] for the present time in milliseconds    
    for (int i=array_length; i>1; i--){
      timer_array[i-1]=timer_array[i-2];
    }
    
  //reading the present values for timer, angle and linear postion and save them to element 0 in their arrays  
    timer=millis();
    timer_array[0]=timer;
    pend_angle=Enc_pend.read();
    opposite_pend_angle = - pend_angle;
    pend_angle_array[0]=pend_angle;
    pend_angle_in_degree = pend_angle_array[0]/pend_enc_slots*360;
    cart_position=Enc_cart.read();
    cart_position_array[0]=cart_position;
    cart_position_in_centimeter = cart_position_array[0]/travel_slot*100;
}

void calculate_setpoint_for_pend_balance() {
  if (pend_angle >= 0) pend_setpoint = cart_output + pend_enc_slots/2;
  else pend_setpoint = cart_output - pend_enc_slots/2;
}
void calculate_cart_velocity() {
  // Calculate the velocity of the cart
  cart_vel = (cart_position_array[0]-cart_position_array[array_length-1])/(timer_array[0]-timer_array[array_length-1])*1000;
  cart_vel_in_centimeter_per_second = cart_vel/travel_slot*100;
}

void calculate_pend_velocity() {
  // Calculate the velocity of the pendulum
  pend_vel = (pend_angle_array[0]-pend_angle_array[array_length-1])/(timer_array[0]-timer_array[array_length-1])*1000;
  pend_vel_in_degree_per_second = pend_vel/pend_enc_slots*360;
}
void print_values(){
    // Serial.print(pend_angle);
    // Serial.print(",");
    // Serial.print(cart_position);
    // Serial.print(",");
    //Serial.print(opposite_pend_angle);
    //Serial.print(",");
    //Serial.print(pend_setpoint/pend_enc_slots*360); // pendsetpoint in degree
    //Serial.print(",");
    Serial.print(pend_angle_in_degree);
    Serial.print(",");
    //Serial.print(control_mode);
    //Serial.print(",");
    //Serial.print(pend_vel_in_degree_per_second);
    //Serial.print(",");
    Serial.println(cart_position_in_centimeter);
    //Serial.println(",");
    //Serial.print(cart_vel_in_centimeter_per_second);
    //Serial.print(",");
    //Serial.println(pwm_output);
}
void print_values_labview(){
    Serial.print("a");
    Serial.print(pend_angle_in_degree);
    Serial.print("b");
    Serial.print(pend_vel_in_degree_per_second);
    Serial.print("c");
    Serial.print(cart_position_in_centimeter);
    Serial.print("d");
    Serial.print(cart_vel_in_centimeter_per_second);
    Serial.print("e");
}
