#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <utility>  // for std::pair
#include <tuple> 

// Motor pins and PWM channels
extern int pwm_channel_mr_a, pwm_channel_mr_b, pwm_channel_ml_a, pwm_channel_ml_b;
extern int motor_right_a, motor_right_b, motor_left_a, motor_left_b;

// Encoder variables
extern volatile int encoder_left_count;   // Khai báo encoder trái
extern volatile int encoder_right_count;  // Khai báo encoder phải

// Motor control setup function
void motor_setup();

// Motor control functions
void forward(int speed);
void reverse(int speed);
void stop();
void right(int speed);
void left(int speed);

// Encoder reading function
std::tuple<int, int, int, int > read_encoder();  // Trả về cặp giá trị encoder

#endif // MOTOR_H
