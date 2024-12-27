#include <Arduino.h>

// Cấu hình các chân động cơ và các kênh PWM
int pwm_channel_mr_a = 0; // Kênh PWM cho động cơ phải A
int pwm_channel_mr_b = 1; // Kênh PWM cho động cơ phải B
int pwm_channel_ml_a = 2; // Kênh PWM cho động cơ trái A
int pwm_channel_ml_b = 3; // Kênh PWM cho động cơ trái B

int motor_right_a = 18;  // Chân cho động cơ phải A
int motor_right_b = 19;  // Chân cho động cơ phải B
int motor_left_a = 22;   // Chân cho động cơ trái A
int motor_left_b = 23;   // Chân cho động cơ trái B

// Biến đếm encoder và hướng cho động cơ trái và phải
volatile int encoder_left_count = 0;
volatile int encoder_right_count = 0;

const int encoder_resolution = 530; // Độ phân giải của encoder

// Chân encoder
int encoder_left_c1 = 26;
int encoder_left_c2 = 25;
int encoder_right_c1 = 34;
int encoder_right_c2 = 35;

// ISR cho encoder trái C1
void IRAM_ATTR encoder_left_c1_isr() {
    if (digitalRead(encoder_left_c2) == LOW) {
        encoder_left_count--;
    } else {
        encoder_left_count++;
    }
}

// ISR cho encoder phải C1
void IRAM_ATTR encoder_right_c1_isr() {
    if (digitalRead(encoder_right_c2) == LOW) {
        encoder_right_count++;
    } else {
        encoder_right_count--;
    }
}

// Hàm thiết lập động cơ
void motor_setup() {
    pinMode(motor_right_a, OUTPUT);
    pinMode(motor_right_b, OUTPUT);
    pinMode(motor_left_a, OUTPUT);
    pinMode(motor_left_b, OUTPUT);

    pinMode(encoder_left_c1, INPUT);
    pinMode(encoder_left_c2, INPUT);
    pinMode(encoder_right_c1, INPUT);
    pinMode(encoder_right_c2, INPUT);

    ledcSetup(pwm_channel_mr_a, 5000, 8);
    ledcSetup(pwm_channel_mr_b, 5000, 8);
    ledcSetup(pwm_channel_ml_a, 5000, 8);
    ledcSetup(pwm_channel_ml_b, 5000, 8);

    ledcAttachPin(motor_right_a, pwm_channel_mr_a);
    ledcAttachPin(motor_right_b, pwm_channel_mr_b);
    ledcAttachPin(motor_left_a, pwm_channel_ml_a);
    ledcAttachPin(motor_left_b, pwm_channel_ml_b);

    attachInterrupt(digitalPinToInterrupt(encoder_left_c1), encoder_left_c1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_right_c1), encoder_right_c1_isr, RISING);
}

// Hàm điều khiển động cơ
void reverse(int speed) {
    ledcWrite(pwm_channel_mr_a, speed*0.7);
    ledcWrite(pwm_channel_mr_b, 0);
    ledcWrite(pwm_channel_ml_a, speed*0.7);
    ledcWrite(pwm_channel_ml_b, 0);
}

void forward(int speed) {
    ledcWrite(pwm_channel_mr_a, 0);
    ledcWrite(pwm_channel_mr_b, speed*0.7);
    ledcWrite(pwm_channel_ml_a, 0);
    ledcWrite(pwm_channel_ml_b, speed*0.7);
}

void stop() {
    ledcWrite(pwm_channel_mr_a, 0);
    ledcWrite(pwm_channel_mr_b, 0);
    ledcWrite(pwm_channel_ml_a, 0);
    ledcWrite(pwm_channel_ml_b, 0);
}

void left(int speed) {
    ledcWrite(pwm_channel_mr_b, speed * 0.3);
    ledcWrite(pwm_channel_ml_b, speed * 0.48);
    ledcWrite(pwm_channel_mr_a, 0);
    ledcWrite(pwm_channel_ml_a, 0);
}

void right(int speed) {
    ledcWrite(pwm_channel_mr_b, speed * 0.48);
    ledcWrite(pwm_channel_ml_b, speed * 0.3);
    ledcWrite(pwm_channel_mr_a, 0);
    ledcWrite(pwm_channel_ml_a, 0);
}

std::tuple<int, int> read_encoder() {
    return std::make_tuple(encoder_left_count, encoder_right_count);
}

std::pair<int, int> error_motor_drive(int error) {
    return std::make_pair(0, 0);  // Placeholder
}
