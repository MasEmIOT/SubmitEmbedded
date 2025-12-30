#include <Arduino.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// 1. THƯ VIỆN CHẠY AI (Cài EloquentTinyML trong Library Manager)
#include <EloquentTinyML.h>
// 2. FILE MODEL CỦA BẠN
#include "model_data.h"

#define SETPOINT 178.64
LMotorController motor(32, 25, 26, 33, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 30

MPU6050 mpu;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool dmpReady = false;

// CẤU HÌNH TENSORFLOW LITE
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 1
#define TENSOR_ARENA_SIZE 2 * 1024 // 2KB RAM cho AI

Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22, 400000);

    // KHỞI TẠO MPU
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXGyroOffset(-73); 
    mpu.setYGyroOffset(-95);
    mpu.setZGyroOffset(-20); 
    mpu.setZAccelOffset(1350);
    mpu.setDMPEnabled(true);
    dmpReady = true;

    // KHỞI TẠO AI MODEL
    ml.begin(robot_model);
    Serial.println("DEEP LEARNING MODEL LOADED!");
}

void loop() {
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // 1. LẤY DỮ LIỆU INPUT
        float current_angle = ypr[1] * 180 / M_PI + 180;
        float error = current_angle - SETPOINT;
        
        // Input cho mạng neron là mảng float
        float input_array[1] = { error };
        
        // 2. AI SUY LUẬN (PREDICT)
        // Mạng neron sẽ trả về con số từ -1.0 đến 1.0 (do lúc train mình đã chia 255)
        float predicted_val = ml.predict(input_array);
        
        // 3. GIẢI MÃ OUTPUT (Denormalize)
        int pwm_out = predicted_val * 255;

        // 4. ĐIỀU KHIỂN
        // Kẹp giá trị an toàn
        if (pwm_out > 255) pwm_out = 255;
        if (pwm_out < -255) pwm_out = -255;

        if (abs(error) > 30) {
            motor.move(0);
        } else {
            // Chạy cực êm vì pwm_out là số liên tục, không bị nhảy cóc
            motor.move(pwm_out, MIN_ABS_SPEED);
        }

        // Debug xem AI thông minh cỡ nào
        // Serial.print("Err: "); Serial.print(error);
        // Serial.print(" -> AI PWM: "); Serial.println(pwm_out);
    }
}