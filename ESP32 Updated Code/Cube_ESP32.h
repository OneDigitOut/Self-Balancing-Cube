#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include <FastLED.h>

#pragma once

// Battery Definistions
#define VBAT        34
#define BATTERY_CALIBRATION 262 // Calculated by dividing battery voltage by (double)analogRead(VBAT) 

#define BRAKE       26

// Motor 1 definition
#define DIR1        5
#define ENC1_1      16
#define ENC1_2      17
#define PWM1        18
#define PWM1_CH     2

// Motor 2 definition
#define DIR2        4
#define ENC2_1      35
#define ENC2_2      33
#define PWM2        32
#define PWM2_CH     1

// Motor 3 definition
#define DIR3        15
#define ENC3_1      13
#define ENC3_2      14
#define PWM3        25
#define PWM3_CH     0

#define TIMER_BIT   8
#define BASE_FREQ   20000

// MPU6050 Definition
#define MPU6050       0x68        // Device address
#define ACCEL_CONFIG  0x1C        // Accelerometer configuration address
#define GYRO_CONFIG   0x1B        // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define accSens 0                 // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0                // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

// EEPROM Definition
#define EEPROM_SIZE   64

// LED definition
#define INT_LED       2
#define LED_PIN       19          // Pin that connects to WS2812B
#define NUM_PIXELS    3           // The number of LEDs (pixels) on WS2812B

// BUZZER definition
#define INSTALLED_BUZZER_TYPE 0   // 0=Passive, 1=Active 
#define BUZZER      27

// BUZZER note definition
#define NOTE_C        524
#define NOTE_DURATION 250

enum led_order {
  LED0 = 0,
  LED1 = 1,
  LED2 = 2
};

extern float Gyro_amount;

extern bool vertical_vertex;
extern bool vertical_edge;
extern bool calibrating;
extern bool vertex_calibrated;
extern bool calibrated;
extern bool calibrated_leds;

extern float K1;
extern float K2; 
extern float K3;
extern float K4;
extern float zK2;
extern float zK3;

extern float eK1;
extern float eK2; 
extern float eK3;
extern float eK4;

extern int loop_time;

struct OffsetsObj {
  int ID;
  float acXv;
  float acYv;
  float acZv;
  float acXe;
  float acYe;
  float acZe;
};
extern OffsetsObj offsets;

extern float alpha;

extern int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
extern float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
extern float speed_X, speed_Y;

extern int16_t  GyZ_offset;
extern int16_t  GyY_offset;
extern int16_t  GyX_offset;
extern int32_t  GyZ_offset_sum;
extern int32_t  GyY_offset_sum;
extern int32_t  GyX_offset_sum;

extern float robot_angleX, robot_angleY;
extern float Acc_angleX, Acc_angleY;      
extern int32_t motors_speed_X; 
extern int32_t motors_speed_Y;
extern int32_t motors_speed_Z;   

extern long currentT, previousT_1, previousT_2;

extern volatile int  enc_count1, enc_count2, enc_count3;
extern int16_t motor1_speed;         
extern int16_t motor2_speed;         
extern int16_t motor3_speed;     

extern BluetoothSerial SerialBT;
extern CRGB leds[NUM_PIXELS];

// function prototypes
void Motor1_control(int);
void Motor2_control(int);
void Motor3_control(int);
void angle_setup(void);
int Tuning(void);
void angle_calc(void);
void threeWay_to_XY(int, int, int); 
void XYZ_to_threeWay(float, float, float);
void battVoltage(double); 
void ENC1_READ(void);
void ENC2_READ(void);
void ENC3_READ(void);