#include "Cube_ESP32.h"

float Gyro_amount = 0.996;

bool vertical_vertex = false;
bool vertical_edge = false;
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;

float K1 = 180;
float K2 = 30.00; 
float K3 = 1.6;
float K4 = 0.008;
float zK2 = 8.00;
float zK3 = 0.30;

float eK1 = 190;
float eK2 = 31.00; 
float eK3 = 2.5;
float eK4 = 0.014;

int loop_time = 15;

OffsetsObj offsets;

float alpha = 0.7;

int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float speed_X, speed_Y;

int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;
int32_t  GyX_offset_sum = 0;

float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;      
int32_t motors_speed_X; 
int32_t motors_speed_Y;
int32_t motors_speed_Z;   

long currentT, previousT_1, previousT_2;

volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motor1_speed;         
int16_t motor2_speed;         
int16_t motor3_speed;     

BluetoothSerial SerialBT;
CRGB leds[NUM_PIXELS];

void setup() {

  Serial.begin(115200);

  SerialBT.begin("ESP32-Cube V1"); // Bluetooth device name
  EEPROM.begin(EEPROM_SIZE);

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS); // GRB ordering is typical

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  for (int i = 0; i <= 255; i += 10) {
    leds[LED0] = CRGB(i, 0, 0);
    leds[LED1] = CRGB(i, 0, 0);
    leds[LED2] = CRGB(i, 0, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i = 0; i <= 255; i += 10) {
    leds[LED0] = CRGB(0, i, 0);
    leds[LED1] = CRGB(0, i, 0);
    leds[LED2] = CRGB(0, i, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i = 0; i <= 255; i += 10) {
    leds[LED0] = CRGB(0, 0, i);
    leds[LED1] = CRGB(0, 0, i);
    leds[LED2] = CRGB(0, 0, i);
    FastLED.show();
    delay(5);
  }
  delay(300);
  leds[LED0] = CRGB::Black;
  leds[LED1] = CRGB::Black;
  leds[LED2] = CRGB::Black;
  FastLED.show();

  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  ledcAttachChannel(PWM1, BASE_FREQ, TIMER_BIT, PWM1_CH);
  Motor1_control(0);  

  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcAttachChannel(PWM2, BASE_FREQ, TIMER_BIT, PWM2_CH);
  Motor2_control(0);

  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcAttachChannel(PWM3, BASE_FREQ, TIMER_BIT, PWM3_CH);
  Motor3_control(0);

  EEPROM.get(0, offsets);   // Read EEPROM and get result into structure             
  if (offsets.ID == 96)     // Check if structuire ID is valid
    calibrated = true;      // if valid set calibrated to true

  delay(200);
  angle_setup();            // get current angles
}

void loop() {

  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();

    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;
    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;

  if (vertical_vertex && calibrated && !calibrating) {
      // *** Process standing on point
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroY = GyY / 131.0;
      gyroZ = GyZ / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;

      int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);

      motors_speed_X += speed_X / 5;
      motors_speed_Y += speed_Y / 5;
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
    } else if (vertical_edge && calibrated && !calibrating) {
      // Process standing on edge
        digitalWrite(BRAKE, HIGH);
        gyroX = GyX / 131.0;
        gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;

        int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);

        motors_speed_X += motor3_speed / 5;
        Motor3_control(pwm_X);
    }
    else {
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
    }
    previousT_1 = currentT;
  }

  if (currentT - previousT_2 >= 2000) {
    battVoltage((double)analogRead(VBAT) / BATTERY_CALIBRATION); // value 204 must be selected by measuring battery voltage!
    // DEBUG_INFO("Battery Reading : %4.2f, Calibrated Reading : %4.2f", (double)analogRead(VBAT), (double)analogRead(VBAT) / BATTERY_CALIBRATION);
    if (!calibrated && !calibrating)
    {
      SerialBT.println("first you need to calibrate the balancing points...");
      Serial.println("first you need to calibrate the balancing points (over bluetooth)...");
      
      if (!calibrated_leds) {
        leds[LED0] = CRGB(0, 255, 0);
        leds[LED1] = CRGB(0, 255, 0);
        leds[LED2] = CRGB(0, 255, 0);
        FastLED.show();
        calibrated_leds = true;
      }
      else {
        leds[LED0] = CRGB::Black;
        leds[LED1] = CRGB::Black;
        leds[LED2] = CRGB::Black;
        FastLED.show();
        calibrated_leds = false;
      }
    }
    previousT_2 = currentT;
  }
}
