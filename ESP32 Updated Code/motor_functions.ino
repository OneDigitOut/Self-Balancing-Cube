#include "Cube_ESP32.h"

void pwmSet(uint8_t pin, uint32_t value) {
  ledcWrite(pin, value);
}

void Motor1_control(int sp) {
  sp = sp + motor1_speed;
  if (sp < 0) 
    digitalWrite(DIR1, LOW);
  else 
    digitalWrite(DIR1, HIGH);
  pwmSet(PWM1, 255 - abs(sp));
}

void Motor2_control(int sp) {
  sp = sp + motor2_speed;
  if (sp < 0) 
    digitalWrite(DIR2, LOW);
  else 
    digitalWrite(DIR2, HIGH);
  pwmSet(PWM2, 255 - abs(sp));
}

void Motor3_control(int sp) {
  sp = sp + motor3_speed;
  if (sp < 0) 
    digitalWrite(DIR3, LOW);
  else 
    digitalWrite(DIR3, HIGH);
  pwmSet(PWM3, 255 - abs(sp));
}

void ENC1_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

void ENC2_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

void ENC3_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}
