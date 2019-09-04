#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "Motor.h"
 
#define L_MOTOR_SPEED_PIN 3
#define L_MOTOR_CONTROL1_PIN 4
#define L_MOTOR_CONTROL2_PIN 5

#define R_MOTOR_SPEED_PIN 8
#define R_MOTOR_CONTROL1_PIN 6
#define R_MOTOR_CONTROL2_PIN 7

#define START_BUTTON_PIN 25
#define STOP_BUTTON_PIN 30

const int ROTATE_POWER = 10;

  // ===== Objetos =====
LiquidCrystal_I2C Lcd(0x27, 16, 2);
Motor LMotor(L_MOTOR_SPEED_PIN, L_MOTOR_CONTROL1_PIN, L_MOTOR_CONTROL2_PIN, 80, 255, 32);  
Motor RMotor(R_MOTOR_SPEED_PIN, R_MOTOR_CONTROL1_PIN, R_MOTOR_CONTROL2_PIN, 80, 255, 32);

void setup() {
  // ===== Inicialização de Objetos =====
  Wire.begin();
  Lcd.init();
  Lcd.backlight(); 
  // ===== Pinos =======
  for (int i = 3; i < 9; i++){
      pinMode(i, OUTPUT);
  }
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  // ====== CODE =======
  msg(0, 0, "TARS vAlpha", false);
  waitStartButton();
  /*move(16, 0);
  delay(2000);
  rotate(8);
  delay(3000);
  move(-16, -4);
  delay(2500);
  stop();
  */
  rotateAngle(90);
}

void loop() {
}
//====================================
//ATUADORES
//====================================
void move(int power, int turn){
  LMotor.setPower(power + turn);
  RMotor.setPower(power - turn);
}
void stop(){
  LMotor.stop();
  RMotor.stop();
}
void rotate(int power){
  LMotor.setPower(-power);
  RMotor.setPower(power);
}
void rotateAngle(int angle){
  if (angle >= 0){
  rotate(ROTATE_POWER);
  delay(angle * 610 / 180);
}
  else{
    rotate(-ROTATE_POWER);
    delay(-angle * 610/180); 
  }
  brake();
}
void brake(){
  LMotor.brake();
  RMotor.brake();
}
//====================================
// INTERFACE
//====================================
void msg(int linhas, int colunas, String string, bool d){
  if(d){
    Lcd.setCursor(0, 1);
    Lcd.print("                ");
  }
  Lcd.setCursor(colunas, 1);
  Lcd.print(string);
}

void waitStartButton(){
  do{
    
  } while(digitalRead(START_BUTTON_PIN) == HIGH);
}
