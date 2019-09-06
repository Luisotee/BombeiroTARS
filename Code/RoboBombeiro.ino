#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Motor.h"
#include <NewPing.h>
#include "RBFlameSensor.h"

#define R_MOTOR_SPEED_PIN 2
#define R_MOTOR_CONTROL1_PIN 3
#define R_MOTOR_CONTROL2_PIN 4

#define L_MOTOR_SPEED_PIN 6
#define L_MOTOR_CONTROL1_PIN 7
#define L_MOTOR_CONTROL2_PIN 5

#define START_BUTTON_PIN 25
#define STOP_BUTTON_PIN 30

#define L_SONAR_PIN 36
#define F_SONAR_PIN 34
#define R_SONAR_PIN 38

#define L_BUMPER_PIN 52
#define R_BUMPER_PIN 53

#define LED_PIN 24
#define FAN_MOTOR_PIN 9

#define R_LINE_SENSOR_PIN 15

const int WAIT = 0;
const int NAV_RIGHT = 1;
const int CENTER = 2;
const int PUT_OUT = 3;

const int ROTATE_POWER = 8;
const int RIGHT_DIST = 20;
const int BASE_POWER = 8;
const float GAIN = 1;      //MODIFICANDO-O IRÁ INFLUÊNCIAR NA CORREÇÃO QUE ELE FARÁ AO DETECTAR A PAREDE
const int DELTA_LIMIT = 6;
const int FRONT_DIST = 20;

const int LINE_TAG = 0;
const int CIRCLE_TAG = 1;
const int NO_TAG = 2;

LiquidCrystal_I2C Lcd(0x27, 16, 2);

Motor LMotor(L_MOTOR_SPEED_PIN, L_MOTOR_CONTROL1_PIN, L_MOTOR_CONTROL2_PIN, 80, 255, 32);
Motor RMotor(R_MOTOR_SPEED_PIN, R_MOTOR_CONTROL1_PIN, R_MOTOR_CONTROL2_PIN, 80, 255, 32);
NewPing LSonar(L_SONAR_PIN, L_SONAR_PIN, 100);
NewPing FSonar(F_SONAR_PIN, F_SONAR_PIN, 100);
NewPing RSonar(R_SONAR_PIN, R_SONAR_PIN, 100);
RBFlameSensor flameSensor(3);

int state = WAIT;
int xFlame, yFlame;

void setup() {
  // ===== Inicialização de objetos ===== 
  Serial.begin(9600);  
  Wire.begin();
  Lcd.init();
  Lcd.backlight();
  flameSensor.init();
  // ===== Configuração de pinos =====
  for(int i = 3; i < 9; i++)
    pinMode(i, OUTPUT);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  pinMode(L_BUMPER_PIN, INPUT_PULLUP);
  pinMode(R_BUMPER_PIN, INPUT_PULLUP);

  pinMode(LED_PIN , OUTPUT);
  pinMode(FAN_MOTOR_PIN , OUTPUT);
 }
 
void loop() {
  switch(state) {
    case WAIT:
      state = waitState();     
      break;
    case NAV_RIGHT:
      state = navRightState();
      break;
    case CENTER:
      state = centerState();  
      break;
    case PUT_OUT:
      state = putOutState();
      break;  
  }
  showState();
  if(digitalRead(STOP_BUTTON_PIN) == LOW){
       state = WAIT;
  }
}
// ================================================================================
// Estados
// ================================================================================
int waitState() {
  stop();
  do {
     
  }while(digitalRead(START_BUTTON_PIN) == HIGH);

  return NAV_RIGHT;
}
int navRightState() {
  // SE DETECTAR CHAMA
  if(flameSensor.update() == true){
  digitalWrite(LED_PIN, HIGH);
  return CENTER;
  }
  
  if(getDistance(FSonar) < FRONT_DIST)
    rotateAngle(90);

  checkBumpers();

  int dist = getDistance(RSonar);
  int error = dist - RIGHT_DIST;
  int delta = error * GAIN;     

  if(delta > DELTA_LIMIT)
    delta = DELTA_LIMIT;
  
  move(BASE_POWER, delta);
  return NAV_RIGHT;
}
int centerState(){
  bool isFlame = flameSensor.update();
  int getdir = flameSensor.getDir();
  
  switch (getdir){  //PERGUNTAR AO PROFESSOR
    case 1: rotate(-4); break;  //Chama a direita
    case 2: move(4, 0);         //Chama a frente
      if(getDistance(FSonar) < 15){
        brake();
        return PUT_OUT; 
      }
      break;  
    case 3: rotate(4); break;  //Chama a esquerda
    case -1: brake(); 
    return PUT_OUT; 
  }
  /*if (y > limite) {
    return PUT_OUT; 
  }*/
  return NAV_RIGHT;
}
int putOutState(){
  //Apagar chama
  digitalWrite(FAN_MOTOR_PIN, HIGH);
  delay(3000);
  digitalWrite(FAN_MOTOR_PIN, LOW);
  
  moveCrash(-4, 0, 1500);
  
  if (flameSensor.update() == true){
    digitalWrite(LED_PIN, HIGH);
    return CENTER;  //SE CHAMA AINDA PRESENTE
  }
  digitalWrite(LED_PIN, LOW);
  return  WAIT; //NAV_RIGHT PARA VOLTAR AO COMEÇO
}
// ================================================================================
// Atuadores
// ================================================================================
void move(int power, int delta) {
  LMotor.setPower(power + delta);
  RMotor.setPower(power - delta);
}
void moveCrash(int power, int delta, int time) {
  LMotor.setPower(power + delta);
  RMotor.setPower(power - delta);
  delay(time);
  brake();
}
void stop(void) {
  LMotor.stop();
  RMotor.stop();
}
void brake(void) {
  LMotor.brake();
  RMotor.brake();
}
void rotate(int power) {
  LMotor.setPower(-power);
  RMotor.setPower(power);
}
void rotateAngle(int angle) {
  if(angle >= 0) {
    rotate(ROTATE_POWER);
    delay(angle * (610 / 180));  
  } else {
    rotate(-ROTATE_POWER);
    delay(-angle * (610 / 180));  
  } 
  brake();
}
void moveForward(char lSpeed, char rSpeed) {
  analogWrite(L_MOTOR_SPEED_PIN, lSpeed);
  digitalWrite(L_MOTOR_CONTROL1_PIN, HIGH);
  digitalWrite(L_MOTOR_CONTROL2_PIN, LOW);

  analogWrite(R_MOTOR_SPEED_PIN, rSpeed);
  digitalWrite(R_MOTOR_CONTROL1_PIN, HIGH);
  digitalWrite(R_MOTOR_CONTROL2_PIN, LOW);
}
// ================================================================================
// Sensores
// ================================================================================
int getDistance(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);

  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);

  pinMode(pin, INPUT);
  long duration = pulseIn(pin, HIGH);
  int distance = duration / 58;  
  
  return distance;
}
int getFloorTagD(){                               //RECOMENDADO
  if(digitalRead(R_LINE_SENSOR_PIN), LOW){
      moveCrash(BASE_POWER, 0, 1000);      
      
      if(digitalRead(R_LINE_SENSOR_PIN), LOW){
          return CIRCLE_TAG;
      }
   return LINE_TAG;
   }
  return NO_TAG;  
  }
int getFloorTagA(){
  if(analogRead(R_LINE_SENSOR_PIN) <  150){         //BRANCO = ~50, PRETO ~600
      moveCrash(BASE_POWER, 0, 1000);      
      
      if(analogRead(R_LINE_SENSOR_PIN) < 150){
          return CIRCLE_TAG;
      }
   return LINE_TAG;
   }
  return NO_TAG;  
}
int getDistance(NewPing &sonar) {
  int dist = sonar.ping_cm();
  if(dist == 0) dist = 100;

  return dist;
}
void checkBumpers() {
  if(digitalRead(L_BUMPER_PIN) == HIGH && digitalRead(R_BUMPER_PIN) == HIGH) 
    moveCrash(-BASE_POWER, 0, 1000);
  else if(digitalRead(L_BUMPER_PIN) == HIGH) 
    moveCrash(-BASE_POWER, 4, 1000);
  else if(digitalRead(R_BUMPER_PIN) == HIGH) 
    moveCrash(-BASE_POWER, -4, 1000);
}
// ================================================================================
// Interface
// ================================================================================
void msg(int linhas, int colunas, String string, bool apagar) {
  if(apagar == true) {
    Lcd.setCursor(0, 1);
    Lcd.print("                ");
  }
  Lcd.setCursor(colunas, linhas);
  Lcd.print(string);
}
void showState(){
  switch(state){
    case WAIT:
      msg(0, 0, "WAIT", true);
      break;
    case NAV_RIGHT:
      msg(0, 0, "NAV_RIGHT", true);
      break;
    case CENTER:
      msg(0, 0, "CENTER", true);
      break;
    case PUT_OUT:
      msg(0, 0, "PUT_OUT", true);        
      break;
  } 
}
