#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Motor.h"
#include <NewPing.h>
#include "RBFlameSensor.h"

#define R_MOTOR_SPEED_PIN 3
#define R_MOTOR_CONTROL1_PIN 4
#define R_MOTOR_CONTROL2_PIN 5

#define L_MOTOR_SPEED_PIN 8
#define L_MOTOR_CONTROL1_PIN 6
#define L_MOTOR_CONTROL2_PIN 7

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

#define R_ENCODER_PIN 44
#define L_ENCODER_PIN 45

const int WAIT = 0;
const int NAV_RIGHT = 1;
const int CENTER = 2;
const int PUT_OUT = 3;

const int ROTATE_POWER = 8;
const int RIGHT_DIST = 12;
const int BASE_POWER = 8;
const float GAIN = 0.9; 
//MODIFICANDO-O IRÁ INFLU8NCIAR NA CORREÇÃO QUE ELE FARÁ AO DETECTAR A PAREDE
const int DELTA_LIMIT = 7;
const int FRONT_DIST = 17;

const int LINE_TAG = 0;
const int CIRCLE_TAG = 1;
const int NO_TAG = 2;
const int PULSE = 10.525; //MM por pulso

LiquidCrystal_I2C Lcd(0x27, 16, 2);

Motor LMotor(L_MOTOR_SPEED_PIN, L_MOTOR_CONTROL1_PIN, L_MOTOR_CONTROL2_PIN, 70, 255, 32);
Motor RMotor(R_MOTOR_SPEED_PIN, R_MOTOR_CONTROL1_PIN, R_MOTOR_CONTROL2_PIN, 68, 255, 32);
NewPing LSonar(L_SONAR_PIN, L_SONAR_PIN, 100);
NewPing FSonar(F_SONAR_PIN, F_SONAR_PIN, 100);
NewPing RSonar(R_SONAR_PIN, R_SONAR_PIN, 100);
RBFlameSensor flameSensor(3);

int state = WAIT;
int xFlame, yFlame;
bool flame = false; 
int room = 0;
bool flameInRoom = false;

void setup() {
  // ===== Inicialização de objetos ===== 
  Serial.begin(9600);  
  Wire.begin();
  Lcd.init();
  Lcd.backlight();
  flameSensor.init();
  // ===== Configuração de pinos =====
  for(int i = 2; i < 10; i++)
    pinMode(i, OUTPUT);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  pinMode(L_BUMPER_PIN, INPUT_PULLUP);
  pinMode(R_BUMPER_PIN, INPUT_PULLUP);

  pinMode(LED_PIN , OUTPUT);
 }
 
void loop() {
  showState();
  //moveForward(100, 110);
switch(state) {
    case WAIT:
      state = waitState();     
      break;
    case NAV_RIGHT:
      state = navRightStatev1();
      break;
    case CENTER:
      state = centerState();  
      break;
    case PUT_OUT:
      state = putOutState();
      break;  
  }
  if(digitalRead(STOP_BUTTON_PIN) == LOW){
       Serial.println("Stop");
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
int navRightStatev1() {
  // SE DETECTAR CHAMA
  bool isFlame = flameSensor.update();
  if(isFlame == true){     //ESSE IF AQUI
  digitalWrite(LED_PIN, HIGH);
  return CENTER;
  } 
  if(getDistance(FSonar) < FRONT_DIST)
    rotateAngle(45);

  checkBumpers();

  int dist = getDistance(RSonar);
  if (dist < 5)
    rotateAngle(45);
  int error = dist - RIGHT_DIST;
  int delta = error * GAIN;

  if(delta > DELTA_LIMIT)
    delta = DELTA_LIMIT;
  
  move(BASE_POWER, delta);
  return NAV_RIGHT;
}
int navRightStatev2(){
  // SE DETECTAR CHAMA
  int tag = getFloorTagD();
  bool isFlame = false;
  if(tag == LINE_TAG){
    room++; 
    brake();
    delay(1000);
    moveCrash(BASE_POWER, 0, 1000); //MAYBE
    rotateAngle(180);   //MAYBE
    if (bool isFlame = flameSensor.update() == true){
      flameInRoom = true;
      digitalWrite(LED_PIN, HIGH);    //SUBSTITUI O IF DE BAIXO PARA OTIMIZAR O CODIGO 
      return CENTER;
    }
    else
      maneuverToGoToNextRoom();
  }
  else if (tag == CIRCLE_TAG){
    if (flame == true) meneuverToGoToIslandRoom();
  } 
  if(getDistance(FSonar) < FRONT_DIST)
    rotateAngle(90);

  checkBumpers();

  int dist = getDistance(RSonar);

  if (dist < 5)
    rotateAngle(45);
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
  
  switch (getdir){  
    case 1: rotate(-4); break;  //Chama a direita
    case 2: 
      moveForward(100, 110);         //Chama a frente
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
  showFlameData(isFlame, getdir);
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
  rotateAngle(180);
  return  NAV_RIGHT; //O ORIGINAL ERA WAIT
}
void meneuverToGoToIslandRoom(){
  rotateAngle(180);
  moveCrash(BASE_POWER, 0, 2000);
}
void maneuverToGoToNextRoom(){
  switch(room){
    case 1: moveCrash(-BASE_POWER, 0, 1000); rotateAngle(180);  break;
    case 2: moveCrash(-BASE_POWER, 0, 1000); rotateAngle(180);  break;
    case 3: moveCrash(-BASE_POWER, 0, 1000); rotateAngle(180);  break;
    default: centerState();
  }
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
    delay(angle * (1370 / 180));  
  } else {
    rotate(-ROTATE_POWER);
    delay(-angle * (1370 / 180));  
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
  if(digitalRead(R_LINE_SENSOR_PIN) == LOW){
      moveCrash(BASE_POWER, 0, 1000);      
      
      if(digitalRead(R_LINE_SENSOR_PIN) == LOW){
          return CIRCLE_TAG;
      }
      else
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
  if(digitalRead(L_BUMPER_PIN) == LOW && digitalRead(R_BUMPER_PIN) == LOW) 
    moveCrash(-BASE_POWER, 0, 1000);
  else if(digitalRead(L_BUMPER_PIN) == LOW) 
    moveCrash(-BASE_POWER, 4, 1000);
  else if(digitalRead(R_BUMPER_PIN) == LOW) 
    moveCrash(-BASE_POWER, -4, 1000);
}
void encoder(){
  if (digitalRead(R_ENCODER_PIN) == HIGH){
    //UMA VOLTA NO ENCODER DIREITO
  }
  if (digitalRead(L_ENCODER_PIN) == HIGH){
    //UMA VOLTA NO ENCODER ESQUERDO
  }
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
void showFlameData(bool flame, int dir){
  msg(1, 0, "F: " + String(flame), true);
  msg(1, 0, "D: " + String(dir), false);
}
