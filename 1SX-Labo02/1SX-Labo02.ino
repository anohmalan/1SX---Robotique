#include "MeAuriga.h"

MeUltrasonicSensor ultraSensor(PORT_10);

#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0

MeRGBLed led(PORT0, LEDNUM);

enum State { NORMAL,
             RALENTI,
             DANGER,
             RONDE };

State currentState = NORMAL;

long currentTime = 0;

int detectDist = 80;
int dangerDist = 40;

int maxPwm = 200;
int halfPwm = 128;
int turnPwm = 150;

//Motor Left
const int m1_pwm = 11;
const int m1_in1 = 48;  // M1 ENA
const int m1_in2 = 49;  // M1 ENB

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 47;  // M2 ENA
const int m2_in2 = 46;  // M2 ENB

void setup() {
  Serial.begin(115200);

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m1_in1, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);

  currentTime = millis();

  led.setpin(LEDPIN);
}

void loop() {
  static unsigned long lastDebug = 0;
  static long statePrevious = 0;
  static long stateDelay = 5000;

  currentTime = millis();

  stateManager();
  ledState();

  if (currentTime - lastDebug >= 250) {
    lastDebug = currentTime;
    Serial.print("Distance: ");
    Serial.println(ultraSensor.distanceCm());
  }
}

void stateManager() {
  switch (currentState) {
    case NORMAL:
      normalMode();
      break;
    case RALENTI:
      ralentiMode();
      break;
    case DANGER:
      dangerMode();
      break;
    case RONDE:
      rondeMode();
      break;
  }
}

void normalMode() {

  if (ultraSensor.distanceCm() > detectDist) {

    currentState = NORMAL;

    digitalWrite(m1_in2, LOW);
    digitalWrite(m1_in1, HIGH);
    analogWrite(m1_pwm, maxPwm);

    digitalWrite(m2_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    analogWrite(m2_pwm, maxPwm);
  }

  else if (ultraSensor.distanceCm() < detectDist && ultraSensor.distanceCm() > dangerDist) {
    currentState = RALENTI;
    return;
  } else {
    currentState = DANGER;
    return;
  }
}

void ralentiMode() {

  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, halfPwm);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, halfPwm);

  if (ultraSensor.distanceCm() > detectDist) {
    currentState = NORMAL;
  } else if (ultraSensor.distanceCm() <= dangerDist) {
    currentState = DANGER;
  } else {
    currentState = RALENTI;
  }
}

void dangerMode() {
  static unsigned long previousTime = 0;
  static int step = 0;
  int rateStop = 500;
  int rateRecul = 1000;
  int rate = 2000;

  unsigned long currentTime = millis();

  switch (step) {
    case 0:  // STOP
      analogWrite(m1_pwm, 0);
      analogWrite(m2_pwm, 0);

      if (currentTime - previousTime >= rateStop) {
        step = 1;
        previousTime = currentTime;
      }
      break;

    case 1:  // RECUL
      digitalWrite(m1_in2, HIGH);
      digitalWrite(m1_in1, LOW);
      analogWrite(m1_pwm, maxPwm);

      digitalWrite(m2_in2, HIGH);
      digitalWrite(m2_in1, LOW);
      analogWrite(m2_pwm, maxPwm);

      if (currentTime - previousTime >= rateRecul) {
        step = 2;
        previousTime = currentTime;
      }
      break;

    case 2:

      digitalWrite(m1_in2, HIGH);
      digitalWrite(m1_in1, LOW);
      analogWrite(m1_pwm, maxPwm);

      digitalWrite(m2_in2, LOW);
      digitalWrite(m2_in1, LOW);
      analogWrite(m2_pwm, maxPwm);

      if (currentTime - previousTime >= rate) {
        step = 3;
        previousTime = currentTime;
      }
      break;
    case 3:
      if (ultraSensor.distanceCm() > detectDist) {
        currentState = NORMAL;
        step = 0;
        return;
      } else if (ultraSensor.distanceCm() > dangerDist) {
        currentState = RALENTI;
        step = 0;
        return;
      } else {
        currentState = DANGER;
        step = 0;
        return;
      }
      break;
  }
}

void ledState() {
  switch (currentState) {
    case NORMAL:
      // moitié avant en vert
      for (int i = 0; i < LEDNUM / 2; i++) led.setColor(i, 0, 255, 0);
      for (int i = LEDNUM / 2; i < LEDNUM; i++) led.setColor(i, 0, 0, 0);
      led.show();
      break;

    case RALENTI:
      // moitié arrière en bleu
      for (int i = 0; i < LEDNUM / 2; i++) led.setColor(i, 0, 0, 0);
      for (int i = LEDNUM / 2; i < LEDNUM; i++) led.setColor(i, 0, 0, 255);
      led.show();
      break;

    case DANGER:
      // anneau complet en rouge
      for (int i = 0; i < LEDNUM; i++) led.setColor(i, 255, 0, 0);
      led.show();
      break;
  }
}
void rondeMode() {
  unsigned long startTime = millis();
  unsigned long previousBlink = millis();
  const unsigned long blinkInterval = 250;
  bool ledsOn = false;
  int rate = 2000;

  Serial.println("Ronde Mode");

  while (millis() - startTime < rate) {
    unsigned long currentTime = millis();

    if (currentTime - previousBlink >= blinkInterval) {
      previousBlink = currentTime;
      ledsOn = !ledsOn;
      if (ledsOn) {
        led.setColor(0, 255, 0);
      } else {
        led.setColor(0, 0, 0);
      }
      led.show();
    }
  }
  currentState = NORMAL;
}
