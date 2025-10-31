//Author : Rodrigue Brim

#include "MeAuriga.h"

MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

#define DA6 3                 
#define DISTANCE_XX 180.0      
#define ARRET_YY 60.0    


MeRGBLed led(PORT0, 12);
MeUltrasonicSensor ultra(PORT_10);

#define LEDPIN 44
#define LEDNUM 12

#define PULSE 9
#define RATIO 39.267

enum State { SETUP,SEGMENT1, PIVOT, SEGMENT2, LIVRAISON, RETOUR_SEGMENT2, PIVOT_BONUS, RETOUR_SEGMENT1, TERMINE };
State currentState = SETUP;


const int speed = 120;
const float vitesse = 40.0; 

float tempsMis;

float distParcourue = 0.0;
float segment2Distance = 0.0;
unsigned long lastTime = 0;
float distanceCm = 0;
unsigned long lastUltraRead = 0;
bool deliveryDone = false;

unsigned long blinkStart = 0;
int blinkStep = 0;
bool blinking = false;

unsigned long animTimer = 0;
int animIndex = 0;

int angle = 90;

void clearLeds() {
  for (int i = 0; i < LEDNUM; ++i) led.setColor(i, 0, 0, 0);
  led.show();
}

void updateProgressBar(float progress) {
  if (progress < 0.0) progress = 0.0;
  int ledsToLight = (int)(progress * LEDNUM + 0.0001);
  if (ledsToLight > LEDNUM) ledsToLight = LEDNUM;
  for (int i = 0; i < LEDNUM; ++i) {
    if (i < ledsToLight) led.setColor(i, 0, 255, 0);
    else led.setColor(i, 0, 0, 0);
  }
  led.show();
}

void startBlink() {
  blinking = true;
  blinkStart = millis();
  blinkStep = 0;
}

void blinkLoop() {
  if (!blinking) return;
  unsigned long now = millis();
  if (now - blinkStart >= 250) {
    blinkStart = now;
    blinkStep++;
    if (blinkStep / 2 < 6) {
      for (int i = 0; i < LEDNUM; ++i)
        led.setColor(i, (blinkStep % 2 == 0) ? 0 : 0, (blinkStep % 2 == 0) ? 255 : 0, 0);
      led.show();
    } else {
      blinking = false;
      clearLeds();
    }
  }
}


void retourAnimation() {
  if (millis() - animTimer < 150) return;
  animTimer = millis();

  clearLeds();
    led.setColor(animIndex % LEDNUM, 0, 0, 255);
    animIndex--;
    if (animIndex < 0) animIndex = LEDNUM - 1;
  led.show();
}


// ********* INTERRUPTIONS ***********

void rightEncoderInterrupt(void){
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

// ************* DÉBUT ************

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}

void encoderLoop() {
  encoderLeft.loop();
  encoderRight.loop();
}

void setupState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  static unsigned long exitTime = 0;

  const int timeout = 3000;

  if (firstTime) {
    firstTime = false;
    exitTime = ct + timeout;

    Serial.println("Attente de 3 secondes avant de démarrer.");
  }

  // Là la là... j'attends

  if (ct >= exitTime) {
    firstTime = true;
    currentState = SEGMENT1;
  }
}

void Stop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

void avanceState(short speed = 100, short firstRun = 0) {
    static double zAngleGoal = 0.0;
    
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    const double kp = 6.75;
    const double kd = 1.0;

  if (firstRun) {
    gyro.resetData();
    zAngleGoal = gyro.getAngleZ();
    previousError = 0;
    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;
  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);

  previousError = error;

  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed - output);
}

bool spinning = false;
float spinTarget = 0.0;
double previousSpinError = 0.0;

void startSpin(float angleDeg) {
  gyro.resetData();
  spinTarget = angleDeg;
  previousSpinError = 0;
  spinning = true;
}

void spinLoop(float maxSpeed = 40) {
  const double kpspin = 10.0;
  const double kdspin = 1.0; 
  const double margeError = 1.5;
  if (!spinning) return;

  float currentAngle = gyro.getAngleZ();
  float error = spinTarget - currentAngle;
  double output = kpspin * error + kdspin * (error - previousSpinError);
  previousSpinError = error;

  if (output > maxSpeed) output = maxSpeed;
  if (output < -maxSpeed) output = -maxSpeed;

  encoderLeft.setMotorPwm(output);
  encoderRight.setMotorPwm(output);

  if (abs(error) < margeError) {
    spinning = false;
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    Serial.println("Pivot terminé !");
  }
}


void setup() {
  Serial.begin(115200);
  led.setpin(LEDPIN);
  encoderConfig();
  gyro.begin();

  currentState = SETUP;
}

float updateTime(){
  unsigned long cT = millis();
  float second = 1000.0;
  float time;

  tempsMis = (cT - lastTime) / second;
  time = tempsMis;
  lastTime = cT;
  return time;
}

void updateDistance(){
  distParcourue += vitesse * updateTime();
}


void gererEtat(unsigned long currentTime) {
  blinkLoop();
  const int marge =15;
    switch (currentState) {
      case SETUP:
        setupState(currentTime);
        break;
      case SEGMENT1:
        updateDistance();
        avanceState(speed);
        updateProgressBar(distParcourue / DISTANCE_XX);
        if (distParcourue >= DISTANCE_XX) {
          clearLeds();
          
          startSpin(angle);
          currentState = PIVOT;
        }
        break;

      case PIVOT:
        spinLoop(speed);
        if (!spinning) {
          distParcourue = 0;
          gyro.resetData();
          avanceState(speed, 1);
          currentState = SEGMENT2;
        }
        break;

      case SEGMENT2:
        if (distanceCm > ARRET_YY) {
          updateDistance();
          avanceState(speed);
        } else {
          Stop();
          segment2Distance = distParcourue;
          currentState = LIVRAISON;
          startBlink();
        }
        break;

      case LIVRAISON:
        if (!deliveryDone && !blinking) {
          deliveryDone = true;
          distParcourue = 0;
          gyro.resetData();
          avanceState(-speed, 1);
          currentState = RETOUR_SEGMENT2;
        }
        break;

      case RETOUR_SEGMENT2:
        retourAnimation();
        updateDistance();
        avanceState(-speed);
        if (distParcourue +marge >= segment2Distance) {
          Stop();
          distParcourue = 0;
          startSpin(-angle);  
          currentState = PIVOT_BONUS;
        }
        break;

      case PIVOT_BONUS:
        retourAnimation();
        spinLoop(speed);
        if (!spinning) {
          distParcourue = 0;
          gyro.resetData();
          avanceState(-speed, 1);
          currentState = RETOUR_SEGMENT1;
        }
        break;

      case RETOUR_SEGMENT1:
      int delay = 5000;
        retourAnimation();
        updateDistance();
        avanceState(-speed);
        if (distParcourue -marge >= DISTANCE_XX) {
          Stop();
          clearLeds();
          for (int i = 0; i < LEDNUM; ++i) led.setColor(i, 255, 0, 0);
          led.show();
          unsigned long t = millis();
          while (millis() - t < delay) { gyro.update(); encoderLoop(); } 
          clearLeds();
          currentState = TERMINE;
        }
        break;

      case TERMINE:
        Stop();
        break;
  }
}


void loop() {
  unsigned long currentTime = millis();
  gyro.update();
  encoderLoop();

  lastTime = currentTime;

  if (currentTime - lastUltraRead >= 100) {
    lastUltraRead = currentTime;
    float d = ultra.distanceCm();
    if (d > 0 && d < 400) distanceCm = d; 
  }

  gererEtat(currentTime);
}