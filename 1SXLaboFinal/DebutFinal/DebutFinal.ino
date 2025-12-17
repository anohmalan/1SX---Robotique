//Author : Rodrigue Brim

#include <Adafruit_seesaw.h>
#include <MeAuriga.h>
#include <Wire.h>
// StaticJsonDocument<128> doc;

#define NB_IR 5
#define PULSE 9
#define LEDPIN 44
#define LEDNUM 12
#define DIST_WHEEL 155

int speedMin = 150;
int speed = 100;
int speedMax = 255;
int pwm = 80;
float pi = 3.1416;
int rangerPwm = 80;
float RATIO = 39.267;
float DIAMETER = 6.5;
float DISTANCE_XX = 30.0;
unsigned long currentTime;
bool generalFirstTime = false;
float circumference = DIAMETER * pi;

MeGyro gyro(0, 0x69);
MeRGBLed led(PORT0, 12);
MeUltrasonicSensor ultra(PORT_10);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

bool firstRun = true;
bool firstPivot = true;

bool firstTime = true;
bool firstCalibration = true;

int distCheckA = 50;

struct Capteur {
  int valeurLue;
  int valeurNormalisee;
  int valMin = 1023;
  int valMax = 0;
  int seuil;
  bool onLine = false;
};
Capteur capteurs[5];

Adafruit_seesaw ss;

int sensorValues[NB_IR];  // Tableau pour stocker les valeurs des capteurs

enum State { DEMARRAGE,
             AUTO,
             STOP,
             MAN,
             MISSION };
State currentState;

enum StateAuto { SETUP,
                 AVANCE,
                 CHECKPOINTA,CHECKPOINTB,
                 LINETRACK };
StateAuto ctAuto;



int chronometre = 0;
bool chrono = false;

void setup() {
  unsigned long currentTime = millis();
  Serial.begin(115200);
  led.setpin(LEDPIN);

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1)
      ;
  }
  encoderConfig();
  gyro.begin();

  Serial.println("Connexion réussie au LyneTracker!");

  currentState = DEMARRAGE;
  ctAuto = SETUP;
  clearLeds();
  delay(1000);
}

#pragma region INTERRUPTION_GYRO

// ********* INTERRUPTIONS ***********

void rightEncoderInterrupt(void) {
  if (digitalRead(encoderRight.getPortB()) == 0) {
    encoderRight.pulsePosMinus();
  } else {
    encoderRight.pulsePosPlus();
    ;
  }
}

void leftEncoderInterrupt(void) {
  if (digitalRead(encoderLeft.getPortB()) == 0) {
    encoderLeft.pulsePosMinus();
  } else {
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

  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setPosPid(1.8, 0, 1.2);

  encoderRight.setSpeedPid(0.18, 0, 0);
  encoderLeft.setSpeedPid(0.18, 0, 0);

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

// void encoderLoop() {
//   encoderLeft.loop();
//   encoderRight.loop();
// }

void offMotors() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

#pragma endregion

#pragma region updates

void gyroTask(unsigned long ct) {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

#pragma endregion

void calibrer() {

  for (int i = 0; i < NB_IR; i++) {
    sensorValues[i] = ss.analogRead(i);

    capteurs[i].valeurLue = sensorValues[i];

    if (capteurs[i].valeurLue < capteurs[i].valMin) {
      capteurs[i].valMin = capteurs[i].valeurLue;
    }
    if (capteurs[i].valeurLue > capteurs[i].valMax) {
      capteurs[i].valMax = capteurs[i].valeurLue;
    }
    capteurs[i].seuil = (capteurs[i].valMax + capteurs[i].valMin) / 2;
  }
}
// bool faireTourComplet(int turningAngle) {
//   static bool first = true;
//   static float startAngle = 0;
//   static float zAngleGoal = 0;
//   static double error = 0.0;
//   static double previousError = 0.0;
//   static double output = 0.0;
//   const int turnSpeed = 40;

//   const double kp = 1.0;
//   const double kd = 0.2;
//   const float angleMargin = 2.5;

//   if (first) {
//     first = false;
//     encoderLeft.setMotorPwm(turnSpeed);
//     encoderRight.setMotorPwm(turnSpeed);

//     zAngleGoal = gyro.getAngleZ() + turningAngle;
//     startAngle = gyro.getAngleZ();
//   }


//   error = gyro.getAngleZ() - zAngleGoal;
//   output = kp * error + kd * (error - previousError);
//   output = constrain(output, -turnSpeed, turnSpeed);


//   previousError = error;

//   encoderLeft.setMotorPwm(turnSpeed - output);
//   encoderRight.setMotorPwm(turnSpeed - output);

//   float currentAngle = gyro.getAngleZ();
//   float deltaAngle = currentAngle - zAngleGoal;

//   if (deltaAngle > 180) deltaAngle -= 360;
//   if (deltaAngle < -180) deltaAngle += 360;

//   bool finished = fabs(deltaAngle) <= angleMargin;

//   if (finished) {
//     encoderLeft.setMotorPwm(pwm);
//     encoderRight.setMotorPwm(pwm);
//     first = true;
//     return true;
//   } else {

//     return false;
//   }
// }

  double getUnwrappedAngle(bool reset = false) {
    static double lastWrapped = 0.0;
    static double total = 0.0;
    double tourCicle = 360.0;
    double halfCircle = 180.0;

    if (reset) {
      lastWrapped = gyro.getAngleZ();
      total = lastWrapped;
      return total;
    }

    double wrapped = gyro.getAngleZ(); 

    double diff = wrapped - lastWrapped;

    if (diff > halfCircle)  diff -= tourCicle;
    if (diff < -halfCircle) diff += tourCicle;

    total += diff;
    lastWrapped = wrapped;

    return total;
  }

  bool faireTourComplet(int targetAngle){
    
    static double zAngleGoal = 0.0;
  short tolerance = 2;
    if (firstPivot) {
      gyro.resetData();
      getUnwrappedAngle(true);           // reset interne
      zAngleGoal = getUnwrappedAngle() + targetAngle;
      firstPivot = false;
      return false;
    }

    double current = getUnwrappedAngle();
    double remaining = zAngleGoal - current;

    Serial.println(current);
    Serial.println(remaining);

    if (fabs(remaining) <= tolerance) {
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      return true;
    }

    // Ajuste dynamiquement la vitesse pour la précision
    double absRem = fabs(remaining);
    int pwm;

    if (absRem > 30) pwm = speed;
    else if (absRem > 10) pwm = speed * 0.6;
    else if (absRem > 3) pwm = speed * 0.3;
    else pwm = speed * 0.15;

    pwm *= (remaining > 0 ? 1 : -1);

    encoderLeft.setMotorPwm(pwm);
    encoderRight.setMotorPwm(pwm);

    return false;
  }

float computePID(float position, float consigne = 2.0f) {
  // Ajuster les coefficients selon vos besoins
  static float kp = 0.65;  // Coefficient proportionnel  //o.65 sa marche avec roue de coté
  static float ki = 0.01;  // Coefficient intégral
  static float kd = 0.01;  // Coefficient dérivé

  static float integral = 0;
  static float derivative = 0;
  static float lastError = 0;

  float error = position - consigne;

  integral += error;

  // Adapter cette valeur selon les besoins de votre application
  const float integralLimit = 1000;

  // Limiter l'intégrale pour éviter l'emballement intégral
  integral = constrain(integral, -integralLimit, integralLimit);

  derivative = error - lastError;
  lastError = error;

  float output = kp * error + ki * integral + kd * derivative;

  return output;
}
bool trackLine() {
  bool detected = false;

  for (int i = 0; i < NB_IR; i++) {

    capteurs[i].onLine = (capteurs[i].valeurLue < capteurs[i].seuil);

    if (capteurs[i].onLine) {
      detected = true;
    }
  }

  return detected;
}

bool detectObjet(int dist) {
  int distance = ultra.distanceCm();

  if (distance >= dist) {
    // offMotors();
    // faireTourComplet(180);
    return true;
  } else {
    return false;
  }
}

float calculerPositionLigne() {
  float numerateur = 0;
  float denominateur = 0;

  for (int i = 0; i < NB_IR; i++) {
    numerateur += capteurs[i].valeurNormalisee * (i - 2);
    denominateur += capteurs[i].valeurNormalisee;
  }
  return numerateur / denominateur * 1000;
}

float capteurLectureNormalisee(int index) {
  return ((capteurs[index].valeurLue - capteurs[index].valMin) * 1.0) / (capteurs[index].valMax - capteurs[index].valMin) * 1000.0;
}
void normaliserValeurs() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].valeurNormalisee = capteurLectureNormalisee(i);
  }
}

void suivreLigne(float adjustement) {

  encoderLeft.setMotorPwm(rangerPwm - adjustement);
  encoderRight.setMotorPwm(-rangerPwm - adjustement);
}

float getCurrentPos() {
  float leftPulsPos = encoderLeft.getPulsePos();
  float rightPulsPos = encoderRight.getPulsePos();


  float leftDistanceCM = (leftPulsPos / (PULSE * RATIO)) * (DIAMETER * pi);
  float rightDistanceCM = (rightPulsPos / (PULSE * RATIO)) * (DIAMETER * pi);


  return (leftDistanceCM + rightDistanceCM) / 2;
}

void sensorTask() {

  for (int i = 0; i < NB_IR; i++) {
    sensorValues[i] = ss.analogRead(i);

    capteurs[i].valeurLue = sensorValues[i];
  }
}
bool isAtIntersection() {
  for (int i = 0; i < NB_IR; i++) {
    if (capteurs[i].valeurLue >= capteurs[i].seuil) {
      return false;  // Si au moins un capteur ne détecte pas la ligne, ce n'est pas une intersection
    }
  }
  return true;  // Tous les capteurs détectent la ligne
}
int determineDirection() {
  int leftSum = capteurs[0].valeurLue + capteurs[1].valeurLue + capteurs[2].valeurLue;
  int rightSum = capteurs[3].valeurLue + capteurs[4].valeurLue;

  if (leftSum > rightSum) {
    return -1;
  } else {
    return 1;
  }
}
void handleIntersection() {
  int direction = determineDirection();

  if (direction == -1) {
    // Tourner à gauche
    faireTourComplet(-90);
  } else {
    // Tourner à droite
    faireTourComplet(90);
  }
}
// void mission01() {
//   float consigne = 0.0f;  // Position centrale

//   if (firstCalibration) {
//     calibrationAutomatique();
//   } else {
//     if (trackLine()) {
//       if (trackLine()) {
//         if (isAtIntersection()) {
//           handleIntersection();
//         } else {
//           normaliserValeurs();
//           float position = calculerPositionLigne();
//           float adjustment = computePID(position, consigne);
//           suivreLigne(adjustment);
//         }

//       } else {
//         static bool turningBack = false;
//         static bool doneTurn = false;

//         if (!turningBack) {
//           turningBack = true;
//           doneTurn = false;
//         }

//         if (turningBack && !doneTurn) {
//           doneTurn = faireTourComplet(180);  // tourne de 180°
//         }

//         if (doneTurn) {
//           turningBack = false;
//           doneTurn = false;
//         }
//       }
//     }
//   }
// }
void mission01(){
   float consigne = 0.0f; // Position centrale

    //logique prochaine note 
    //si 012 cest true pour angle gauche , 234 pour angle de droite pis 01234 pour le t qui sera en premier dans le if , cest pour sa que ya un on line chaque capteur et non un global
      if(trackLine()){
          normaliserValeurs();

          // Calculer la position de la ligne
          float position = calculerPositionLigne();

          // Calculer l'ajustement à apporter à la trajectoire
          float adjustment = computePID(position, consigne);

          // Ajuster la trajectoire du robot en fonction de l'ajustement
          // Par exemple, ajuster la vitesse des moteurs
          suivreLigne(adjustment);

      }
      else{
            static bool turningBack = false;
            static bool doneTurn = false;

            if (!turningBack) {
              turningBack = true;
              doneTurn = false;
            }

            if (turningBack && !doneTurn) {
              doneTurn = faireTourComplet(180); // tourne de 180°
            }

            if (doneTurn) {
              turningBack = false;
              doneTurn = false;

            }
      }


}

void ledAction(int r, int g, int b) {
  led.setColor(r, g, b);
  led.show();
}
void clearLeds() {
  int anneau = 0;
  for (int i = anneau; i < LEDNUM; ++i) led.setColor(i, 0, 0, 0);
  led.show();
}

void calibrationAutomatique() {
  int turningAngle = 360;
  if (!faireTourComplet(turningAngle)) {
    calibrer();
  } else {
    firstCalibration = false;
  }
}

void demarrage(unsigned long ct) {
  int rateDecount = 1000;
  static unsigned long ptDecount = 0;
  static int indexDecount = 3;

  if (ct - ptDecount >= rateDecount) {
    if (indexDecount == 1) {
      Serial.println(indexDecount);
      clearLeds();
      ledAction(255, 255, 0);
    } else if (indexDecount == 0) {
      Serial.println("Go!");
      clearLeds();
      ledAction(0, 255, 0);
    } else if (indexDecount == -1) {
      chrono = true;
      ctAuto = SETUP;
      currentState = AUTO;
      clearLeds();
    } else {
      Serial.println(indexDecount);
      ledAction(255, 0, 0);
    }

    indexDecount--;
    ptDecount = ct;
  }
}

void jsonTask() {
}
int startChrono() {
  unsigned long ct = millis();
  static unsigned long pt = 0;
  int rate = 1000;
  static int count = 0;
  if (ct - pt >= rate) {
    count++;
    pt = ct;
  }
  return count;
}
void avanceCommand(short speed = speedMin) {
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  const double kp = 2.0;
  const double kd = 0.2;

  if (firstRun) {
    gyro.resetData();
    zAngleGoal = gyro.getAngleZ();
    previousError = 0;
    firstRun = false;
    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;
  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);

  previousError = error;
  Serial.println("J'AVANCE");
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed + output);
}
void Stop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  firstRun = true;
  // analogWrite(BUZZER_PIN, LOW);
}
// void avanceAuto(int vitesse) {
//   clearLeds();
//   Serial.println("AVANCE COMMAND");

//   float consigne = 0.0f;
//     //logique prochaine note
//     //si 012 cest true pour angle gauche , 234 pour angle de droite pis 01234 pour le t qui sera en premier dans le if , cest pour sa que ya un on line chaque capteur et non un global
//     if (trackLine()) {
//       normaliserValeurs();

//       // Calculer la position de la ligne
//       float position = calculerPositionLigne();

//       // Calculer l'ajustement à apporter à la trajectoire
//       float adjustment = computePID(position, consigne);

//       // Ajuster la trajectoire du robot en fonction de l'ajustement
//       // Par exemple, ajuster la vitesse des moteurs
//       suivreLigne(adjustment);

//     } else {
//       avanceCommand(150);
//     }
//   // avanceCommand(vitesse);

//   // calibrer();
//   // normaliserValeurs();

//   //       normaliserValeurs();

//   //     // Calculer la position de la ligne
//   //     float position = calculerPositionLigne();

//   //     // Calculer l'ajustement à apporter à la trajectoire
//   //     float adjustment = computePID(position, consigne);

//   //     // Ajuster la trajectoire du robot en fonction de l'ajustement
//   //     // Par exemple, ajuster la vitesse des moteurs
//   //     suivreLigne(adjustment);

//   // if (trackLine()) {
//   //   Serial.println("TRICKLIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIINE");
//   //   delay(5000);
//   //   Stop();
//   //   clearLeds();
//   //   ctAuto = LINETRACK;
//   // }
// }

void avanceAuto(int vitesse) {
  clearLeds();
  static bool firstEnter = true;
  if (firstEnter) {
    if (trackLine()) {
      avanceCommand(200);
    } else {
      Serial.println("Sortie Avance");
      firstEnter = false;
      return;
    }

  } else {
    if (trackLine()) {
      Stop();
      firstPivot = true;
      ctAuto = CHECKPOINTA;
    } else {
      avanceCommand(150);
    }
  }
}


void stateAutoManager(unsigned long ct) {
  switch (ctAuto) {
    case SETUP:
      if (firstCalibration) {
        calibrationAutomatique();
        sensorTask();
        normaliserValeurs();
      } else {
        firstCalibration = false;
        ctAuto = AVANCE;
      }
      break;
    case AVANCE:
      Serial.println("AVANCE");
      firstTime = true;
      avanceAuto(speedMin);
      break;
    case CHECKPOINTA:
      if (firstTime) {
        Serial.println("First Time Checkpoint");
        Serial.println(firstPivot);
        //firstRun = true;
        if(!faireTourComplet(90)){
          Serial.println("Pivot 90° terminé");
          firstTime = false;
        }       
      }
      //sensorTask();
      Serial.println("Mission");
      mission01();
      if (detectObjet(50)) {
        Serial.println("Detect object");
        Stop();
        ctAuto = CHECKPOINTB;
      }
      
      break;
    case CHECKPOINTB:
      Stop();
      break;
  }
}
void stateManager(unsigned long ct) {
  switch (currentState) {
    case DEMARRAGE:
      demarrage(ct);
      break;
    case AUTO:
      Serial.println("Mode auto");
      sensorTask();
      stateAutoManager(ct);
      break;
  }
}

void loop() {
  unsigned long currentTime = millis();
  gyro.update();
  static unsigned long pt = 0;
  int rate = 1000;
  if (chrono) {
    chronometre = startChrono();
  }
  if (currentTime - pt >= rate) {
    Serial.print("Time : ");
    Serial.print(chronometre);
    Serial.println("s");
    pt = currentTime;
  }
  stateManager(currentTime);


  // gyro.update();
  // //distance = distanceTask(currentTime);
  // //changeSpeed(currentTime);
  // sensorTask();
  // mission01();


  // for (int i = 0; i < NB_IR; i++) {
  //   Serial.print("IR");
  //   Serial.print(i);
  //   Serial.print(":");
  //   Serial.print(" min  ");
  //   Serial.print(capteurs[i].valMin);
  //   Serial.print(" max  ");
  //   Serial.println(capteurs[i].valMax);
  //   Serial.print(" norm  ");
  //   Serial.print(capteurs[i].valeurNormalisee);
  //   Serial.print(" Lue  ");
  //   Serial.println(capteurs[i].valeurLue);

  //   Serial.print("IR");
  //   Serial.print(i);
  //   Serial.print(":");
  //   Serial.print(capteurs[i].valeurLue);
  //   Serial.print("\t");
  // }
  // sensorTask();
  // detectObjet();
  // mission01();
}