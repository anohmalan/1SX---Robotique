//Author : Rodrigue Brim

#include <Adafruit_seesaw.h>
#include <MeAuriga.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define NB_IR 5
#define PULSE 9
#define LEDPIN 44
#define LEDNUM 12
#define RATIO 39.267
#define BUZZER_PIN 45
#define DIST_WHEEL 155

// int pwm = 80;
int speed = 120;
float pi = 3.1416;
int speedMin = 100;
int speedMax = 250;
int pivotPwm = 70;
// short vitessePivot = 100;
int rangerPwm = 120;
int rangerPwmL = 120;
float DIAMETER = 6.5;
float DISTANCE_XX = 30.0;
unsigned long currentTime;

float norm = 1000.0;
int turningAngle = 0;

bool firstRun = true;
bool firstPivot = true;
bool firstTime = true;
bool firstCalibration = true;
static bool firstCheckC = true;
static bool firstCheckC1 = true;
static bool restCheckC = false;
static bool firstAvance = true;

int frequence = 1000;

int distChPA = 50;
int distChPB = 20;
int distChPE = 10;
int distAmbT = 30;
int distCheckA = 50;

bool chaqueSeconde = false;
bool chaqueCp = false;

String lastCommand;

MeBuzzer buzzer;
MeGyro gyro(0, 0x69);
MeRGBLed led(PORT0, 12);
MeUltrasonicSensor ultra(PORT_10);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

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
                 CHECKPOINTA,
                 CHECKPOINTB,
                 CHECKPOINTC1,
                 CHECKPOINTC2,
                 CHECKPOINTE,
                 CHECKPOINTF,
                 LINETRACK };
StateAuto ctAuto;

bool chrono = false;
long int chronometre = 0;
unsigned long chronoStart = 0;

void setup() {
  unsigned long currentTime = millis();
  Serial.begin(115200);
  led.setpin(LEDPIN);
  buzzer.setpin(BUZZER_PIN);
  pinMode(BUZZER_PIN, OUTPUT);

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1)
      ;
  }
  encoderConfig();
  gyro.begin();

  Serial.println("Connexion réussie au LyneTracker!");

  currentState = MAN;
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

void encoderLoop() {
  encoderLeft.loop();
  encoderRight.loop();
}

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

#pragma region CHECK_POINT_STATE
enum EtatAuto {
  AUTO_FIRST_TIME,
  AUTO_RUN
};
EtatAuto etatAuto = AUTO_FIRST_TIME;

enum EtatCheckpointB {
  B_AVANCE,
  B_TOURNE,
  B_SUIVRE_LIGNE
};
EtatCheckpointB etatB = B_AVANCE;

enum EtatDecision {
  D_TOURNE_DROITE,
  D_CHECK_DROITE,
  D_TOURNE_GAUCHE,
  D_CHECK_GAUCHE,
  D_RETOUR_CENTRE
};
EtatDecision etatDecision = D_TOURNE_DROITE;

enum EtatMission02 {
  M_SUIVI_LIGNE,
  M_INIT,
  M_TOURNE_GAUCHE
};
EtatMission02 etatMission02 = M_SUIVI_LIGNE;
#pragma endregion

#pragma region COMMANDES

void avanceCommand(short speed = speedMax) {
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

  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed + output);
}

bool faireAvance(float cm) {

  static long startPulses = 0;
  float ctm = 20.26;

  float cm_per_pulse = ctm / PULSE;

  if (firstAvance) {
    firstAvance = false;
    gyro.resetData();
    encoderLeft.setPulsePos(0);  // IMPORTANT !
    encoderRight.setPulsePos(0);
    startPulses = 0;
  }

  long pulses = encoderLeft.getPulsePos();

  double kp = 7;
  double kd = 1;
  static double previousError = 0;

  double error = gyro.getAngleZ();
  double output = kp * error + kd * (error - previousError);
  previousError = error;

  // Commande moteurs
  int v = speedMin;

  int leftPWM = v - output;
  int rightPWM = -(v + output);  // inversion moteur droit

  encoderLeft.setMotorPwm(leftPWM);
  encoderRight.setMotorPwm(rightPWM);

  // Condition d'arrêt
  if (encoderLeft.getPulsePos() >= cm) {
    firstAvance = true;
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    return true;
  }

  return false;
}

void avanceAutoCommand(int vitesse) {

  int distance = ultra.distanceCm();
  clearLeds();

  avanceCommand(vitesse);

  if (distance <= DISTANCE_XX) {
    Stop();
    clearLeds();
    currentState = MAN;
  }
}

void pivotCommand(bool droite) {

  if (droite) {
    encoderLeft.setMotorPwm(pivotPwm);
    encoderRight.setMotorPwm(pivotPwm);
  } else {
    encoderLeft.setMotorPwm(-pivotPwm);
    encoderRight.setMotorPwm(-pivotPwm);
  }
}

void Stop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  firstRun = true;
  analogWrite(BUZZER_PIN, LOW);
}

void clearLeds() {
  int anneau = 0;
  for (int i = anneau; i < LEDNUM; ++i) led.setColor(i, 0, 0, 0);
  led.show();
}

void ledAction(int r, int g, int b) {
  led.setColor(r, g, b);
  led.show();
}

void ledAction(int idx, int r, int g, int b) {
  int anneau = 0;
  if (idx == anneau) {
    led.setColor(r, g, b);
  } else {
    led.setColor(idx, r, g, b);
  }

  led.show();
}

void klaxonCommad() {
  analogWrite(BUZZER_PIN, frequence);
}

void beepCommand() {
  static bool on = true;
  static long int dernierBip = 0;
  const int delai = 150;

  unsigned long tempsActuel = millis();

  if (tempsActuel - dernierBip >= delai) {
    dernierBip = tempsActuel;
    on = !on;
  }

  if (on) {
    analogWrite(BUZZER_PIN, frequence);
  } else {
    analogWrite(BUZZER_PIN, LOW);
  }
}

void commandLight(String params) {
  int commaCount = countCharOccurrences(params, ',');

  // Vérifie le nombre de paramètres en comptant les virgules
  if (commaCount == 2) {
    // Trois paramètres (r, g, b) pour définir toute la couleur de l'anneau
    int r = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int g = params.substring(0, params.indexOf(',')).toInt();
    int b = params.substring(params.indexOf(',') + 1).toInt();

    ledAction(r, g, b);  // Appel pour affecter l'ensemble de l'anneau
  } else if (commaCount == 3) {
    // Quatre paramètres (idx, r, g, b) pour définir une LED spécifique
    int idx = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int r = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int g = params.substring(0, params.indexOf(',')).toInt();
    int b = params.substring(params.indexOf(',') + 1).toInt();

    ledAction(idx, r, g, b);  // Appel pour affecter une LED spécifique
  } else {
  }
}

#pragma endregion

#pragma region FONCTION_GLOBAL
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

  if (diff > halfCircle) diff -= tourCicle;
  if (diff < -halfCircle) diff += tourCicle;

  total += diff;
  lastWrapped = wrapped;

  return total;
}

bool faireTourComplet(int targetAngle) {

  int theSeuil = 3;
  int direction = 1;
  double coef2 = 0.2;
  double coef1 = 0.4;
  double coef3 = 0.15;
  short tolerance = 10;
  static double zAngleGoal = 0.0;

  if (firstPivot) {
    gyro.resetData();
    getUnwrappedAngle(true);  // reset interne
    zAngleGoal = getUnwrappedAngle() + targetAngle;
    firstPivot = false;
    return false;
  }

  double current = getUnwrappedAngle();
  double remaining = zAngleGoal - current;

  if (fabs(remaining) <= tolerance) {
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    firstPivot = true;
    return true;
  }

  // Ajuste dynamiquement la vitesse pour la précision
  double absRem = fabs(remaining);
  int pwm;

  if (absRem > distAmbT) pwm = speed;
  else if (absRem > distChPE) pwm = speed * coef1;
  else if (absRem > theSeuil) pwm = speed * coef2;
  else pwm = speed * coef3;

  pwm *= (remaining > 0 ? direction : -direction);

  encoderLeft.setMotorPwm(pwm);
  encoderRight.setMotorPwm(pwm);

  return false;
}

bool detectObjet(int dist) {
  int distance = ultra.distanceCm();

  if (distance <= dist) {
    return true;
  } else {
    return false;
  }
}

void colis(int r, int g, int b) {
  int zero = 0;
  int ledArretStart = 6;
  int ledArretEnd = 11;
  int ledsToTurnOff[] = { 12, 11, 10, 9, 8, 7 };
  int nbOff = sizeof(ledsToTurnOff) / sizeof(ledsToTurnOff[0]);

  for (int i = zero; i < nbOff; i++) {
    led.setColor(ledsToTurnOff[i], 0, 0, 0);
  }

  for (int i = ledArretStart; i <= ledArretEnd; i++) {
    led.setColor(i, r, g, b);
  }

  led.show();
}

void demarrage(unsigned long ct) {
  int rateDecount = 1000;
  static unsigned long ptDecount = 0;
  static unsigned int indexDecount = 3;

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
      startChrono();
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

void avanceAuto(int vitesse) {
  clearLeds();
  static bool firstEnter = true;
  if (firstEnter) {
    if (trackLine()) {
      avanceCommand(speedMax);
    } else {
      firstEnter = false;
      return;
    }

  } else {
    if (trackLine()) {
      Stop();
      ledAction(6, 50, 0, 50);
      ctAuto = CHECKPOINTA;
    } else {
      avanceCommand(speed);
    }
  }
}

void ledLoopTask() {
  static float j;
  static float f;
  static float k;

  for (uint8_t t = 0; t < LEDNUM; t++) {
    uint8_t red = 8 * (1 + sin(t / 2.0 + j / 4.0));
    uint8_t green = 8 * (1 + sin(t / 1.0 + f / 9.0 + 2.1));
    uint8_t blue = 8 * (1 + sin(t / 3.0 + k / 14.0 + 4.2));
    led.setColorAt(t, red, green, blue);
  }
  led.show();

  j += random(1, 6) / 6.0;
  f += random(1, 6) / 6.0;
  k += random(1, 6) / 6.0;
}
#pragma endregion

#pragma region SENSOR
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

float calculerPositionLigne() {
  float numerateur = 0;
  float denominateur = 0;

  for (int i = 0; i < NB_IR; i++) {
    numerateur += capteurs[i].valeurNormalisee * (i - 2);
    denominateur += capteurs[i].valeurNormalisee;
  }
  return numerateur / denominateur * norm;
}

float capteurLectureNormalisee(int index) {
  return ((capteurs[index].valeurLue - capteurs[index].valMin) * 1.0) / (capteurs[index].valMax - capteurs[index].valMin) * norm;
}

void normaliserValeurs() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].valeurNormalisee = capteurLectureNormalisee(i);
  }
}

void suivreLigne(float adjustement) {

  encoderLeft.setMotorPwm(rangerPwmL - adjustement);
  encoderRight.setMotorPwm(-rangerPwmL - adjustement);
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

bool trackLeft() {
  for (int i = 0; i < 3; i++) {
    if (capteurs[i].valeurLue >= capteurs[i].seuil) {
      return false;  // Si au moins un capteur ne détecte pas la ligne, ce n'est pas une intersection
    }
  }
  return true;
}
#pragma endregion

#pragma region PID_& _CALIBRATION
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

void calibrationAutomatique() {
  turningAngle = 360;
  if (!faireTourComplet(turningAngle)) {
    calibrer();
  } else {
    firstCalibration = false;
  }
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
#pragma endregion

#pragma region PRINT_TASK
void jsonTask() {
  StaticJsonDocument<256> doc;

  if (chaqueSeconde) {
    doc["ts"] = millis();
    doc["chrono"] = chronometre;
    doc["etat"] = currentState;
    doc["gz"] = gyro.getAngleZ();
    JsonObject pwm = doc.createNestedObject("pwm");
    pwm["l"] = encoderLeft.getCurPwm();
    pwm["r"] = encoderRight.getCurPwm();
    JsonArray capt = doc.createNestedArray("capt");
    for (int i = 0; i < NB_IR; i++) {
      capt.add(sensorValues[i]);
    }
    chaqueSeconde = false;
  }

  if (chaqueCp) {
    doc["ts"] = millis();
    doc["chrono"] = chronometre;
    doc["etat"] = currentState;
    doc["cp"] = ctAuto;
    chaqueCp = false;
  }

  serializeJson(doc, Serial);
  Serial.println();
}
unsigned long elapsedChrono() {
  return millis() - chronoStart;
}
void startChrono() {
  chronoStart = millis();
}
#pragma endregion

#pragma region FONCTION_CHECK_POINT
void mission02() {

  float consigne = 0.0f;

  normaliserValeurs();

  float position = calculerPositionLigne();
  float adjustment = computePID(position, consigne);

  switch (etatMission02) {
    case M_INIT:
      speed = 150;
      speedMin = 100;
      firstAvance = true;
      offMotors();
      faireAvance(100);
      etatMission02 = M_TOURNE_GAUCHE;
      break;
    case M_TOURNE_GAUCHE:
      turningAngle = -90;
      if (!faireTourComplet(turningAngle)) {
        //Tourne à droite
        return;
      } else {
        firstPivot = true;
        etatMission02 = M_SUIVI_LIGNE;
      }
      return;
      break;

    case M_SUIVI_LIGNE:
      suivreLigne(adjustment);

      if (trackLeft()) {
        etatMission02 = M_INIT;
      }
      break;
  }
}
enum CheckpointCState {
  CPC_FIRST_CHECK,
  CPC_ADVANCE,
  CPC_DECISION
};

static CheckpointCState cpcState = CPC_FIRST_CHECK;
void takeDecision() {

  int distance = 0;

  switch (etatDecision) {

    //Tourner à droite
    case D_TOURNE_DROITE:
      speed = 110;
      turningAngle = 90;

      if (faireTourComplet(turningAngle)) {
        firstPivot = true;
        etatDecision = D_CHECK_DROITE;
      }
      break;

    //Vérifier distance à droite
    case D_CHECK_DROITE:
      distance = ultra.distanceCm();

      if (distance > distChPA) {
        firstRun = true;
        firstCheckC1 = true;
        restCheckC = false;
        chaqueCp = true;
        jsonTask();
        ledAction(3, 50, 0, 50);
        etatDecision = D_TOURNE_DROITE;
        ctAuto = CHECKPOINTC2;
      } else {
        etatDecision = D_TOURNE_GAUCHE;
      }
      break;

    //Tourner à gauche
    case D_TOURNE_GAUCHE:
      speed = 110;
      turningAngle = -180;

      if (faireTourComplet(turningAngle)) {
        firstPivot = true;
        etatDecision = D_CHECK_GAUCHE;
      }
      break;

    //Vérifier distance à gauche
    case D_CHECK_GAUCHE:
      distance = ultra.distanceCm();

      if (distance > distChPA) {
        firstRun = true;
        firstCheckC1 = true;
        restCheckC = false;
        chaqueCp = true;
        jsonTask();
        ledAction(3, 50, 0, 50);
        etatDecision = D_TOURNE_DROITE;
        ctAuto = CHECKPOINTC2;
      } else {
        etatDecision = D_RETOUR_CENTRE;
      }
      break;

    //Retour au centre
    case D_RETOUR_CENTRE:
      speed = 110;
      turningAngle = -90;

      if (faireTourComplet(turningAngle)) {
        firstPivot = true;
        chaqueCp = true;
        jsonTask();
        ledAction(2, 50, 0, 50);
        etatDecision = D_TOURNE_DROITE;
        ctAuto = CHECKPOINTE;
      }
      break;
      // rangerPwm = 80;
      cpcState = CPC_DECISION;
  }
}
void ctAutoSetup() {
  if (firstCalibration) {
    calibrationAutomatique();
    sensorTask();
    normaliserValeurs();
  } else {
    firstCalibration = false;
    ctAuto = AVANCE;
  }
}
void ctAutoAvance() {

  int distance = ultra.distanceCm();
  firstTime = true;

  int vitesse = 130;

  clearLeds();

  if (distance < distChPA) {
    vitesse = 110;
  }

  if (trackLine() && distance < distAmbT) {
    firstTime = true;
    chaqueCp = true;
    jsonTask();
    ctAuto = CHECKPOINTA;
    return;
  }

  avanceCommand(vitesse);
}
#pragma endregion

#pragma region CHECK_POINT
void ctAutoCheckpointA() {
  switch (etatAuto) {

    case AUTO_FIRST_TIME:
      static bool tourne = true;

      if (!faireTourComplet(90)) {
        return;
      }

      etatAuto = AUTO_RUN;
      break;

    case AUTO_RUN:
      mission02();
      break;
  }

  // Partie commune (reste en dehors du switch)
  if (detectObjet(distChPA)) {
    firstRun = true;
    chaqueCp = true;
    jsonTask();
    ledAction(5, 50, 0, 50);
    ctAuto = CHECKPOINTB;
  }
}

void ctAutoCheckpointB() {

  const int vit = 100;
  const int cmPlus = 100;
  switch (etatB) {

    case B_AVANCE:
      avanceCommand(cmPlus);

      if (detectObjet(distChPB)) {
        Stop();
        etatB = B_TOURNE;
      }
      break;

    case B_TOURNE:
      turningAngle = -180;

      if (faireTourComplet(turningAngle)) {
        firstRun = true;
        etatB = B_SUIVRE_LIGNE;
      }
      break;

    case B_SUIVRE_LIGNE:
      if (trackLine()) {
        Stop();
        chaqueCp = true;
        jsonTask();
        ledAction(4, 50, 0, 50);
        ctAuto = CHECKPOINTC1;
      } else {
        avanceCommand(vit);
      }
      break;
  }
}

void ctAutoCheckpointC() {
  int distance = ultra.distanceCm();
  int leftDist = 0;
  int rightDist = 0;
  if (distance < distAmbT) {
    rangerPwm = 100;
  }

  if (firstCheckC1) {
    if (isAtIntersection()) {
      firstCheckC1 = false;
      restCheckC = true;
      return;
    } else {
      mission02();
    }
  }
  if (restCheckC) {
    static bool firstTake = true;
    static unsigned long pt = 0;
    int rate = 1000;
    if (firstTake) {
      if (faireAvance(70)) {
        Stop();
        firstTake = false;
      }
    }
    if (!firstTake) {
      Stop();
      takeDecision();
    }
  }
}


#pragma endregion

#pragma region STATE_MANAGER
void stateAutoManager(unsigned long ct) {
  switch (ctAuto) {
    case SETUP:
      ctAutoSetup();
      break;
    case AVANCE:
      ctAutoAvance();
      break;
    case CHECKPOINTA:
      ctAutoCheckpointA();
      break;
    case CHECKPOINTB:
      colis(255, 255, 0);
      ctAutoCheckpointB();
      break;
    case CHECKPOINTC1:
      ctAutoCheckpointC();
      break;
    case CHECKPOINTC2:
      ctAutoCheckpointC();
      break;
    case CHECKPOINTE:
      colis(0, 255, 0);
      if (faireAvance(-150)) {
        Stop();
        chaqueCp = true;
        jsonTask();

        currentState = MAN;
      }
      break;
    case CHECKPOINTF:
      ledAction(1, 50, 0, 50);
      avanceCommand(130);
      int distance = ultra.distanceCm();
      if (distance <= 10) {
        Stop();
        currentState = STOP;
        return;
      }
      break;
  }
}

void stateManager(unsigned long ct) {
  switch (currentState) {
    case DEMARRAGE:
      demarrage(ct);
      break;
    case AUTO:
      sensorTask();
      stateAutoManager(ct);
      break;
    case MAN:
      break;
    case STOP:
      ledLoopTask();
      chrono = false;
      break;
  }
}
#pragma endregion

void loop() {
  unsigned long currentTime = millis();
  gyro.update();
  static unsigned long pt = 0;
  int rate = 1000;
  if (chrono) {
    chronometre = elapsedChrono();
  }
  if (currentTime - pt >= rate) {
    chaqueSeconde = true;
    jsonTask();
    pt = currentTime;
  }

  encoderLoop();

  stateManager(currentTime);
}

#pragma region RECEIVED_COMMANDE

// Événement qui se déclenche lorsqu'il y a réception de données via le port série
void serialEvent() {
  static String receivedData = "";

  if (!Serial.available()) return;

  receivedData = Serial.readStringUntil('\n');
  parseData(receivedData);
}

/**
  Fonction servant à analyser les données reçues.
  "parse" veut dire analyser
*/
void parseData(String& receivedData) {
  bool isFromBLE = false;  // Indicateur de source des données

  if (receivedData.length() >= 2) {
    // Vérifier si les deux premiers octets sont 0xFF55 (BLE)
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55) {
      isFromBLE = true;
      // Supprimer les deux premiers octets
      receivedData.remove(0, 2);
    }
    // Vérifier si les deux premiers caractères sont "!!" (Moniteur Série)
    else if (receivedData.startsWith("!!")) {
      // Supprimer les deux premiers caractères
      receivedData.remove(0, 2);
    } else {
      // En-tête non reconnue
      Serial.print(F("Données non reconnues : "));
      Serial.println(receivedData);
      return;
    }
  } else {
    Serial.print(F("Données trop courtes : "));
    Serial.println(receivedData);
    return;
  }

  // Découpage de la commande et des paramètres
  int firstComma = receivedData.indexOf(',');

  if (firstComma == -1) {
    // Pas de virgule, donc c'est une commande sans paramètres
    handleCommand(receivedData);
    lastCommand = receivedData;
  } else {
    // Il y a des paramètres
    String command = receivedData.substring(0, firstComma);
    String params = receivedData.substring(firstComma + 1);

    lastCommand = command;
    handleCommandWithParams(command, params);
  }
}

#pragma endregion

#pragma region COMMANDES
// Fonction pour gérer une commande sans paramètres
void handleCommand(String command) {
  bool mouvement = false;
  bool droite = false;
  unsigned long dernierBip = 0;

  if (currentState == AUTO) {
    return;
  }

  // Utilisation d'un switch pour les commandes sans paramètres
  char cmd = command[0];
  switch (cmd) {
    case 'F':  // Commande "Avance"
      mouvement = true;
      if (mouvement) {
        avanceCommand(speedMax);
      }
      break;
    case 'B':  // Commande "Recule"
      mouvement = true;
      if (mouvement) {
        avanceCommand(-speedMax);
      }
      break;
    case 'R':  // Commande "Pivot Droit"
      mouvement = true;
      droite = true;
      if (mouvement) {
        pivotCommand(droite);
      }
      break;
    case 'L':  // Commande "Pivot Droit"
      mouvement = true;
      droite = false;
      if (mouvement) {
        pivotCommand(droite);
      }
      break;
    case 'S':  // Commande "BEEP"
      mouvement = false;
      Stop();
      break;

    case 'C':  // Commande pour Calibration
      if (!faireTourComplet(360)) {
        return;
      }
      break;
    case 'H':
      currentState = AUTO;
      ctAuto = CHECKPOINTF;
      break;

    default:
      break;
  }
}

// Fonction pour gérer une commande avec paramètres
void handleCommandWithParams(String command, String params) {

  if (command == "AUTO") {
    currentState = AUTO;
    return;
  } else if (command == "MAN") {
    currentState = MAN;
    Stop();
    return;
  } else if (command == "GO") {
    currentState = DEMARRAGE;
    ctAuto = SETUP;
  } else if (command == "STOP") {
    currentState = STOP;
  }
  if (currentState == AUTO) {
    return;
  }
  char cmd = command[0];
  switch (cmd) {
    case 'p':  // Commande "p"
      if (params.toInt() >= speedMin && params.toInt() <= speedMax) {
        speed = params.toInt();
      } else {
      }
      break;
    case 'l':  // Commande "LIGHT" pour définir la couleur de l'anneau LED
      commandLight(params);
      break;
    default:
      break;
  }
}

#pragma endregion

#pragma region HELPERS
int countCharOccurrences(const String& str, char ch) {
  int zero = 0;
  int count = 0;
  for (int i = zero; i < str.length(); i++) {
    if (str[i] == ch) {
      count++;
    }
  }
  return count;
}
#pragma endregion