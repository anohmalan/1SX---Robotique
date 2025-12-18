#include <Adafruit_seesaw.h>
#include <MeAuriga.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define NB_IR 5
#define LEDPIN 44
#define LEDNUM 12

float norm = 1000.0;

MeGyro gyro(0, 0x69);
MeRGBLed led(PORT0, 12);

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

int sensorValues[NB_IR];

void setup() {
  unsigned long currentTime = millis();
  Serial.begin(115200);
  led.setpin(LEDPIN);

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1)
      ;
  }
  gyro.begin();

  Serial.println("Connexion réussie au LyneTracker!");

  clearLeds();
  delay(1000);
}

#pragma region updates

void gyroTask(unsigned long ct) {
  gyro.update();
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

// void suivreLigne(float adjustement) {

//   encoderLeft.setMotorPwm(rangerPwmL - adjustement);
//   encoderRight.setMotorPwm(-rangerPwmL - adjustement);
// }

// float getCurrentPos() {
//   float leftPulsPos = encoderLeft.getPulsePos();
//   float rightPulsPos = encoderRight.getPulsePos();

//   float leftDistanceCM = (leftPulsPos / (PULSE * RATIO)) * (DIAMETER * pi);
//   float rightDistanceCM = (rightPulsPos / (PULSE * RATIO)) * (DIAMETER * pi);

//   return (leftDistanceCM + rightDistanceCM) / 2;
// }

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


int convertirCodeCapteur(Capteur* capteurs) {
    int code = 0;

    for (int i = 0; i < NB_IR; i++) {
        code <<= 1;                 // décale le code vers la gauche
        if (!capteurs[i].surLigne)  // capteur VOIT du NOIR
            code |= 1;              // met le bit à 1
    }
    return code;
}
void suivreLigne(int code) {
  switch (code) {

    case 0b00100:   // centré
      // avancer();
      break;

    case 0b01100:
    case 0b01000:   // ligne à gauche
      // tournerGauche();
      break;

    case 0b00110:
    case 0b00010:   // ligne à droite
      // tournerDroite();
      break;

    case 0b00000:   // ligne perdue
      // stopMoteurs();
      break;

    case 0b11111:   // intersection
      // gererIntersection();
      break;
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
void clearLeds() {
  int anneau = 0;
  for (int i = anneau; i < LEDNUM; ++i) led.setColor(i, 0, 0, 0);
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


void loop() {
  unsigned long currentTime = millis();
  gyro.update();

  int codeIR = convertirCodeCapteur(capteurs);

  suivreLigne(codeIR);

  // static unsigned long pt = 0;
  // int rate = 1000;
  // if (chrono) {
  //   chronometre = elapsedChrono();
  // }
  // if (currentTime - pt >= rate) {
  //   chaqueSeconde = true;
  //   jsonTask();
  //   pt = currentTime;
  // }

  // encoderLoop();

  // stateManager(currentTime);
}