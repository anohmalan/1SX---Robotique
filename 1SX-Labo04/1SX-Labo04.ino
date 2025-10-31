//Author : Rodrigue Brim

#include <MeAuriga.h>

#define BUZZER_PIN 45

#define LEDPIN 44
#define LEDNUM 12

#define PULSE 9
#define RATIO 39.267

MeBuzzer buzzer;

MeGyro gyro(0, 0x69);
MeRGBLed led(PORT0, 12);
MeUltrasonicSensor ultra(PORT_10);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

enum State { MAN,
             AUTO };
State currentState;

unsigned long currentTime = 0;
bool debugMode = false;

int speed = 150;
int speedMin = 50;
int speedMax = 255;
int pivotPwm = 100;
bool firstRun = true;

float DISTANCE_XX = 0.0;

String lastCommand;

bool blinkState = false;
unsigned long lastBlinkTime = 0;

int frequence = 1000;


void setup() {
  Serial.begin(115200);
  buzzer.setpin(BUZZER_PIN);

  led.setpin(LEDPIN);
  pinMode(BUZZER_PIN, OUTPUT);

  encoderConfig();
  gyro.begin();

  clearLeds();
  currentState = MAN;
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

#pragma endregion

#pragma region COMMANDES

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

  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed + output);
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
  autoMOdLed();
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

void clignotantGauche() {
  int rate = 50;
  int anneau = 0;
  int tabLedS[] = { 12, 1, 2 };
  unsigned long currentTime = millis();
  int nbLeds = sizeof(tabLedS) / sizeof(tabLedS[0]);

  if (currentTime - lastBlinkTime > rate) {
    blinkState = !blinkState;
    lastBlinkTime = currentTime;

    if (blinkState) {
      for (int i = anneau; i < nbLeds; i++) { led.setColor(tabLedS[i], 255, 165, 0); }
      led.show();
    } else {
      clearLeds();
    }
  } else {
    clearLeds();
  }
}

void clignotantDroit() {
  int rate = 50;
  int anneau = 0;
  int tabLedS[] = { 4, 5, 6 };
  unsigned long currentTime = millis();
  int nbLeds = sizeof(tabLedS) / sizeof(tabLedS[0]);

  if (currentTime - lastBlinkTime > rate) {
    blinkState = !blinkState;
    lastBlinkTime = currentTime;

    if (blinkState) {
      for (int i = anneau; i < nbLeds; i++) { led.setColor(tabLedS[i], 255, 165, 0); }
    } else {
      clearLeds();
    }
    led.show();
  }
}

void arret() {
  int zero = 0;
  int ledArretStart = 7;
  int ledArretEnd = 11;
  int ledsToTurnOff[] = { 12, 1, 2, 3, 4, 5, 6 };
  int nbOff = sizeof(ledsToTurnOff) / sizeof(ledsToTurnOff[0]);

  for (int i = zero; i < nbOff; i++) {
    led.setColor(ledsToTurnOff[i], 0, 0, 0);
  }

  for (int i = ledArretStart; i <= ledArretEnd; i++) {
    led.setColor(i, 255, 0, 0);
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

void debugMod() {
  static unsigned long lastDebugTime = 0;
  static String lastCommandSent = "";
  const unsigned long interval = 200;  // 200 ms

  unsigned long now = millis();

  if (!debugMode) return;  // Ne rien faire si le mode débogage est désactivé

  if (now - lastDebugTime >= interval) {
    lastDebugTime = now;

    int distance = ultra.distanceCm();

    if (lastCommand != "" && lastCommand != lastCommandSent) {
      lastCommandSent = lastCommand;  // On garde une trace
    }

    if (currentState != AUTO) {
      Serial.print(F("[DEBUG] Mode: MANUEL | Vitesse actuelle: "));
      Serial.println(speed);
    } else if (currentState == AUTO) {
      Serial.print(F("[DEBUG] Distance: "));
      Serial.print(distance);
      Serial.println(F(" cm"));
      Serial.print(F("[DEBUG] Dernière commande: "));
      Serial.println(lastCommand);
    }
  }
}

void autoMOdLed() {
  unsigned long currentTime = millis();
  int rate = 100;

  if (currentTime - lastBlinkTime > rate) {
    blinkState = !blinkState;
    lastBlinkTime = currentTime;

    if (blinkState) {
      led.setColor(0, 255, 165, 0);
    }
    led.show();
  } else {
    clearLeds();
  }
}


#pragma endregion

void loop() {
  currentTime = millis();
    gyro.update();
  encoderLoop();
  stateManager(currentTime);
  debugMod();
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
      clearLeds();
      mouvement = true;
      if (mouvement) {
        avanceCommand(speed);
      }
      break;
    case 'B':  // Commande "Recule"
      clearLeds();
      mouvement = true;
      if (mouvement) {
        avanceCommand(-speed);
        beepCommand();
      }
      break;
    case 'R':  // Commande "Pivot Droit"
      mouvement = true;
      droite = true;
      if (mouvement) {
        clignotantDroit();
        pivotCommand(droite);
      }
      break;
    case 'L':  // Commande "Pivot Droit"
      mouvement = true;
      droite = false;
      if (mouvement) {
        clignotantGauche();
        pivotCommand(droite);
      }
      break;
    case 'S':  // Commande "BEEP"
      mouvement = false;
      arret();
      Stop();
      break;
    case 'K':  // Commande "BEEP"
      klaxonCommad();
      break;

    case 'd':  // Commande pour basculer le mode débogage
      debugMode = !debugMode;
      Serial.print(F("Mode débogage : "));
      Serial.println(debugMode ? F("activé") : F("désactivé"));
      break;

    default:
      break;
  }
}

// Fonction pour gérer une commande avec paramètres
void handleCommandWithParams(String command, String params) {

  if (command == "AUTO") {
    currentState = AUTO;
    DISTANCE_XX = params.toFloat();
    return;
  } else if (command == "MAN") {
    currentState = MAN;
    Stop();
    return;
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

void autoMode() {
  avanceAutoCommand(speed);
}

void stateManager(unsigned long ct) {
  switch (currentState) {
    case MAN:
      break;
    case AUTO:
      autoMode();
      break;
  }
}


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