#include <Adafruit_seesaw.h>
#include <Wire.h>

#define NB_IR 5

struct Capteur {
  int valeurLue;
  int valeurNormalisee;
  int valMin = 1023;
  int valMax = 0;
  int seuil;
  bool onLine = false;  // comme un bouton
};

Capteur capteurs[NB_IR];
Adafruit_seesaw ss;

// Pins des LEDs correspondant aux capteurs
const int ledPins[NB_IR] = {2, 3, 4, 5, 6};

void setup() {
  Serial.begin(115200);
  if (!ss.begin()) {
    Serial.println("Erreur Seesaw !");
    while (1);
  }

  // Définir les pins des LEDs comme sorties
  for (int i = 0; i < NB_IR; i++) {
    pinMode(ledPins[i], OUTPUT);
    capteurs[i].seuil = 500; // seuil à ajuster selon ton capteur
  }
}

void loop() {
  for (int i = 0; i < NB_IR; i++) {
    // Lecture IR
    capteurs[i].valeurLue = ss.analogRead(i);

    // Mise à jour min/max pour normalisation
    if (capteurs[i].valeurLue < capteurs[i].valMin) capteurs[i].valMin = capteurs[i].valeurLue;
    if (capteurs[i].valeurLue > capteurs[i].valMax) capteurs[i].valMax = capteurs[i].valeurLue;

    // Normalisation 0-1023
    capteurs[i].valeurNormalisee = map(capteurs[i].valeurLue, capteurs[i].valMin, capteurs[i].valMax, 0, 1023);

    // Détection “bouton”
    capteurs[i].onLine = (capteurs[i].valeurNormalisee > capteurs[i].seuil);

    // Allumer / éteindre LED
    digitalWrite(ledPins[i], capteurs[i].onLine ? HIGH : LOW);
  }

  delay(50); // anti-rebond
}
