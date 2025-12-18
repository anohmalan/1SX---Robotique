#include <MeAuriga.h>

/* ===== CONFIGURATION ===== */
#define ENCODER_PORT PORT_3
#define VOLUME_MIN   0
#define VOLUME_MAX   100

MePort encoder(ENCODER_PORT);

/* ===== VARIABLES ===== */
int volume = 50;           // Volume initial (0–100)
bool lastA = HIGH;         // État précédent du signal A

/* ===== FONCTION ENCODEUR ===== */
void lireEncodeur() {
  bool A = encoder.dpRead1();   // SLOT1
  bool B = encoder.dpRead2();   // SLOT2

  // Détection d'un front
  if (A != lastA) {
    if (A == B)
      volume++;   // sens horaire
    else
      volume--;   // sens antihoraire

    volume = constrain(volume, VOLUME_MIN, VOLUME_MAX);

    Serial.print("Volume = ");
    Serial.println(volume);
  }

  lastA = A;
}

/* ===== SETUP ===== */
void setup() {
  Serial.begin(9600);
  Serial.println("Encodeur comme potentiometre de volume");
}


// int maxPwm = 255;

// //Motor right
// const int M1_PWM = 11;
// const int M1_IN2 = 49; // M1 ENB
// const int M1_IN1 = 48; // M1 ENA

// /// Vitesse ridicule!!
// void FullSpeedMode() {
//   digitalWrite(M1_IN2, LOW);
//   digitalWrite(M1_IN1, HIGH);
//   analogWrite(M1_PWM, maxPwm);
// }

/* ===== LOOP ===== */
void loop() {
  lireEncodeur();

  // Exemple d'utilisation du volume (PWM)
  int pwmValue = map(volume, 0, 100, 0, 255);
  analogWrite(9, pwmValue);  // pin PWM si besoin

  delay(1); // petite stabilité
}
