#include <MeAuriga.h>
#include <Wire.h>
#include <math.h>

/* ===== CONFIGURATION ===== */
#define LEDPIN   44
#define LEDNUM   12

MeGyro gyro(0, 0x69);
MeRGBLed led(PORT0, LEDNUM);

/* ===== VARIABLES ===== */
double angleRef = 0;

/* ===== FONCTIONS ===== */

double normaliserAngle(double angle) {
  angle = fmod(angle, 360.0);
  if (angle < 0) angle += 360.0;
  return angle;
}

int angleVersLed(double angle) {
  angle = normaliserAngle(angle);
  return (int)((angle + 15) / 30) % LEDNUM;
}

void clearLeds() {
  for (int i = 0; i < LEDNUM; i++) {
    led.setColor(i, 0, 0, 0);
  }
  led.show();
}

void afficherBoussole(int ledIndex) {
  clearLeds();
  led.setColor(ledIndex, 0, 0, 255);   // Bleu
  led.show();
}

/* ===== SETUP ===== */

void setup() {
  Wire.begin();

  gyro.begin();
  gyro.update();

  led.setpin(LEDPIN);
  clearLeds();

  // Direction de référence = NORD
  angleRef = gyro.getAngleZ();
}

/* ===== LOOP ===== */

void loop() {
  gyro.update();

  double angle = gyro.getAngleZ() - angleRef;
  int ledIndex = angleVersLed(angle);

  afficherBoussole(ledIndex);

  delay(40);
}
