#include "gyro.h"

// === Déclarations globales ===
Adafruit_seesaw seesaw;
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

static int ir_values[NB_IR];

void gyro_init() {
    // Gyroscope
    gyro.begin();

    // Encodeurs
    encoderRight.setPulse(9);
    encoderLeft.setPulse(9);

    // Seesaw (si tu l’utilises)
    seesaw.begin(0x3A);

    Serial.println("Gyro & sensors initialized");
}

void gyro_update() {
    gyro.update();  // met à jour le gyro
    encoderRight.loop();
    encoderLeft.loop();

    // lecture IR (adapte selon ton hardware)
    for (int i = 0; i < NB_IR; i++) {
        ir_values[i] = analogRead(i);
    }
}

// === Fonctions IR ===

int ir_read(int index) {
    if (index < 0 || index >= NB_IR) return 0;
    return ir_values[index];
}

// retourne un index 0..4 ou -1 si rien
int ir_getPosition() {
    int bestIndex = -1;
    int minValue = 9999;

    for (int i = 0; i < NB_IR; i++) {
        if (ir_values[i] < minValue) {
            minValue = ir_values[i];
            bestIndex = i;
        }
    }
    return bestIndex;
}

bool ir_onLine() {
    return ir_getPosition() != -1;
}
