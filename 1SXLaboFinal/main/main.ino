#include "gyro.h"
#include "motors.h"

void setup() {
    Serial.begin(9600);

    gyro_init();
    motors_init();
c:\Users\anohm\OneDrive - Cégep de Shawinigan\CShawi\E25\1SX - Robotique\1SX---Robotique\1SX-LaboFinal\gyro.h
    Serial.println("System ready.");
}

void loop() {
    gyro_update();       // lecture gyro / encodeurs / IR
    motors_update();     // mouvements (si tu veux gérer ça automatiquement)

    // Exemple simple :
    // int ligne = ir_getPosition();
    // if (ligne == -1) stop();
}
