#include "motors.h"
#include "gyro.h"

void motors_init() {
    encoderRight.setMotorPwm(0);
    encoderLeft.setMotorPwm(0);
}

void motors_update() {
    // si tu veux un contrôle automatique basé sur gyro
    // tu peux mettre ça ici plus tard
}

void avancer(int speed) {
    encoderRight.setMotorPwm(speed);
    encoderLeft.setMotorPwm(-speed);
}

void reculer(int speed) {
    encoderRight.setMotorPwm(-speed);
    encoderLeft.setMotorPwm(speed);
}

void tournerGauche(int speed) {
    encoderRight.setMotorPwm(speed);
    encoderLeft.setMotorPwm(speed);
}

void tournerDroite(int speed) {
    encoderRight.setMotorPwm(-speed);
    encoderLeft.setMotorPwm(-speed);
}

void stop() {
    encoderRight.setMotorPwm(0);
    encoderLeft.setMotorPwm(0);
}
