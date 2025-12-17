#define MOTORS_H

#include <MeAuriga.h>

extern MeEncoderOnBoard encoderRight;
extern MeEncoderOnBoard encoderLeft;

void motors_init();
void motors_update();

void avancer(int speed);
void reculer(int speed);
void tournerGauche(int speed);
void tournerDroite(int speed);
void stop();


