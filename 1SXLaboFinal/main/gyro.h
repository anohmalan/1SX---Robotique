#define GYRO_H

#include <Adafruit_seesaw.h>
#include <MeAuriga.h>

#define NB_IR 5

extern MeGyro gyro;
extern MeEncoderOnBoard encoderRight;
extern MeEncoderOnBoard encoderLeft;

void gyro_init();
void gyro_update();

// --- Fonctions capteurs IR ---
int ir_read(int index);            // lire un capteur
int ir_getPosition();              // position de la ligne
bool ir_onLine();                  // détecte si ligne présente

