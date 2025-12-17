#include <Adafruit_seesaw.h>
#include <MeAuriga.h>

#define NB_IR 5

#define PULSE 9
#define RATIO 39.267

MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

float seuil;  // Seuil de détection de la ligne

int speed = 200;
int speedMin = 50;
int speedMax = 255;
int pivotPwm = 100;

bool firstRun = true;

Adafruit_seesaw ss;

int sensorValues[NB_IR];  // Tableau pour stocker les valeurs des capteurs

struct Capteur {
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue = 0;
  int valeurNormalisee = 0;
};


const int maxAngle = 360;
const int anglePivot = 90;

const float pi = 3.14159;

short vitessePivot = 150;

short tolerance = 2;

enum State {PIVOT, FIN};
State state = PIVOT;

unsigned long currentTime;




Capteur capteurs[NB_IR];

void setup() {
  Serial.begin(115200);

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1)
      ;
  }
  Serial.println("Connexion réussie au LyneTracker!");

  encoderConfig();
  gyro.begin();
  //calibrationAutomatique();
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

  void offMotors(){
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0); 
 }

#pragma endregion

#pragma region updates

  void gyroTask(unsigned long ct) {
    gyro.update();
  }

  void encodersTask(unsigned long ct) {
    encoderRight.loop();
    encoderLeft.loop();
  }

#pragma endregion

#pragma region PID

  void goStraight(short speed = 100, short firstRun = 0) {

    static double zAngleGoal = 0.0;  
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    static double errorSum = 0.0;
      
    // PD Controller
    // Change les valeurs selon tes besoins
    // higher kp = plus réactive, peu osciller
    // lowewr kp = sluggish, moins d'oscillation
    // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 6.5;
    const double kd = 1.1;
      
    if (firstRun) {

      gyro.resetData();
      zAngleGoal = gyro.getAngleZ();
      Serial.println ("Setting Vitesse");

      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed); 
        
      return;
    }
      
    error = gyro.getAngleZ() - zAngleGoal;
      
    // Google : ELI5 PID
    // Astuce web : ELI5 = Explain Like I'm 5
    output = kp * error + kd * (error - previousError);
      
    previousError = error;       
  
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);
  }

  double getUnwrappedAngle(bool reset = false) {
    static double lastWrapped = 0.0;
    static double total = 0.0;
    double tourCicle = 360.0;
    double halfCircle = 180.0;

    if (reset) {
      lastWrapped = gyro.getAngleZ();
      total = lastWrapped;
      return total;
    }

    double wrapped = gyro.getAngleZ(); 

    double diff = wrapped - lastWrapped;

    if (diff > halfCircle)  diff -= tourCicle;
    if (diff < -halfCircle) diff += tourCicle;

    total += diff;
    lastWrapped = wrapped;

    return total;
  }

  bool spin(short speed, int targetAngle, bool firstRun = false){

    static double zAngleGoal = 0.0;

    if (firstRun) {
      gyro.resetData();
      getUnwrappedAngle(true);           // reset interne
      zAngleGoal = getUnwrappedAngle() + targetAngle;
      return false;
    }

    double current = getUnwrappedAngle();
    double remaining = zAngleGoal - current;

    Serial.println(current);
    Serial.println(remaining);

    if (fabs(remaining) <= tolerance) {
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      return true;
    }

    // Ajuste dynamiquement la vitesse pour la précision
    double absRem = fabs(remaining);
    int pwm;

    if (absRem > 30) pwm = speed;
    else if (absRem > 10) pwm = speed * 0.6;
    else if (absRem > 3) pwm = speed * 0.3;
    else pwm = speed * 0.15;

    pwm *= (remaining > 0 ? 1 : -1);

    encoderLeft.setMotorPwm(pwm);
    encoderRight.setMotorPwm(pwm);

    return false;
  }


#pragma endregion

#pragma region State

  void pivotState(unsigned long ct){
    static bool firstTime = 1;
    static bool firstRun = 1;

   if(firstRun){    
    firstRun = 0;
    if(firstTime){
      firstTime = 0;
      spin(vitessePivot, maxAngle, 1);
    }

    bool transition = spin(vitessePivot, anglePivot);

    if(transition){
      state = FIN;
      firstTime = 1;
      Serial.println("Sortie etat: Pivot");
      return;
    }}else{

          if(firstTime){
      firstTime = 0;
      spin(vitessePivot, maxAngle, 1);
    }

    bool transition = spin(vitessePivot, anglePivot);

    if(transition || !noDetectLine()){
      state = FIN;
      firstTime = 1;
      Serial.println("Sortie etat: Pivot");
      return;
    }

    }
  }

  void finState(unsigned long ct){
    static bool firstTime = 1;

    if(firstTime){
      Serial.println("Entrée etat: Fin");
      firstTime = 0;
      offMotors();
    }
    

  }

  void manageState(unsigned long ct){

    switch(state){
      case PIVOT:
        pivotState(ct);
        break;
      case FIN:
        finState(ct);
        break;
    }
    
  }

#pragma endregion

#pragma region CALIBRATION

  void calibrer() {
    for (int i = 0; i < 5; i++) {
      capteurs[i].valeurLue = ss.analogRead(i);

      if (capteurs[i].valeurLue < capteurs[i].valeurMin)
        capteurs[i].valeurMin = capteurs[i].valeurLue;

      if (capteurs[i].valeurLue > capteurs[i].valeurMax)
        capteurs[i].valeurMax = capteurs[i].valeurLue;

    }
  }

  bool faireTourComplet(short speed = 150) {
    
    manageState(currentTime);
    
  }
  bool noDetectLine() {
    const int THRESHOLD = 600;  // ajustable selon ton setup

    for (int i = 0; i < 5; i++) {
        if (capteurs[i].valeurNormalisee < THRESHOLD) {
            // Un capteur voit du noir → faux
            return false;
        }
    }
    return true; // Tous sont au-dessus → aucun capteur ne capte la ligne
}

  bool calibrationAutomatique() {
    calibrer();
    if(noDetectLine()){
      state = PIVOT;
    }
    return faireTourComplet(speed);
  }


#pragma endregion


float computePID(float position, float consigne = 0.0f) {
    // Ajuster les coefficients selon vos besoins
    static float kp = 0.2; // Coefficient proportionnel
    static float ki = 0.01; // Coefficient intégral
    static float kd = 0.01; // Coefficient dérivé

    static float integral = 0;
    static float derivative = 0;
    static float lastError = 0;

    float error = position - consigne;

    integral += error;

    // Adapter cette valeur selon les besoins de votre application
    const float integralLimit = 1000;
    
    // Limiter l'intégrale pour éviter l'emballement intégral
    integral = constrain(integral, -integralLimit, integralLimit);

    derivative = error - lastError;
    lastError = error;
    
    float output = kp * error + ki * integral + kd * derivative;
    
    return output;
}

double capteurLectureNormalisee(int index) {
  double valueCapter;

  // Protection : éviter une division par zéro
  if (capteurs[index].valeurMax == capteurs[index].valeurMin) {
    valueCapter = 0.0;
    return valueCapter;
  }

  // Formule de normalisation (résultat entre 0 et 1000) 
  valueCapter = ((capteurs[index].valeurLue - capteurs[index].valeurMin) * 1.0) / (capteurs[index].valeurMax - capteurs[index].valeurMin) * 1000.0;

   return valueCapter;
}

void normaliserValeurs() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].valeurNormalisee = capteurLectureNormalisee(i);
  }
}

float calculerPositionLigne() {
  double numerateur = 0.0;
  double denominateur = 0.0;

  for (int i = 0; i < NB_IR; i++) {
    numerateur += capteurs[i].valeurNormalisee * (i - 2);
    denominateur += capteurs[i].valeurNormalisee;
  }

  if (denominateur == 0) return 0.0;

  double pos = (numerateur / denominateur) * 1000.0;
  return pos;
}

void suivreLigne(float adjustment) {
  bool droite = false;
  const float baseSpeed = 70.0;

  float leftSpeed = baseSpeed - adjustment;
  float rightSpeed = -baseSpeed - adjustment;

  // Appliquer les vitesses aux moteurs
  encoderLeft.setMotorPwm(leftSpeed);
  encoderRight.setMotorPwm(rightSpeed); // sens inverse pour l’autre moteur
}


void loop() {

  float consigne = 0.0f;  // Position centrale
  // Normaliser les valeurs des capteurs
  currentTime = millis();

  //manageState(currentTime);

  //calibrer();
  calibrationAutomatique();
  normaliserValeurs();

  
  // Calculer la position de la ligne
  float position = calculerPositionLigne();

  // Calculer l'ajustement à apporter à la trajectoire
  float adjustment = computePID(position, consigne);

  gyroTask(currentTime);
  encodersTask(currentTime);

  Serial.println(adjustment);

  for(int i = 0; i < NB_IR; i++){
        Serial.print("IR"); Serial.print(i); Serial.print(":");
        Serial.print(" min  ");
        Serial.print(capteurs[i].valeurMin);
        Serial.print(" max  ");
        Serial.println(capteurs[i].valeurMax);
        Serial.print(" norm  ");
        Serial.print(capteurs[i].valeurNormalisee);
        Serial.print(" Lue  ");
        Serial.println(capteurs[i].valeurLue);

        Serial.print("IR"); Serial.print(i); Serial.print(":");
        Serial.print(capteurs[i].valeurLue);
        Serial.print("\t");
        
  }
  // Ajuster la trajectoire du robot en fonction de l'ajustement
  // Par exemple, ajuster la vitesse des moteurs
  suivreLigne(adjustment);

}