#include <MeAuriga.h>

#define PULSE 9
#define RATIO 39.267

MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

const int maxAngle = 360;
const int anglePivot = 90;

const float pi = 3.14159;

short vitessePivot = 150;

short tolerance = 2;

enum State {PIVOT, FIN};
State state = PIVOT;

unsigned long currentTime;

// ********* INTERRUPTIONS ***********

#pragma region configuration - encodeur

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
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
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
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

  bool spin(short speed = 360, int targetAngle, bool firstRun = false){

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


void setup() {
  
  Serial.begin(115200);
  encoderConfig();
  gyro.begin();


  delay(3000);

}

void loop() {
  
  currentTime = millis();

  manageState(currentTime);

  gyroTask(currentTime);
  encodersTask(currentTime);

}