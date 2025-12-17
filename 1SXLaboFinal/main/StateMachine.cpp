#include "StateMachine.h"
#include "motors.h"
#include "gyro.h"
#include <Arduino.h>


enum State { PIVOT, FIN };
static State state = PIVOT;

static const int maxAngle = 360;
static const int anglePivot = 90;
static const short vitessePivot = 120;

void pivotState(unsigned long ct) {
    static bool first = true;

    if (first) {
        first = false;
        spin(vitessePivot, maxAngle, true);
    }

    if (spin(vitessePivot, anglePivot)) {
        state = FIN;
        first = true;
        Serial.println("Sortie état Pivot");
    }
}

void finState(unsigned long ct) {
    static bool first = true;
    if (first) {
        first = false;
        Serial.println("Entrée état Fin");
        offMotors();
    }
}

void manageState(unsigned long ct) {
    switch (state) {
    case PIVOT: pivotState(ct); break;
    case FIN:   finState(ct);   break;
    }
}
