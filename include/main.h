//
// Created by msimon on 1/6/24.
//

#ifndef MAIN_H
#define MAIN_H

enum class MOTOR_POSITION {
    ACTIVE,
    STANDBY
};

bool isAccelerating(float currentSpeed, float previousSpeed);

bool isSlowingDown(float currentSpeed, float previousSpeed);

void activateBreakLights();

void deactivateBreakLights();

void handleBreakLights(const float &previousSpeed, float averageSpeed);

void handleIndicator(float averageGyroRoll, bool &indicatorOn, unsigned long currentMillis,
                     unsigned long &previousMillis);

void turnRight();

void turnLeft();

void deactivateIndicator();

void resetValues(int &readingsCount, float &accelSumX, float &accelSumY, float &gyroSumRoll);

void setMotorPosition(int lowPower, MOTOR_POSITION &motorPosition);


#endif //MAIN_H
