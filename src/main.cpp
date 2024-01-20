#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <Stepper.h>
#include "main.h"

constexpr int PIN_NEO_PIXEL = 45; // Arduino pin that connects to NeoPixel
constexpr int NUM_PIXELS = 30; // The number of LEDs (pixels) on NeoPixel

constexpr int LOW_POWER_PIN = 52;

constexpr int NUM_PIXELS_INDICATOR_PER_SIDE = 5;

// Define number of steps per rotation:
constexpr int STEPS_PER_REVOLUTION = 2048;
constexpr int INTERVALL = 100;
constexpr int STEPS_TO_DYNAMO = 100;


Adafruit_MPU6050 mpu;
Adafruit_NeoPixel neoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);
// Create stepper object called 'myStepper', note the pin order:
Stepper myStepper = Stepper(STEPS_PER_REVOLUTION, 22, 24, 26, 28);
constexpr int NEEDED_NUMBER_READINGS = 100; // Number of readings to average

void setup() {
    // Set the speed to 5 rpm:
    myStepper.setSpeed(12);
    Serial.begin(9600);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit MPU6050 test!");

    // Try to initialize!
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    pinMode(LOW_POWER_PIN, INPUT);

    neoPixel.begin();

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setMotionInterrupt(true);
    neoPixel.setBrightness(255);
    deactivateBreakLights();
    Serial.println("");
    delay(100);
}


void loop() {
    static int readingsCount = 0;
    static float previousSpeed = 0;
    static float accelSumX = 0.0;
    static float accelSumY = 0.0;
    static float gyroSumRoll = 0.0;

    static unsigned long previousMillis = 0;
    static bool indicatorOn = false;
    static auto motorPosition = MOTOR_POSITION::STANDBY;

    if (!mpu.getMotionInterruptStatus()) {
        /* Get new sensor events with the readings */
        sensors_event_t a;
        sensors_event_t g;
        sensors_event_t temp;
        mpu.getEvent(&a, &g, &temp);

        // Accumulate acceleration values
        accelSumX += a.acceleration.x;
        accelSumY += a.acceleration.y;

        //Accumulate gyro values
        gyroSumRoll += g.gyro.roll;
        readingsCount++;
    }

    setMotorPosition(digitalRead(LOW_POWER_PIN), motorPosition);


    // Check if enough readings have been accumulated
    if (readingsCount < NEEDED_NUMBER_READINGS) {
        return;
    }


    // Calculate average acceleration
    float averageAccelX = accelSumX / NEEDED_NUMBER_READINGS;
    float averageAccelY = accelSumY / NEEDED_NUMBER_READINGS;
    // Calculate average gyro roll
    float averageGyroRoll = gyroSumRoll / NEEDED_NUMBER_READINGS;
    // Calculate average speed (magnitude of the acceleration vector)
    float averageSpeed = sqrtf(averageAccelX * averageAccelX + averageAccelY * averageAccelY);


    resetValues(readingsCount, accelSumX, accelSumY, gyroSumRoll);

    handleBreakLights(previousSpeed, averageSpeed);
    handleIndicator(averageGyroRoll, indicatorOn, millis(), previousMillis);

    previousSpeed = averageSpeed;
}

MOTOR_POSITION getMotorPositionBasedOnPower(const int lowPower) {
    return lowPower > 0 ? MOTOR_POSITION::STANDBY : MOTOR_POSITION::ACTIVE;
}


// This function sets the motor position based on the input power level and updates the motor position variable.
void setMotorPosition(const int lowPower, MOTOR_POSITION &motorPosition) {
    // Get the current motor position based on the provided low power value.
    const auto currentMotorPosition = getMotorPositionBasedOnPower(lowPower);

    // Check if the current motor position is already the desired position.
    if (currentMotorPosition == motorPosition) {
        // If it is, no need to move the motor, so return.
        return;
    }

    // Switch statement to handle different motor positions.
    switch (currentMotorPosition) {
        // If the current position is STANDBY:
        case MOTOR_POSITION::STANDBY:
            // Move the stepper motor 100 steps in the negative direction.
            myStepper.step(-STEPS_TO_DYNAMO);
        // Print a message indicating the movement.
            Serial.println("Move away from wheel");
            break;

        // If the current position is ACTIVE:
        case MOTOR_POSITION::ACTIVE:
            // Move the stepper motor 100 steps in the positive direction.
            myStepper.step(STEPS_TO_DYNAMO);
        // Print a message indicating the movement.
            Serial.println("Move to the wheel");
            break;
    }

    // Update the motor position variable to the current position.
    motorPosition = currentMotorPosition;
}


void resetValues(int &readingsCount, float &accelSumX, float &accelSumY, float &gyroSumRoll) {
    // Reset for the next set of readings
    readingsCount = 0;
    accelSumX = 0.0;
    accelSumY = 0.0;
    gyroSumRoll = 0.0;
}

// Function to handle the turn signal indicator based on gyro roll, managing its activation and deactivation.
void handleIndicator(const float averageGyroRoll, bool &indicatorOn, const unsigned long currentMillis,
                     unsigned long &previousMillis) {
    // Delay for the turn signal to prevent rapid toggling.
    if (currentMillis - previousMillis < INTERVALL) {
        return;
    }

    // Deactivate the turn signal if it is currently on.
    if (indicatorOn) {
        deactivateIndicator();
        indicatorOn = false;
        return;
    }

    // Check gyro roll to determine whether to activate left or right turn signal.
    if (averageGyroRoll > 0.03f) {
        turnRight();
        indicatorOn = true;
    } else if (averageGyroRoll < -0.03f) {
        turnLeft();
        indicatorOn = true;
    } else {
        // If gyro roll is within a neutral range, deactivate the indicator.
        deactivateIndicator();
    }

    // Update the previousMillis for delay management.
    previousMillis = currentMillis;
}

// Function to turn on the right indicator by setting NeoPixels color.
void turnRight() {
    for (int pixel = 0; pixel < NUM_PIXELS_INDICATOR_PER_SIDE; pixel++) {
        // Set the color of each pixel on the right side to indicate a right turn.
        neoPixel.setPixelColor(pixel, 255, 140, 0);
    }
    // Update NeoPixels to display the right turn signal.
    neoPixel.show();
}

// Function to turn on the left indicator by setting NeoPixels color.
void turnLeft() {
    for (int pixel = NUM_PIXELS - NUM_PIXELS_INDICATOR_PER_SIDE; pixel < NUM_PIXELS; pixel++) {
        // Set the color of each pixel on the left side to indicate a left turn.
        neoPixel.setPixelColor(pixel, 255, 140, 0);
    }
    // Update NeoPixels to display the left turn signal.
    neoPixel.show();
}

// Function to deactivate the turn signal indicator by turning off NeoPixels.
void deactivateIndicator() {
    // Turn off the NeoPixels for the right side.
    for (int pixel = NUM_PIXELS - NUM_PIXELS_INDICATOR_PER_SIDE; pixel < NUM_PIXELS; pixel++) {
        neoPixel.setPixelColor(pixel, 0, 0, 0);
    }
    // Turn off the NeoPixels for the left side.
    for (int pixel = 0; pixel < NUM_PIXELS_INDICATOR_PER_SIDE; pixel++) {
        neoPixel.setPixelColor(pixel, 0, 0, 0);
    }
    // Update NeoPixels to hide the turn signal.
    neoPixel.show();
}

// Function to handle the brake lights based on speed comparison.
void handleBreakLights(const float &previousSpeed, const float averageSpeed) {
    // Print the average speed for debugging purposes.
    Serial.print("Average Speed: ");
    Serial.println(averageSpeed);

    // Check if the vehicle is accelerating, slowing down, or maintaining speed.
    if (isAccelerating(averageSpeed, previousSpeed)) {
        Serial.println("Accelerating");
        // If accelerating, deactivate brake lights.
        deactivateBreakLights();
    } else if (isSlowingDown(averageSpeed, previousSpeed)) {
        Serial.println("Braking");
        // If slowing down, activate brake lights.
        activateBreakLights();
    } else {
        Serial.println("Keeping Speed");
        // If maintaining speed, deactivate brake lights.
        deactivateBreakLights();
    }
}

// Function to check if the vehicle is accelerating.
bool isAccelerating(const float currentSpeed, const float previousSpeed) {
    return currentSpeed > previousSpeed + 0.02;
}

// Function to check if the vehicle is slowing down.
bool isSlowingDown(const float currentSpeed, const float previousSpeed) {
    return currentSpeed < previousSpeed - 0.02;
}

// Function to activate the brake lights by setting NeoPixels color.
void activateBreakLights() {
    // Set the color of the brake lights for the central portion of the vehicle.
    for (int pixel = NUM_PIXELS_INDICATOR_PER_SIDE; pixel < NUM_PIXELS - NUM_PIXELS_INDICATOR_PER_SIDE; pixel++) {
        neoPixel.setPixelColor(pixel, 255, 0, 0); // Red color
    }
    // Update NeoPixels to display the activated brake lights.
    neoPixel.show();
}

// Function to deactivate the brake lights by setting NeoPixels color to a dim red.
void deactivateBreakLights() {
    // Set the color of the brake lights to a dim red for the central portion of the vehicle.
    for (int pixel = NUM_PIXELS_INDICATOR_PER_SIDE; pixel < NUM_PIXELS - NUM_PIXELS_INDICATOR_PER_SIDE; pixel++) {
        neoPixel.setPixelColor(pixel, 30, 0, 0); // Dim red color
    }
    // Update NeoPixels to display the deactivated brake lights.
    neoPixel.show();
}
