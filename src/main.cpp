#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Stepper.h>
#include "main.h"

// Define number of steps per rotation:
constexpr int stepsPerRevolution = 2048;

Adafruit_MPU6050 mpu;
// Create stepper object called 'myStepper', note the pin order:
Stepper myStepper = Stepper(stepsPerRevolution, 28, 22, 24, 26);
constexpr int numReadings = 100;  // Number of readings to average
float accelSumX = 0.0, accelSumY = 0.0, accelSumZ = 0.0;
float previousSpeed = 0;

void setup(void) {
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

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    Serial.println("");
    delay(100);
}

void loop() {
    static int readingsCount = 0;


    if (!mpu.getMotionInterruptStatus()) {
        /* Get new sensor events with the readings */
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Accumulate acceleration values
        accelSumX += a.acceleration.x;
        accelSumY += a.acceleration.y;
        accelSumZ += a.acceleration.z;
        readingsCount++;
        //Serial.println(""+String(readingsCount));
    }

    // Check if enough readings have been accumulated
    if (readingsCount == numReadings) {
        // Calculate average acceleration
        float averageAccelX = accelSumX / numReadings;
        float averageAccelY = accelSumY / numReadings;
        float averageAccelZ = accelSumZ / numReadings;

        // Calculate average speed (magnitude of the acceleration vector)
        float averageSpeed = sqrtf(averageAccelX * averageAccelX + averageAccelY * averageAccelY);



        // Print the result
        Serial.print("Average Speed: ");
        Serial.println(averageSpeed);

        // Reset for the next set of readings
        readingsCount = 0;
        accelSumX = accelSumY = accelSumZ = 0.0;
        if(isAccelerating(averageSpeed, previousSpeed)) {
            Serial.println("Accelerating");
            myStepper.step(100);
        } else if (isSlowingDown(averageSpeed, previousSpeed)) {
            Serial.println("Breaking");
            myStepper.step(-100);
        } else {
            Serial.println("Keeping Speed");
            myStepper.step(0);
        }
        previousSpeed = averageSpeed;
    }
}
bool isAccelerating(const float currentSpeed, const float previousSpeed) {
    return currentSpeed > previousSpeed + 0.02;
}

bool isSlowingDown(const float currentSpeed, const float previousSpeed){
    return currentSpeed < previousSpeed - 0.02;
}