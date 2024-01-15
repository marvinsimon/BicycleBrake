#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Stepper.h>
#include "main.h"

#define PIN_NEO_PIXEL 45  // Arduino pin that connects to NeoPixel
#define NUM_PIXELS 20    // The number of LEDs (pixels) on NeoPixel

// Define number of steps per rotation:
constexpr int stepsPerRevolution = 2048;

Adafruit_MPU6050 mpu;
Adafruit_NeoPixel neoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);
// Create stepper object called 'myStepper', note the pin order:
Stepper myStepper = Stepper(stepsPerRevolution, 28, 22, 24, 26);
constexpr int numReadings = 100;  // Number of readings to average

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

    neoPixel.begin();

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
        neoPixel.setPixelColor(pixel, neoPixel.Color(255,  0, 0));  // it only takes effect if pixels.show() is called
    }
    neoPixel.setBrightness(10);
    Serial.println("");
    delay(100);
}

void loop() {
    static int readingsCount = 0;
    static float previousSpeed = 0;

    static float accelSumX = 0.0;
    static float accelSumY = 0.0;
    static float accelSumZ = 0.0;


    if (!mpu.getMotionInterruptStatus()) {
        /* Get new sensor events with the readings */
        sensors_event_t a;
        sensors_event_t g;
        sensors_event_t temp;
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
            deactivateBreakLights();
        } else if (isSlowingDown(averageSpeed, previousSpeed)) {
            Serial.println("Breaking");
            activateBreakLights();
        } else {
            Serial.println("Keeping Speed");
            deactivateBreakLights();
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


void activateBreakLights() {
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
        neoPixel.setPixelColor(pixel, neoPixel.Color(255,  0, 0));  // it only takes effect if pixels.show() is called
    }
    neoPixel.setBrightness(255);
    neoPixel.show();
}

void deactivateBreakLights() {
    neoPixel.setBrightness(30);
    neoPixel.show();
}
