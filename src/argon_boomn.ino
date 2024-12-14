#include <Wire.h>
#include "MAX30105.h"
#include <vector>

// SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

MAX30105 particleSensor;
void handle(const char *event, const char *data) {
    // Placeholder for webhook response handling if needed
}

// Data Structures
int32_t spo2;              // SPO2 value
int32_t heartRate;         // Heart rate value
std::vector<float> offlineData; // Vector to store data when offline

// Timing Variables
const unsigned long dataInterval = 10000; // 10 seconds in milliseconds
unsigned long lastDataTime = 0;
bool reminderActive = false;

// Non-blocking delay variables
unsigned long yellowFlashPrevMillis = 0;
const long yellowFlashInterval = 500; // 500 ms flash interval

void setup() {
    Serial.begin(9600);
    Particle.subscribe("hook-response/activity", handle, MY_DEVICES);  // Subscribe to webhook
    pinMode(D7, OUTPUT);  // Built-in LED setup (if needed)

    // Attempt to initialize the MAX30102 sensor
    if (!particleSensor.begin()) {  
        Serial.println("MAX30102 not found. Check wiring/power.");
        while (1);  // Stop execution if sensor is not found
    } else {
        Serial.println("MAX30102 sensor initialized.");
    }

    // Configure the MAX30102 sensor (Red and IR LEDs only)
    particleSensor.setup();  
    particleSensor.setPulseAmplitudeRed(0x1F);   // Set Red LED brightness
    particleSensor.setPulseAmplitudeIR(0x1F);    // Set IR LED brightness
    particleSensor.setPulseAmplitudeGreen(0x00); // Disable Green LED (not present in MAX30102)

    Serial.println("MAX30102 sensor setup complete.");
}

void loop() {
    unsigned long currentMillis = millis();

    // Publish any stored offline data when reconnected to the cloud
    if (Particle.connected() && !offlineData.empty()) {
        publishOfflineData();
    }

    // Check if it's time to collect data
    if (currentMillis - lastDataTime >= dataInterval) {
        reminderActive = true;
        collectAndHandleData();  // Collect data from the sensor
        lastDataTime = currentMillis;        
    }

    // If reminder is active, keep flashing yellow
    if (reminderActive) {
        flashYellow(currentMillis);
    }
}

// Collect heart rate, SpO2, and timestamp and send data
void collectAndHandleData() {
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();  // Collect red LED value for SpO2 calculation

    Serial.print("IR Value: ");
    Serial.println(irValue);
    Serial.print("Red Value: ");
    Serial.println(redValue);

    if (irValue > 10000) {  // Check if a valid reading is obtained
        float beatsPerMinute = calculateHeartRate(irValue);
        float calculatedSpO2 = calculateSpO2(redValue, irValue);

        // Round the values to 2 decimal places
        beatsPerMinute = roundf(beatsPerMinute * 100) / 100.0;
        calculatedSpO2 = roundf(calculatedSpO2 * 100) / 100.0;

        // Get the current time as a UTC formatted string
        String timestamp = Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL);

        Serial.print("Calculated Heart Rate: ");
        Serial.println(beatsPerMinute);
        Serial.print("Calculated SpO2: ");
        Serial.println(calculatedSpO2);
        Serial.print("Timestamp: ");
        Serial.println(timestamp);  // Print the timestamp to the serial monitor

        // Publish to Particle Cloud if connected, else store offline
        if (Particle.connected()) {
            // Publish heart rate, SpO2, and timestamp in a JSON format
            Particle.publish("health_data", String::format("{\"hr\":%.2f,\"spo2\":%.2f,\"timestamp\":\"%s\"}", beatsPerMinute, calculatedSpO2, timestamp.c_str()), PRIVATE);
            Serial.println("Published heart rate, SpO2, and timestamp to cloud.");
        } else {
            // Store offline if not connected
            offlineData.push_back(beatsPerMinute);  // Store heart rate offline
            offlineData.push_back(calculatedSpO2);  // Store SpO2 offline
            offlineData.push_back(Time.now());      // Store timestamp (epoch time) offline
            Serial.println("Stored heart rate, SpO2, and timestamp offline.");
        }

        flashGreen();  // Flash green to indicate successful reading
        reminderActive = false;  // Reset reminder
    } else {
        Serial.println("No valid reading detected. Ensure finger is placed properly.");
    }
}

// Function to calculate heart rate (BPM) from the IR value
float calculateHeartRate(long irValue) {
    // Placeholder formula for heart rate calculation
    float beatsPerMinute = irValue / 1831.0;  // Adjust this formula as per your sensor calibration
    return beatsPerMinute;
}

// Function to calculate SpO2 using the Red and IR values (more accurate formula)
float calculateSpO2(long redValue, long irValue) {
    // Correct SpO2 calculation formula based on empirical data and sensor calibration
    float ratio = (float)redValue / (float)irValue;
    float SpO2 = 110 - (25 * ratio);  // Empirical adjustment based on the ratio of red to IR light

    // Clamping the value to a valid SpO2 range
    if (SpO2 < 0) SpO2 = 0;
    if (SpO2 > 100) SpO2 = 100;

    return SpO2;
}

// Publish stored offline data to Particle Cloud
void publishOfflineData() {
    Serial.println("Publishing offline data...");

    // Assuming offlineData holds heart rate, SpO2, and timestamp in sequence
    size_t dataSize = offlineData.size() / 3;  // Assuming each entry has HR, SpO2, and Timestamp
    for (size_t i = 0; i < dataSize; i++) {
        float heartRate = offlineData[i * 3];
        float spo2 = offlineData[i * 3 + 1];
        time_t timestampEpoch = (time_t)offlineData[i * 3 + 2];  // Extract Unix timestamp
        String timestamp = Time.format(timestampEpoch, TIME_FORMAT_ISO8601_FULL);  // Format to UTC

        // Publish heart rate, SpO2, and formatted timestamp
        Particle.publish("health_data", String::format("{\"hr\":%.2f,\"spo2\":%.2f,\"timestamp\":\"%s\"}", heartRate, spo2, timestamp.c_str()), PRIVATE);
        Serial.print("Published offline heart rate: ");
        Serial.println(heartRate);
        Serial.print("Published offline SpO2: ");
        Serial.println(spo2);
        Serial.print("Published offline timestamp: ");
        Serial.println(timestamp);

        delay(1000);  // Prevent hitting the rate limit for Particle.publish
    }

    offlineData.clear();  // Clear the offline data once published
    Serial.println("All offline data published.");
}

// Non-blocking function to flash the LED yellow
void flashYellow(unsigned long currentMillis) {
    if (currentMillis - yellowFlashPrevMillis >= yellowFlashInterval) {
        yellowFlashPrevMillis = currentMillis;
        digitalWrite(D7, !digitalRead(D7));  // Toggle LED state
    }
}

// Function to flash the LED green (indicates successful reading)
void flashGreen() {
    digitalWrite(D7, HIGH);  // Green color (on)
    delay(500);
    digitalWrite(D7, LOW);   // Turn off LED
}
