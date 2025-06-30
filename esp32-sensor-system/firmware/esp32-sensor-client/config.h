#ifndef CONFIG_H
#define CONFIG_H

// ================================================================
// WIFI CONFIGURATION
// ================================================================
#define WIFI_SSID "zaky"
#define WIFI_PASSWORD "12345678"
#define WIFI_TIMEOUT 20000  // 20 seconds

// ================================================================
// SERVER CONFIGURATION
// ================================================================
#define SERVER_URL "http://192.168.12.98:8080/api/data"
#define API_KEY "esp32-zaky"

// ================================================================
// DEVICE CONFIGURATION
// ================================================================
#define DEVICE_ID "esp32-sensor-001" // Make this unique for each device

// ================================================================
// TIMING CONFIGURATION (in milliseconds)
// ================================================================
#define SENSOR_READ_INTERVAL 50    // Read sensors every 1 seconds
#define DATA_SEND_INTERVAL 5      // Send data every 2 seconds  
#define HEARTBEAT_INTERVAL 60000    // Send heartbeat every 5 minutes
#define RETRY_DELAY 5000             // Retry delay for failed operations

// ================================================================
// QUEUE CONFIGURATION
// ================================================================
#define MAX_QUEUE_SIZE 50            // Maximum number of queued data points

// ================================================================
// HARDWARE PIN CONFIGURATION
// ================================================================
#define LED_STATUS_PIN 2             // Built-in LED for status indication

// HC-SR04 Ultrasonic Distance Sensor Configuration  
#define HCSR04_ENABLED true
#define HCSR04_TRIGGER_PIN 5         // HC-SR04 trigger pin
#define HCSR04_ECHO_PIN 18           // HC-SR04 echo pin
#define HCSR04_MAX_DISTANCE 2000      // Maximum distance in cm
#define HCSR04_TIMEOUT 150000        // Timeout in microseconds

// Infrared Sensor Configuration (GPIO 36)
#define INFRARED_ENABLED true
#define INFRARED_PIN 36              // Infrared sensor input pin
#define INFRARED_THRESHOLD 2000      // ADC threshold for hole detection (0-4095)
#define INFRARED_SAMPLES 5           // Number of samples for averaging

// Water Level Sensor Configuration (GPIO 39)
#define WATER_LEVEL_ENABLED true
#define WATER_LEVEL_PIN 39           // Water level sensor input pin
#define WATER_LEVEL_THRESHOLD 1500   // ADC threshold for water detection (0-4095)
#define WATER_LEVEL_SAMPLES 5        // Number of samples for averaging

// Vibration Motor Configuration (GPIO 13)
#define VIBRATION_MOTOR_ENABLED true
#define VIBRATION_MOTOR_PIN 13       // Vibration motor control pin
#define VIBRATION_DURATION 300       // Duration of vibration in milliseconds
#define OBSTACLE_VIBRATION_THRESHOLD 15.0  // Distance threshold for vibration (cm)

// Buzzer Configuration (GPIO 19)
#define BUZZER_ENABLED true
#define BUZZER_PIN 19                // Buzzer control pin
#define BUZZER_PATTERN_HOLE_DURATION 500    // Duration for hole pattern (ms)
#define BUZZER_PATTERN_WATER_DURATION 200   // Duration for water pattern (ms)
#define BUZZER_PATTERN_HOLE_BEEPS 3         // Number of beeps for hole
#define BUZZER_PATTERN_WATER_BEEPS 5        // Number of beeps for water
#define BUZZER_PATTERN_HOLE 1
#define BUZZER_PATTERN_WATER 2

// ================================================================
// SENSOR READING CONFIGURATION
// ================================================================
#define SENSOR_RETRY_COUNT 3         // Number of retries for failed readings
#define SENSOR_RETRY_DELAY 500       // Delay between retries in ms

#endif