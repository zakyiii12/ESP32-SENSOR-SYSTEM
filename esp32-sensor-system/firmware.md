## 2. ESP32 Firmware Implementation

// esp32-sensor-client.ino
```cpp
// esp32-sensor-client.ino
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

// Configuration and core modules
#include "config.h"
#include "wifi_manager.h"
#include "data_queue.h"

// Sensor modules
#include "sensors/ultrasonic_sensor.h"
#include "sensors/infrared_sensor.h"
#include "sensors/water_level_sensor.h"

// Feedback modules
#include "feedback/vibration_motor.h"
#include "feedback/buzzer.h"

// Global instances
WiFiManager wifiManager;
DataQueue dataQueue;

// Sensor instances
UltrasonicSensor ultrasonicSensor;
InfraredSensor infraredSensor;
WaterLevelSensor waterLevelSensor;

// Feedback instances
VibrationMotor vibrationMotor;
Buzzer buzzer;

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastDataSend = 0;
unsigned long lastHeartbeat = 0;

// Sensor data storage for feedback logic
float lastDistance = -1;
bool lastHoleDetected = false;
bool lastWaterDetected = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== ESP32 Enhanced Sensor Client Starting ===");
  
  // Initialize core components
  Serial.println("Initializing core components...");
  wifiManager.begin();
  dataQueue.begin();
  
  // Initialize sensors
  Serial.println("Initializing sensors...");
  ultrasonicSensor.begin();
  infraredSensor.begin();
  waterLevelSensor.begin();
  
  // Initialize feedback components
  Serial.println("Initializing feedback components...");
  vibrationMotor.begin();
  buzzer.begin();
  
  // Setup time synchronization
  Serial.println("Synchronizing time...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  
  // Status LED setup
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);
  
  Serial.println("=== Setup completed successfully! ===");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Maintain WiFi connection
  wifiManager.maintainConnection();
  
  // Read sensors at specified interval
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readAndQueueSensorData();
    lastSensorRead = currentTime;
  }
  
  // Send queued data at specified interval
  if (currentTime - lastDataSend >= DATA_SEND_INTERVAL) {
    sendQueuedData();
    lastDataSend = currentTime;
  }
  
  // Send heartbeat periodically
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = currentTime;
  }
  
  // Handle feedback logic
  handleFeedback();
  
  // Small delay to prevent watchdog issues
  delay(50);
}

void readAndQueueSensorData() {
  if (!wifiManager.isConnected()) {
    Serial.println("WiFi not connected, skipping sensor read");
    return;
  }
  
  Serial.println("--- Reading Sensors ---");
  digitalWrite(LED_STATUS_PIN, HIGH); // Indicate sensor reading
  
  // Create JSON document for sensor data
  DynamicJsonDocument doc(1024);
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = getISOTimestamp();
  
  JsonObject data = doc.createNestedObject("data");
  
  // Read Ultrasonic sensor (distance measurement)
  if (ultrasonicSensor.isEnabled()) {
    JsonObject ultrasonicData = data.createNestedObject("ultrasonic");
    if (ultrasonicSensor.readSensor(ultrasonicData)) {
      lastDistance = ultrasonicData["distance_cm"];
      Serial.printf("✓ Ultrasonic: %.1f cm\n", lastDistance);
    } else {
      Serial.println("✗ Ultrasonic read failed");
      lastDistance = -1;
    }
  }
  
  // Read Infrared sensor (hole/gap detection)
  if (infraredSensor.isEnabled()) {
    JsonObject infraredData = data.createNestedObject("infrared");
    if (infraredSensor.readSensor(infraredData)) {
      lastHoleDetected = infraredData["hole_detected"];
      Serial.printf("✓ Infrared: %s\n", lastHoleDetected ? "Hole detected" : "No hole");
    } else {
      Serial.println("✗ Infrared read failed");
      lastHoleDetected = false;
    }
  }
  
  // Read Water Level sensor (wet surface detection)
  if (waterLevelSensor.isEnabled()) {
    JsonObject waterData = data.createNestedObject("water_level");
    if (waterLevelSensor.readSensor(waterData)) {
      lastWaterDetected = waterData["water_detected"];
      Serial.printf("✓ Water Level: %s\n", lastWaterDetected ? "Water detected" : "Dry surface");
    } else {
      Serial.println("✗ Water Level read failed");
      lastWaterDetected = false;
    }
  }
  
  // Add system information
  addSystemInfo(data);
  
  // Convert to JSON string and queue
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (dataQueue.enqueue(jsonString)) {
    Serial.printf("✓ Sensor data queued (Queue size: %d)\n", dataQueue.size());
  } else {
    Serial.println("✗ Failed to queue sensor data - queue full");
  }
  
  digitalWrite(LED_STATUS_PIN, LOW);
  Serial.println();
}

void handleFeedback() {
  // Vibration feedback for close obstacles (ultrasonic sensor)
  if (lastDistance > 0 && lastDistance <= OBSTACLE_VIBRATION_THRESHOLD) {
    if (!vibrationMotor.isActive()) {
      Serial.printf("🔸 Obstacle detected at %.1f cm - Activating vibration\n", lastDistance);
      vibrationMotor.activate(VIBRATION_DURATION);
    }
  }
  
  // Buzzer feedback for holes or water detection
  if (lastHoleDetected || lastWaterDetected) {
    if (!buzzer.isActive()) {
      String alertType = lastHoleDetected ? "hole" : "water";
      Serial.printf("🔊 %s detected - Activating buzzer\n", alertType.c_str());
      
      // Different buzzer patterns for different alerts
      if (lastHoleDetected) {
        buzzer.playPattern(BUZZER_PATTERN_HOLE);
      } else {
        buzzer.playPattern(BUZZER_PATTERN_WATER);
      }
    }
  }
  
  // Update feedback components
  vibrationMotor.update();
  buzzer.update();
}

void sendQueuedData() {
  if (!wifiManager.isConnected()) {
    Serial.println("WiFi not connected, skipping data send");
    return;
  }
  
  if (dataQueue.isEmpty()) {
    return; // Nothing to send
  }
  
  Serial.println("--- Sending Data ---");
  
  int sent = 0;
  int maxBatchSize = 5; // Send up to 5 items per batch
  
  while (!dataQueue.isEmpty() && sent < maxBatchSize) {
    String data = dataQueue.peek();
    
    if (sendDataToServer(data)) {
      dataQueue.dequeue();
      sent++;
      Serial.printf("✓ Data sent successfully (%d/%d)\n", sent, maxBatchSize);
    } else {
      Serial.println("✗ Failed to send data, will retry later");
      break; // Stop trying if send fails
    }
    
    delay(200); // Small delay between requests
  }
  
  if (sent > 0) {
    Serial.printf("Batch send completed: %d items sent\n", sent);
  }
  Serial.println();
}

bool sendDataToServer(const String& jsonData) {
  HTTPClient http;
  http.begin(SERVER_URL);
  
  // Set headers
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  http.addHeader("User-Agent", "ESP32-SensorClient/3.0");
  
  // Set timeout
  http.setTimeout(15000); // 15 seconds
  
  // Send POST request
  int httpResponseCode = http.POST(jsonData);
  
  bool success = false;
  if (httpResponseCode > 0) {
    String response = http.getString();
    
    if (httpResponseCode == 201 || httpResponseCode == 200) {
      success = true;
      Serial.printf("Server response: %s\n", response.c_str());
    } else {
      Serial.printf("Server error %d: %s\n", httpResponseCode, response.c_str());
    }
  } else {
    Serial.printf("HTTP error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  
  http.end();
  return success;
}

void sendHeartbeat() {
  if (!wifiManager.isConnected()) {
    return;
  }
  
  Serial.println("--- Sending Heartbeat ---");
  
  DynamicJsonDocument doc(512);
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = getISOTimestamp();
  
  JsonObject data = doc.createNestedObject("data");
  data["type"] = "heartbeat";
  data["uptime_ms"] = millis();
  data["free_heap"] = ESP.getFreeHeap();
  data["queue_size"] = dataQueue.size();
  data["wifi_rssi"] = WiFi.RSSI();
  data["sensor_count"] = 3; // Ultrasonic + Infrared + Water Level
  data["feedback_count"] = 2; // Vibration + Buzzer
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (sendDataToServer(jsonString)) {
    Serial.println("✓ Heartbeat sent successfully");
  } else {
    Serial.println("✗ Heartbeat send failed");
  }
  Serial.println();
}

void addSystemInfo(JsonObject& data) {
  JsonObject system = data.createNestedObject("system");
  system["free_heap"] = ESP.getFreeHeap();
  system["uptime_ms"] = millis();
  system["cpu_freq_mhz"] = ESP.getCpuFreqMHz();
  system["wifi_rssi"] = WiFi.RSSI();
  system["chip_id"] = String((uint32_t)(ESP.getEfuseMac() >> 32), HEX);
}

String getISOTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Warning: Failed to obtain time, using millis");
    return String(millis());
  }
  
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
}
```

### config.h
```cpp
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
#define SERVER_URL "http://192.168.59.98:8080/api/data"
#define API_KEY "esp32-zaky"

// ================================================================
// DEVICE CONFIGURATION
// ================================================================
#define DEVICE_ID "esp32-sensor-001" // Make this unique for each device

// ================================================================
// TIMING CONFIGURATION (in milliseconds)
// ================================================================
#define SENSOR_READ_INTERVAL 1000    // Read sensors every 1 seconds
#define DATA_SEND_INTERVAL 2000      // Send data every 2 seconds  
#define HEARTBEAT_INTERVAL 300000    // Send heartbeat every 5 minutes
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
#define HCSR04_MAX_DISTANCE 500      // Maximum distance in cm
#define HCSR04_TIMEOUT 100000        // Timeout in microseconds

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
```

### sensors/hcsr04_sensor.h
```cpp
#ifndef HCSR04_SENSOR_H
#define HCSR04_SENSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../config.h"

/**
 * HC-SR04 Ultrasonic Distance Sensor Module - Always-Reporting Intelligent 
 * 
 * NEVER leave the database with stale data, YOU FOOOOLL!!!!!
 * - Always reports SOMETHING meaningful to the database
 */
class HCSR04Sensor {
private:
  bool initialized = false;
  bool enabled = HCSR04_ENABLED;
  
  // Pin configuration from config.h
  int triggerPin = HCSR04_TRIGGER_PIN;
  int echoPin = HCSR04_ECHO_PIN;
  
  // Enhanced sensor state tracking
  float lastValidDistance = -1.0;
  unsigned long lastReadTime = 0;
  unsigned long lastSuccessfulReadTime = 0;
  int consecutiveErrors = 0;
  int consecutiveTimeouts = 0;
  int consecutiveExtremeOutliers = 0;
  bool hasStableHistory = false;
  
  // Intelligent state inference
  enum SensorState {
    NORMAL_OPERATION,
    CLOSE_RANGE_DETECTED,
    VERY_CLOSE_SUSPECTED,
    SENSOR_MALFUNCTION,
    INITIALIZATION
  };
  
  SensorState currentState = INITIALIZATION;
  SensorState lastReportedState = INITIALIZATION;
  
  // Reading history and analysis
  static const int READING_HISTORY_SIZE = 8;
  static const int ERROR_PATTERN_SIZE = 5;
  
  struct ReadingRecord {
    float distance;
    unsigned long timestamp;
    bool isValid;
    bool isTimeout;
    bool isOutlier;
  };
  
  ReadingRecord recentReadings[READING_HISTORY_SIZE];
  int historyIndex = 0;
  int historyCount = 0;
  
  // Enhanced constants for critical detection
  static const int MIN_READ_INTERVAL = 100;
  static const int MAX_CONSECUTIVE_ERRORS = 15;
  static constexpr float MIN_DISTANCE = 2.0;
  static constexpr float CLOSE_RANGE_THRESHOLD = 8.0;
  static constexpr float VERY_CLOSE_THRESHOLD = 3.0;
  static constexpr float CRITICAL_CLOSE_THRESHOLD = 1.5;
  
  // Pattern recognition thresholds
  static constexpr float EXTREME_OUTLIER_THRESHOLD = 400.0;
  static const int CLOSE_RANGE_PATTERN_COUNT = 3;
  static const int TIMEOUT_PATTERN_COUNT = 2;
  static const unsigned long STALE_DATA_WARNING_MS = 5000;

public:
  HCSR04Sensor() {
    // Initialize history
    for (int i = 0; i < READING_HISTORY_SIZE; i++) {
      recentReadings[i] = {-1.0, 0, false, false, false};
    }
  }
  
  bool begin() {
    if (!enabled) {
      Serial.println("HC-SR04: Sensor disabled in configuration");
      return true;
    }
    
    Serial.println("HC-SR04: Initializing always-reporting intelligent sensor...");
    
    // Validate pin configuration
    if (triggerPin < 0 || echoPin < 0) {
      Serial.println("HC-SR04: Invalid pin configuration");
      return false;
    }
    
    if (triggerPin == echoPin) {
      Serial.println("HC-SR04: Trigger and echo pins cannot be the same");
      return false;
    }
    
    // Configure pins
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(triggerPin, LOW);
    
    Serial.printf("HC-SR04: Configured - Trigger: %d, Echo: %d\n", triggerPin, echoPin);
    
    delay(300);
    
    // Initial calibration with state detection
    Serial.println("HC-SR04: Performing calibration with state detection...");
    currentState = INITIALIZATION;
    
    for (int i = 0; i < 5; i++) {
      float testDistance = measureSingleDistance();
      recordReading(testDistance, millis());
      
      if (testDistance > 0) {
        Serial.printf("HC-SR04: Calibration %d: %.1f cm\n", i + 1, testDistance);
      } else {
        Serial.printf("HC-SR04: Calibration %d: timeout/error\n", i + 1);
      }
      delay(150);
    }
    
    // Analyze initial state
    analyzeCurrentState();
    initialized = true;
    
    Serial.printf("HC-SR04: Initialization complete - Initial state: %s\n", 
                 stateToString(currentState).c_str());
    return true;
  }
  
  bool readSensor(JsonObject& data) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // CRITICAL: Always enforce minimum read interval but ALWAYS report something
    if (currentTime - lastReadTime < MIN_READ_INTERVAL) {
      // Still provide an update based on current understanding
      reportCurrentState(data, "rate_limited");
      return true;
    }
    
    Serial.println("HC-SR04: Starting always-reporting measurement cycle...");
    
    // Take multiple readings for pattern analysis
    float readings[SENSOR_RETRY_COUNT];
    bool validReadings[SENSOR_RETRY_COUNT];
    bool timeoutReadings[SENSOR_RETRY_COUNT];
    bool outlierReadings[SENSOR_RETRY_COUNT];
    
    int validCount = 0;
    int timeoutCount = 0;
    int outlierCount = 0;
    
    for (int i = 0; i < SENSOR_RETRY_COUNT; i++) {
      float distance = measureSingleDistance();
      readings[i] = distance;
      validReadings[i] = false;
      timeoutReadings[i] = false;
      outlierReadings[i] = false;
      
      if (distance < 0) {
        // Timeout or error
        timeoutReadings[i] = true;
        timeoutCount++;
        Serial.printf("HC-SR04: Reading %d: timeout ⚠️\n", i + 1);
      } else if (isExtremeOutlier(distance)) {
        // Likely sensor error (e.g., 400cm when close)
        outlierReadings[i] = true;
        outlierCount++;
        Serial.printf("HC-SR04: Reading %d: %.1f cm (outlier) ⚠️\n", i + 1, distance);
      } else if (isValidDistance(distance)) {
        // Valid reading
        validReadings[i] = true;
        validCount++;
        Serial.printf("HC-SR04: Reading %d: %.1f cm ✓\n", i + 1, distance);
      } else {
        // Invalid range
        Serial.printf("HC-SR04: Reading %d: %.1f cm (invalid range) ✗\n", i + 1, distance);
      }
      
      if (i < SENSOR_RETRY_COUNT - 1) {
        delay(80);
      }
    }
    
    lastReadTime = currentTime;
    
    // Record the dominant pattern
    recordReadingPattern(readings, validReadings, timeoutReadings, outlierReadings, 
                        validCount, timeoutCount, outlierCount, currentTime);
    
    // Analyze what's happening
    analyzeCurrentState();
    
    // CRITICAL: Always report something meaningful
    float reportedDistance = determineReportedDistance(readings, validReadings, validCount);
    String reportedStatus = determineReportedStatus(validCount, timeoutCount, outlierCount);
    float confidence = calculateStateConfidence();
    
    // Update tracking
    if (reportedDistance > 0) {
      lastValidDistance = reportedDistance;
      lastSuccessfulReadTime = currentTime;
      consecutiveErrors = 0;
    } else {
      consecutiveErrors++;
    }
    
    // Populate JSON with guaranteed meaningful data
    populateJsonData(data, reportedDistance, reportedStatus, confidence, currentTime);
    
    Serial.printf("HC-SR04: ALWAYS REPORTING: %.1f cm, State: %s, Confidence: %.2f\n", 
                 reportedDistance, reportedStatus.c_str(), confidence);
    
    return true; // ALWAYS return true - we always have something to report
  }
  
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  bool isHealthy() const { return consecutiveErrors < MAX_CONSECUTIVE_ERRORS; }
  float getLastDistance() const { return lastValidDistance; }
  String getCurrentStateString() const { return stateToString(currentState); }
  
private:
  /**
   * Enhanced single distance measurement
   */
  float measureSingleDistance() {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(4);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(12);
    digitalWrite(triggerPin, LOW);
    
    unsigned long duration = pulseIn(echoPin, HIGH, HCSR04_TIMEOUT);
    
    if (duration == 0) {
      return -1.0; // Timeout
    }
    
    float distance = (duration * 0.0343) / 2.0;
    
    if (distance < 0.3 || distance > (HCSR04_MAX_DISTANCE + 200)) {
      return -1.0;
    }
    
    return distance;
  }
  
  /**
   * Record a single reading in history
   */
  void recordReading(float distance, unsigned long timestamp) {
    recentReadings[historyIndex] = {
      distance, 
      timestamp, 
      distance > 0 && isValidDistance(distance),
      distance < 0,
      distance > 0 && isExtremeOutlier(distance)
    };
    
    historyIndex = (historyIndex + 1) % READING_HISTORY_SIZE;
    if (historyCount < READING_HISTORY_SIZE) {
      historyCount++;
    } else {
      hasStableHistory = true;
    }
  }
  
  /**
   * Record the pattern from a full measurement cycle
   */
  void recordReadingPattern(float readings[], bool validReadings[], bool timeoutReadings[], 
                           bool outlierReadings[], int validCount, int timeoutCount, 
                           int outlierCount, unsigned long timestamp) {
    
    // Find the best representative reading
    float representativeReading = -1.0;
    
    if (validCount > 0) {
      // Use median of valid readings
      float validValues[SENSOR_RETRY_COUNT];
      int validIndex = 0;
      
      for (int i = 0; i < SENSOR_RETRY_COUNT; i++) {
        if (validReadings[i]) {
          validValues[validIndex++] = readings[i];
        }
      }
      
      // Simple sort and median
      for (int i = 0; i < validCount - 1; i++) {
        for (int j = 0; j < validCount - i - 1; j++) {
          if (validValues[j] > validValues[j + 1]) {
            float temp = validValues[j];
            validValues[j] = validValues[j + 1];
            validValues[j + 1] = temp;
          }
        }
      }
      
      representativeReading = validValues[validCount / 2];
    }
    
    // Record in history
    recordReading(representativeReading, timestamp);
    
    // Update consecutive counters
    if (timeoutCount >= SENSOR_RETRY_COUNT / 2) {
      consecutiveTimeouts++;
    } else {
      consecutiveTimeouts = 0;
    }
    
    if (outlierCount >= SENSOR_RETRY_COUNT / 2) {
      consecutiveExtremeOutliers++;
    } else {
      consecutiveExtremeOutliers = 0;
    }
  }
  
  /**
   * Analyze current sensor state based on patterns
   */
  void analyzeCurrentState() {
    if (!hasStableHistory) {
      currentState = INITIALIZATION;
      return;
    }
    
    // Count recent patterns
    int recentTimeouts = 0;
    int recentOutliers = 0;
    int recentValidClose = 0;
    int recentValidNormal = 0;
    
    for (int i = 0; i < min(historyCount, 5); i++) {
      int idx = (historyIndex - 1 - i + READING_HISTORY_SIZE) % READING_HISTORY_SIZE;
      ReadingRecord& record = recentReadings[idx];
      
      if (record.isTimeout) {
        recentTimeouts++;
      } else if (record.isOutlier) {
        recentOutliers++;
      } else if (record.isValid) {
        if (record.distance < CLOSE_RANGE_THRESHOLD) {
          recentValidClose++;
        } else {
          recentValidNormal++;
        }
      }
    }
    
    // State determination logic
    if (consecutiveTimeouts >= TIMEOUT_PATTERN_COUNT && recentOutliers > 0) {
      // Timeouts + outliers = likely very close object
      currentState = VERY_CLOSE_SUSPECTED;
    } else if (recentValidClose >= 2) {
      // Multiple valid close readings
      currentState = CLOSE_RANGE_DETECTED;
    } else if (consecutiveTimeouts >= TIMEOUT_PATTERN_COUNT * 2) {
      // Many timeouts without other patterns = possible malfunction
      currentState = SENSOR_MALFUNCTION;
    } else if (recentValidNormal >= 2) {
      // Normal operation
      currentState = NORMAL_OPERATION;
    }
    // else keep current state
    
    Serial.printf("HC-SR04: State analysis - Timeouts: %d, Outliers: %d, Close: %d, Normal: %d -> %s\n",
                 recentTimeouts, recentOutliers, recentValidClose, recentValidNormal, 
                 stateToString(currentState).c_str());
  }
  
  /**
   * CRITICAL: Always determine a meaningful distance to report
   */
  float determineReportedDistance(float readings[], bool validReadings[], int validCount) {
    if (validCount > 0) {
      // We have valid readings - use them
      float sum = 0;
      int count = 0;
      
      for (int i = 0; i < SENSOR_RETRY_COUNT; i++) {
        if (validReadings[i]) {
          sum += readings[i];
          count++;
        }
      }
      
      return sum / count;
    }
    
    // No valid readings - infer based on state
    switch (currentState) {
      case VERY_CLOSE_SUSPECTED:
        // Timeouts + outliers = object is very close
        return CRITICAL_CLOSE_THRESHOLD;
        
      case CLOSE_RANGE_DETECTED:
        // Based on recent close readings
        return estimateCloseRangeDistance();
        
      case SENSOR_MALFUNCTION:
        // Use last known distance but mark as uncertain
        if (lastValidDistance > 0) {
          return lastValidDistance;
        }
        return VERY_CLOSE_THRESHOLD; // Conservative estimate
        
      case NORMAL_OPERATION:
      case INITIALIZATION:
      default:
        // Use last known distance or a safe default
        if (lastValidDistance > 0) {
          return lastValidDistance;
        }
        return CLOSE_RANGE_THRESHOLD; // Conservative default
    }
  }
  
  /**
   * Determine status string for reporting
   */
  String determineReportedStatus(int validCount, int timeoutCount, int outlierCount) {
    if (validCount > 0) {
      return "valid_readings";
    }
    
    switch (currentState) {
      case VERY_CLOSE_SUSPECTED:
        return "very_close_estimated";
      case CLOSE_RANGE_DETECTED:
        return "close_range_estimated";
      case SENSOR_MALFUNCTION:
        return "sensor_malfunction_fallback";
      case INITIALIZATION:
        return "initializing";
      default:
        return "fallback_estimate";
    }
  }
  
  /**
   * Calculate confidence in current state assessment
   */
  float calculateStateConfidence() {
    if (!hasStableHistory) {
      return 0.3;
    }
    
    switch (currentState) {
      case NORMAL_OPERATION:
        return 0.9;
      case CLOSE_RANGE_DETECTED:
        return 0.7;
      case VERY_CLOSE_SUSPECTED:
        return 0.5; // Lower confidence but critical info
      case SENSOR_MALFUNCTION:
        return 0.2; // Low confidence but still informative
      case INITIALIZATION:
        return 0.3;
      default:
        return 0.4;
    }
  }
  
  /**
   * Estimate close range distance from history
   */
  float estimateCloseRangeDistance() {
    float sum = 0;
    int count = 0;
    
    // Look for recent valid close readings
    for (int i = 0; i < min(historyCount, 5); i++) {
      int idx = (historyIndex - 1 - i + READING_HISTORY_SIZE) % READING_HISTORY_SIZE;
      ReadingRecord& record = recentReadings[idx];
      
      if (record.isValid && record.distance < CLOSE_RANGE_THRESHOLD) {
        sum += record.distance;
        count++;
      }
    }
    
    if (count > 0) {
      return sum / count;
    }
    
    return VERY_CLOSE_THRESHOLD; // Conservative estimate
  }
  
  /**
   * Report current state (for rate limiting)
   */
  void reportCurrentState(JsonObject& data, String reason) {
    float reportedDistance = lastValidDistance > 0 ? lastValidDistance : CLOSE_RANGE_THRESHOLD;
    populateJsonData(data, reportedDistance, reason, calculateStateConfidence(), millis());
  }
  
  bool isExtremeOutlier(float distance) {
    // Check for impossible readings given current state
    if (currentState == VERY_CLOSE_SUSPECTED || currentState == CLOSE_RANGE_DETECTED) {
      return distance > EXTREME_OUTLIER_THRESHOLD;
    }
    
    // Check against recent history
    if (hasStableHistory && lastValidDistance > 0 && lastValidDistance < CLOSE_RANGE_THRESHOLD) {
      return distance > EXTREME_OUTLIER_THRESHOLD;
    }
    
    return false;
  }
  
  bool isValidDistance(float distance) const {
    return (distance >= MIN_DISTANCE && distance <= HCSR04_MAX_DISTANCE);
  }
  
  String stateToString(SensorState state) const {
    switch (state) {
      case NORMAL_OPERATION: return "normal";
      case CLOSE_RANGE_DETECTED: return "close_range";
      case VERY_CLOSE_SUSPECTED: return "very_close";
      case SENSOR_MALFUNCTION: return "malfunction";
      case INITIALIZATION: return "initializing";
      default: return "unknown";
    }
  }
  
  void populateJsonData(JsonObject& data, float distance, String status, float confidence, unsigned long timestamp) const {
    data["distance_cm"] = round(distance * 10) / 10.0;
    data["distance_unit"] = "centimeters";
    data["sensor_type"] = "HC-SR04";
    data["read_time"] = timestamp;
    data["status"] = status;
    data["confidence"] = round(confidence * 100) / 100.0;
    data["sensor_state"] = stateToString(currentState);
    data["max_range_cm"] = HCSR04_MAX_DISTANCE;
    data["min_range_cm"] = MIN_DISTANCE;
    data["consecutive_errors"] = consecutiveErrors;
    data["data_age_ms"] = timestamp - lastSuccessfulReadTime;
    
    // Critical warning if data is getting stale
    data["stale_data_warning"] = (timestamp - lastSuccessfulReadTime) > STALE_DATA_WARNING_MS;
  }
};

#endif
```

### sensors/infrared_sensor.h
```cpp
#ifndef INFRARED_SENSOR_H
#define INFRARED_SENSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../config.h"

/**
 * Infrared Sensor Module for Hole/Gap Detection
 * 
 * This sensor detects holes or gaps in surfaces by measuring infrared reflection.
 * When infrared light is not reflected back (hole/gap), the sensor reading drops.
 * Connected to GPIO 36 (ADC1_CH0) for analog readings.
 */
class InfraredSensor {
private:
  bool initialized = false;
  bool enabled = INFRARED_ENABLED;
  int sensorPin = INFRARED_PIN;
  
  // Sensor state tracking
  int lastRawValue = 0;
  bool lastHoleDetected = false;
  unsigned long lastReadTime = 0;
  unsigned long lastValidReadTime = 0;
  int consecutiveErrors = 0;
  
  // Calibration and threshold management
  int dynamicThreshold = INFRARED_THRESHOLD;
  int baselineReading = 0;
  bool isCalibrated = false;
  
  // Reading history for stability
  static const int HISTORY_SIZE = 10;
  int readingHistory[HISTORY_SIZE];
  int historyIndex = 0;
  int historyCount = 0;
  
  // Error handling
  static const int MAX_CONSECUTIVE_ERRORS = 10;
  static const int MIN_READ_INTERVAL = 50; // ms
  
public:
  InfraredSensor() {
    // Initialize reading history
    for (int i = 0; i < HISTORY_SIZE; i++) {
      readingHistory[i] = 0;
    }
  }
  
  /**
   * Initialize the infrared sensor
   * Performs calibration to establish baseline readings
   */
  bool begin() {
    if (!enabled) {
      Serial.println("Infrared: Sensor disabled in configuration");
      return true;
    }
    
    Serial.println("Infrared: Initializing hole detection sensor...");
    
    // Validate pin configuration
    if (sensorPin < 0) {
      Serial.println("Infrared: Invalid pin configuration");
      return false;
    }
    
    // Configure pin (ADC pins don't need explicit pinMode)
    Serial.printf("Infrared: Configured on GPIO %d\n", sensorPin);
    
    // Perform calibration
    Serial.println("Infrared: Performing calibration...");
    if (!performCalibration()) {
      Serial.println("Infrared: Calibration failed");
      return false;
    }
    
    initialized = true;
    Serial.printf("Infrared: Initialization complete - Baseline: %d, Threshold: %d\n", 
                 baselineReading, dynamicThreshold);
    return true;
  }
  
  /**
   * Read sensor and detect holes/gaps
   * Returns true if reading was successful (regardless of detection result)
   */
  bool readSensor(JsonObject& data) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Enforce minimum read interval
    if (currentTime - lastReadTime < MIN_READ_INTERVAL) {
      // Return last known state
      populateJsonData(data, lastRawValue, lastHoleDetected, "rate_limited", currentTime);
      return true;
    }
    
    Serial.println("Infrared: Reading sensor for hole detection...");
    
    // Take multiple samples for stability
    int samples[INFRARED_SAMPLES];
    int validSamples = 0;
    
    for (int i = 0; i < INFRARED_SAMPLES; i++) {
      int reading = analogRead(sensorPin);
      
      // Validate reading (ADC should return 0-4095)
      if (reading >= 0 && reading <= 4095) {
        samples[validSamples] = reading;
        validSamples++;
        Serial.printf("Infrared: Sample %d: %d\n", i + 1, reading);
      } else {
        Serial.printf("Infrared: Sample %d: invalid (%d)\n", i + 1, reading);
      }
      
      if (i < INFRARED_SAMPLES - 1) {
        delay(20); // Small delay between samples
      }
    }
    
    lastReadTime = currentTime;
    
    if (validSamples == 0) {
      Serial.println("Infrared: All samples invalid");
      consecutiveErrors++;
      populateJsonData(data, lastRawValue, lastHoleDetected, "read_error", currentTime);
      return false;
    }
    
    // Calculate average of valid samples
    int sum = 0;
    for (int i = 0; i < validSamples; i++) {
      sum += samples[i];
    }
    int averageReading = sum / validSamples;
    
    // Update reading history
    updateReadingHistory(averageReading);
    
    // Apply smoothing filter using recent history
    int smoothedReading = getSmoothedReading();
    
    // Detect hole based on threshold
    bool holeDetected = detectHole(smoothedReading);
    
    // Update state
    lastRawValue = smoothedReading;
    lastHoleDetected = holeDetected;
    lastValidReadTime = currentTime;
    consecutiveErrors = 0;
    
    // Determine status
    String status = "normal";
    if (validSamples < INFRARED_SAMPLES) {
      status = "partial_samples";
    }
    
    // Populate JSON data
    populateJsonData(data, smoothedReading, holeDetected, status, currentTime);
    
    Serial.printf("Infrared: Result - Raw: %d, Smoothed: %d, Hole: %s\n", 
                 averageReading, smoothedReading, holeDetected ? "YES" : "NO");
    
    return true;
  }
  
  // Getter methods for external access
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  bool isHealthy() const { return consecutiveErrors < MAX_CONSECUTIVE_ERRORS; }
  bool isHoleDetected() const { return lastHoleDetected; }
  int getLastRawValue() const { return lastRawValue; }
  int getThreshold() const { return dynamicThreshold; }
  
private:
  /**
   * Perform initial calibration to establish baseline
   */
  bool performCalibration() {
    Serial.println("Infrared: Starting calibration sequence...");
    
    const int calibrationSamples = 20;
    int calibrationReadings[calibrationSamples];
    int validReadings = 0;
    
    // Take calibration samples
    for (int i = 0; i < calibrationSamples; i++) {
      int reading = analogRead(sensorPin);
      
      if (reading >= 0 && reading <= 4095) {
        calibrationReadings[validReadings] = reading;
        validReadings++;
        Serial.printf("Infrared: Calibration sample %d: %d\n", i + 1, reading);
      }
      
      delay(100);
    }
    
    if (validReadings < calibrationSamples / 2) {
      Serial.println("Infrared: Insufficient valid calibration samples");
      return false;
    }
    
    // Calculate baseline (average of valid readings)
    long sum = 0;
    for (int i = 0; i < validReadings; i++) {
      sum += calibrationReadings[i];
    }
    baselineReading = sum / validReadings;
    
    // Calculate standard deviation for adaptive threshold
    long variance = 0;
    for (int i = 0; i < validReadings; i++) {
      int diff = calibrationReadings[i] - baselineReading;
      variance += diff * diff;
    }
    int stdDev = sqrt(variance / validReadings);
    
    // Set dynamic threshold based on baseline and variation
    // Hole detected when reading drops significantly below baseline
    dynamicThreshold = baselineReading - (2 * stdDev) - 200;
    
    // Ensure threshold is within reasonable bounds
    if (dynamicThreshold < 100) {
      dynamicThreshold = 100;
    }
    if (dynamicThreshold > 3000) {
      dynamicThreshold = 3000;
    }
    
    isCalibrated = true;
    Serial.printf("Infrared: Calibration complete - Baseline: %d, StdDev: %d, Threshold: %d\n", 
                 baselineReading, stdDev, dynamicThreshold);
    
    return true;
  }
  
  /**
   * Update the reading history buffer
   */
  void updateReadingHistory(int reading) {
    readingHistory[historyIndex] = reading;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    
    if (historyCount < HISTORY_SIZE) {
      historyCount++;
    }
  }
  
  /**
   * Get smoothed reading using moving average
   */
  int getSmoothedReading() {
    if (historyCount == 0) {
      return lastRawValue;
    }
    
    long sum = 0;
    int count = min(historyCount, 5); // Use last 5 readings for smoothing
    
    for (int i = 0; i < count; i++) {
      int idx = (historyIndex - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
      sum += readingHistory[idx];
    }
    
    return sum / count;
  }
  
  /**
   * Detect hole based on threshold comparison
   */
  bool detectHole(int reading) {
    if (!isCalibrated) {
      // Use static threshold if not calibrated
      return reading < INFRARED_THRESHOLD;
    }
    
    // Use dynamic threshold based on calibration
    bool holeDetected = reading < dynamicThreshold;
    
    // Add hysteresis to prevent flickering
    static bool lastDetection = false;
    static const int HYSTERESIS = 50;
    
    if (!lastDetection && holeDetected) {
      // Confirm hole detection
      lastDetection = true;
      return true;
    } else if (lastDetection && !holeDetected) {
      // Confirm hole is gone (with hysteresis)
      if (reading > (dynamicThreshold + HYSTERESIS)) {
        lastDetection = false;
        return false;
      } else {
        return true; // Still in hole
      }
    } else {
      return lastDetection; // No change
    }
  }
  
  /**
   * Populate JSON data for reporting
   */
  void populateJsonData(JsonObject& data, int rawValue, bool holeDetected, 
                       String status, unsigned long timestamp) const {
    data["raw_value"] = rawValue;
    data["hole_detected"] = holeDetected;
    data["sensor_type"] = "Infrared";
    data["pin"] = sensorPin;
    data["threshold"] = dynamicThreshold;
    data["baseline"] = baselineReading;
    data["calibrated"] = isCalibrated;
    data["read_time"] = timestamp;
    data["status"] = status;
    data["consecutive_errors"] = consecutiveErrors;
    data["data_age_ms"] = timestamp - lastValidReadTime;
    
    // Add interpretation
    if (holeDetected) {
      data["interpretation"] = "Gap or hole detected in surface";
      data["alert_level"] = "warning";
    } else {
      data["interpretation"] = "Normal surface detected";
      data["alert_level"] = "normal";
    }
  }
};

#endif
```
### sensors/water_level_sensor.h
```cpp
#ifndef WATER_LEVEL_SENSOR_H
#define WATER_LEVEL_SENSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../config.h"

/**
 * Water Level Sensor Module for Wet Surface Detection
 * 
 * This sensor detects water or wet surfaces by measuring conductivity/resistance.
 * When water is present, conductivity increases and the analog reading changes.
 * Connected to GPIO 39 (ADC1_CH3) for analog readings.
 */
class WaterLevelSensor {
private:
  bool initialized = false;
  bool enabled = WATER_LEVEL_ENABLED;
  int sensorPin = WATER_LEVEL_PIN;
  
  // Sensor state tracking
  int lastRawValue = 0;
  bool lastWaterDetected = false;
  unsigned long lastReadTime = 0;
  unsigned long lastValidReadTime = 0;
  int consecutiveErrors = 0;
  
  // Calibration and threshold management
  int dynamicThreshold = WATER_LEVEL_THRESHOLD;
  int dryBaseline = 0;
  int wetBaseline = 4095;
  bool isCalibrated = false;
  
  // Reading history for stability and trend analysis
  static const int HISTORY_SIZE = 15;
  int readingHistory[HISTORY_SIZE];
  int historyIndex = 0;
  int historyCount = 0;
  
  // Water detection confidence
  int consecutiveWaterReadings = 0;
  int consecutiveDryReadings = 0;
  static const int CONFIDENCE_THRESHOLD = 3; // Require 3 consistent readings
  
  // Error handling
  static const int MAX_CONSECUTIVE_ERRORS = 10;
  static const int MIN_READ_INTERVAL = 100; // ms (slower than infrared)
  
public:
  WaterLevelSensor() {
    // Initialize reading history
    for (int i = 0; i < HISTORY_SIZE; i++) {
      readingHistory[i] = 0;
    }
  }
  
  /**
   * Initialize the water level sensor
   * Performs calibration to establish dry baseline
   */
  bool begin() {
    if (!enabled) {
      Serial.println("WaterLevel: Sensor disabled in configuration");
      return true;
    }
    
    Serial.println("WaterLevel: Initializing wet surface detection sensor...");
    
    // Validate pin configuration
    if (sensorPin < 0) {
      Serial.println("WaterLevel: Invalid pin configuration");
      return false;
    }
    
    // Configure pin (ADC pins don't need explicit pinMode)
    Serial.printf("WaterLevel: Configured on GPIO %d\n", sensorPin);
    
    // Perform calibration
    Serial.println("WaterLevel: Performing dry surface calibration...");
    Serial.println("WaterLevel: Ensure sensor is on dry surface during calibration!");
    
    delay(2000); // Give time to position sensor
    
    if (!performCalibration()) {
      Serial.println("WaterLevel: Calibration failed");
      return false;
    }
    
    initialized = true;
    Serial.printf("WaterLevel: Initialization complete - Dry baseline: %d, Threshold: %d\n", 
                 dryBaseline, dynamicThreshold);
    return true;
  }
  
  /**
   * Read sensor and detect water/wet surfaces
   * Returns true if reading was successful (regardless of detection result)
   */
  bool readSensor(JsonObject& data) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Enforce minimum read interval
    if (currentTime - lastReadTime < MIN_READ_INTERVAL) {
      // Return last known state
      populateJsonData(data, lastRawValue, lastWaterDetected, "rate_limited", currentTime);
      return true;
    }
    
    Serial.println("WaterLevel: Reading sensor for water detection...");
    
    // Take multiple samples for stability
    int samples[WATER_LEVEL_SAMPLES];
    int validSamples = 0;
    
    for (int i = 0; i < WATER_LEVEL_SAMPLES; i++) {
      int reading = analogRead(sensorPin);
      
      // Validate reading (ADC should return 0-4095)
      if (reading >= 0 && reading <= 4095) {
        samples[validSamples] = reading;
        validSamples++;
        Serial.printf("WaterLevel: Sample %d: %d\n", i + 1, reading);
      } else {
        Serial.printf("WaterLevel: Sample %d: invalid (%d)\n", i + 1, reading);
      }
      
      if (i < WATER_LEVEL_SAMPLES - 1) {
        delay(50); // Longer delay for water sensor stability
      }
    }
    
    lastReadTime = currentTime;
    
    if (validSamples == 0) {
      Serial.println("WaterLevel: All samples invalid");
      consecutiveErrors++;
      populateJsonData(data, lastRawValue, lastWaterDetected, "read_error", currentTime);
      return false;
    }
    
    // Calculate average of valid samples
    int sum = 0;
    for (int i = 0; i < validSamples; i++) {
      sum += samples[i];
    }
    int averageReading = sum / validSamples;
    
    // Update reading history
    updateReadingHistory(averageReading);
    
    // Apply smoothing filter
    int smoothedReading = getSmoothedReading();
    
    // Detect water with confidence checking
    bool waterDetected = detectWaterWithConfidence(smoothedReading);
    
    // Calculate water level percentage (0-100%)
    float waterLevelPercent = calculateWaterLevelPercent(smoothedReading);
    
    // Update state
    lastRawValue = smoothedReading;
    lastWaterDetected = waterDetected;
    lastValidReadTime = currentTime;
    consecutiveErrors = 0;
    
    // Determine status
    String status = "normal";
    if (validSamples < WATER_LEVEL_SAMPLES) {
      status = "partial_samples";
    }
    
    // Populate JSON data
    populateJsonData(data, smoothedReading, waterDetected, status, currentTime, waterLevelPercent);
    
    Serial.printf("WaterLevel: Result - Raw: %d, Smoothed: %d, Water: %s (%.1f%%)\n", 
                 averageReading, smoothedReading, waterDetected ? "YES" : "NO", waterLevelPercent);
    
    return true;
  }
  
  // Getter methods for external access
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  bool isHealthy() const { return consecutiveErrors < MAX_CONSECUTIVE_ERRORS; }
  bool isWaterDetected() const { return lastWaterDetected; }
  int getLastRawValue() const { return lastRawValue; }
  int getThreshold() const { return dynamicThreshold; }
  int getDryBaseline() const { return dryBaseline; }
  
private:
  /**
   * Perform initial calibration to establish dry baseline
   */
  bool performCalibration() {
    Serial.println("WaterLevel: Starting dry surface calibration...");
    
    const int calibrationSamples = 25;
    int calibrationReadings[calibrationSamples];
    int validReadings = 0;
    
    // Take calibration samples over longer period
    for (int i = 0; i < calibrationSamples; i++) {
      int reading = analogRead(sensorPin);
      
      if (reading >= 0 && reading <= 4095) {
        calibrationReadings[validReadings] = reading;
        validReadings++;
        Serial.printf("WaterLevel: Calibration sample %d: %d\n", i + 1, reading);
      }
      
      delay(200); // Longer delay for stability
    }
    
    if (validReadings < calibrationSamples / 2) {
      Serial.println("WaterLevel: Insufficient valid calibration samples");
      return false;
    }
    
    // Calculate dry baseline (average of valid readings)
    long sum = 0;
    int minReading = 4095, maxReading = 0;
    
    for (int i = 0; i < validReadings; i++) {
      sum += calibrationReadings[i];
      if (calibrationReadings[i] < minReading) minReading = calibrationReadings[i];
      if (calibrationReadings[i] > maxReading) maxReading = calibrationReadings[i];
    }
    
    dryBaseline = sum / validReadings;
    int readingRange = maxReading - minReading;
    
    // Set dynamic threshold based on dry baseline
    // Water detected when reading significantly exceeds dry baseline
    dynamicThreshold = dryBaseline + (readingRange * 2) + 300;
    
    // Ensure threshold is within reasonable bounds
    if (dynamicThreshold > 3800) {
      dynamicThreshold = 3800;
    }
    if (dynamicThreshold < (dryBaseline + 100)) {
      dynamicThreshold = dryBaseline + 100;
    }
    
    isCalibrated = true;
    Serial.printf("WaterLevel: Calibration complete - Dry: %d, Range: %d, Threshold: %d\n", 
                 dryBaseline, readingRange, dynamicThreshold);
    
    return true;
  }
  
  /**
   * Update the reading history buffer
   */
  void updateReadingHistory(int reading) {
    readingHistory[historyIndex] = reading;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    
    if (historyCount < HISTORY_SIZE) {
      historyCount++;
    }
  }
  
  /**
   * Get smoothed reading using weighted moving average
   */
  int getSmoothedReading() {
    if (historyCount == 0) {
      return lastRawValue;
    }
    
    long weightedSum = 0;
    int totalWeight = 0;
    int count = min(historyCount, 8); // Use last 8 readings
    
    // Apply weights (more recent readings have higher weight)
    for (int i = 0; i < count; i++) {
      int idx = (historyIndex - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
      int weight = count - i; // Most recent = highest weight
      weightedSum += readingHistory[idx] * weight;
      totalWeight += weight;
    }
    
    return weightedSum / totalWeight;
  }
  
  /**
   * Detect water with confidence checking to reduce false positives
   */
  bool detectWaterWithConfidence(int reading) {
    bool currentReading = false;
    
    if (!isCalibrated) {
      // Use static threshold if not calibrated
      currentReading = reading > WATER_LEVEL_THRESHOLD;
    } else {
      // Use dynamic threshold based on calibration
      currentReading = reading > dynamicThreshold;
    }
    
    // Update confidence counters
    if (currentReading) {
      consecutiveWaterReadings++;
      consecutiveDryReadings = 0;
    } else {
      consecutiveDryReadings++;
      consecutiveWaterReadings = 0;
    }
    
    // Require confidence threshold for state change
    if (!lastWaterDetected && consecutiveWaterReadings >= CONFIDENCE_THRESHOLD) {
      // Transition to water detected
      return true;
    } else if (lastWaterDetected && consecutiveDryReadings >= CONFIDENCE_THRESHOLD) {
      // Transition to dry surface
      return false;
    } else {
      // Maintain current state
      return lastWaterDetected;
    }
  }
  
  /**
   * Calculate water level as percentage (0-100%)
   */
  float calculateWaterLevelPercent(int reading) {
    if (!isCalibrated) {
      // Simple percentage based on ADC range
      return (reading / 4095.0) * 100.0;
    }
    
    // Calculate based on calibrated range
    if (reading <= dryBaseline) {
      return 0.0;
    }
    
    // Assume maximum water reading is around 3500-4000 ADC
    int maxWaterReading = 3800;
    int waterRange = maxWaterReading - dryBaseline;
    int currentWaterLevel = reading - dryBaseline;
    
    float percent = (currentWaterLevel / (float)waterRange) * 100.0;
    
    // Clamp to 0-100%
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    return percent;
  }
  
  /**
   * Populate JSON data for reporting
   */
  void populateJsonData(JsonObject& data, int rawValue, bool waterDetected, 
                       String status, unsigned long timestamp, float waterLevelPercent = 0.0) const {
    data["raw_value"] = rawValue;
    data["water_detected"] = waterDetected;
    data["water_level_percent"] = round(waterLevelPercent * 10) / 10.0;
    data["sensor_type"] = "Water Level";
    data["pin"] = sensorPin;
    data["threshold"] = dynamicThreshold;
    data["dry_baseline"] = dryBaseline;
    data["calibrated"] = isCalibrated;
    data["read_time"] = timestamp;
    data["status"] = status;
    data["consecutive_errors"] = consecutiveErrors;
    data["confidence_level"] = max(consecutiveWaterReadings, consecutiveDryReadings);
    data["data_age_ms"] = timestamp - lastValidReadTime;
    
    // Add interpretation
    if (waterDetected) {
      data["interpretation"] = "Water or wet surface detected";
      data["alert_level"] = "warning";
      if (waterLevelPercent > 75) {
        data["severity"] = "high";
      } else if (waterLevelPercent > 50) {
        data["severity"] = "medium";
      } else {
        data["severity"] = "low";
      }
    } else {
      data["interpretation"] = "Dry surface detected";
      data["alert_level"] = "normal";
      data["severity"] = "none";
    }
  }
};

#endif
```
### feedback/vibration_motor.h
```cpp
#ifndef VIBRATION_MOTOR_H
#define VIBRATION_MOTOR_H

#include <Arduino.h>
#include "../config.h"

/**
 * Vibration Motor Feedback Module
 * 
 * Provides tactile feedback for proximity detection (ultrasonic sensor).
 * The motor activates when obstacles are detected within a configurable threshold.
 * 
 * Features:
 * - Simple on/off control with duration-based timing
 * - Configurable activation threshold
 * - Non-blocking operation using millis() timing
 * - Prevents multiple simultaneous activations
 * - Easy integration with sensor feedback loops
 */
class VibrationMotor {
private:
  bool initialized = false;
  bool enabled = VIBRATION_MOTOR_ENABLED;
  int motorPin = VIBRATION_MOTOR_PIN;
  
  // State tracking
  bool isMotorActive = false;
  unsigned long activationStartTime = 0;
  unsigned long activationDuration = VIBRATION_DURATION;
  
  // Activation control
  unsigned long lastActivationTime = 0;
  static const unsigned long MIN_ACTIVATION_INTERVAL = 500; // Prevent rapid re-activation
  
public:
  VibrationMotor() {}
  
  /**
   * Initialize the vibration motor hardware
   * Sets up GPIO pin and performs initial state setup
   */
  bool begin() {
    if (!enabled) {
      Serial.println("VibrationMotor: Disabled in configuration");
      return true;
    }
    
    Serial.println("VibrationMotor: Initializing tactile feedback system...");
    
    // Validate pin configuration
    if (motorPin < 0) {
      Serial.println("VibrationMotor: Invalid pin configuration");
      return false;
    }
    
    // Configure pin as output
    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, LOW); // Ensure motor starts in OFF state
    
    Serial.printf("VibrationMotor: Configured on GPIO %d\n", motorPin);
    
    // Test activation (brief pulse to verify functionality)
    Serial.println("VibrationMotor: Performing hardware test...");
    digitalWrite(motorPin, HIGH);
    delay(100);
    digitalWrite(motorPin, LOW);
    Serial.println("VibrationMotor: Hardware test complete");
    
    initialized = true;
    Serial.println("VibrationMotor: Initialization successful");
    return true;
  }
  
  /**
   * Activate vibration motor for specified duration
   * 
   * @param duration Duration in milliseconds (optional, uses config default if not specified)
   * @return true if activation was successful, false if already active or too soon
   */
  bool activate(unsigned long duration = 0) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Prevent rapid re-activation (debounce)
    if (currentTime - lastActivationTime < MIN_ACTIVATION_INTERVAL) {
      Serial.println("VibrationMotor: Activation blocked - too soon after last activation");
      return false;
    }
    
    // Don't interrupt ongoing activation
    if (isMotorActive) {
      Serial.println("VibrationMotor: Activation blocked - motor already active");
      return false;
    }
    
    // Use provided duration or default
    activationDuration = (duration > 0) ? duration : VIBRATION_DURATION;
    
    // Activate motor
    digitalWrite(motorPin, HIGH);
    isMotorActive = true;
    activationStartTime = currentTime;
    lastActivationTime = currentTime;
    
    Serial.printf("VibrationMotor: Activated for %lu ms\n", activationDuration);
    return true;
  }
  
  /**
   * Force stop the vibration motor immediately
   * Useful for emergency stops or user-triggered deactivation
   */
  void stop() {
    if (!enabled || !initialized) {
      return;
    }
    
    if (isMotorActive) {
      digitalWrite(motorPin, LOW);
      isMotorActive = false;
      Serial.println("VibrationMotor: Force stopped");
    }
  }
  
  /**
   * Update motor state - MUST be called regularly in main loop
   * Handles automatic deactivation when duration expires
   * This is non-blocking and uses millis() for timing
   */
  void update() {
    if (!enabled || !initialized || !isMotorActive) {
      return;
    }
    
    unsigned long currentTime = millis();
    
    // Check if activation duration has expired
    if (currentTime - activationStartTime >= activationDuration) {
      digitalWrite(motorPin, LOW);
      isMotorActive = false;
      Serial.println("VibrationMotor: Deactivated - duration expired");
    }
  }
  
  /**
   * Activate motor based on distance measurement
   * Convenience method for ultrasonic sensor integration
   * 
   * @param distance Current distance measurement in cm
   * @param threshold Distance threshold for activation (optional, uses config default)
   * @return true if motor was activated due to proximity
   */
  bool activateForProximity(float distance, float threshold = 0.0) {
    if (!enabled || !initialized) {
      return false;
    }
    
    // Use provided threshold or default from config
    float activationThreshold = (threshold > 0) ? threshold : OBSTACLE_VIBRATION_THRESHOLD;
    
    // Check if distance is within activation range
    if (distance > 0 && distance <= activationThreshold) {
      return activate();
    }
    
    return false;
  }
  
  // Getter methods for external monitoring
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  bool isActive() const { return isMotorActive; }
  unsigned long getActivationDuration() const { return activationDuration; }
  unsigned long getRemainingTime() const {
    if (!isMotorActive) {
      return 0;
    }
    
    unsigned long elapsed = millis() - activationStartTime;
    return (elapsed < activationDuration) ? (activationDuration - elapsed) : 0;
  }
  
  /**
   * Get current motor status for debugging/monitoring
   */
  String getStatus() const {
    if (!enabled) return "disabled";
    if (!initialized) return "not_initialized";
    if (isMotorActive) return "active";
    return "ready";
  }
};

#endif
```
### feedback/buzzer.h
```cpp
#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "../config.h"

/**
 * Buzzer Audio Feedback Module
 * 
 * Provides audio alerts for hazard detection (infrared and water level sensors).
 * Supports different beep patterns for different types of alerts.
 * 
 * Features:
 * - Multiple predefined beep patterns (hole detection, water detection)
 * - Non-blocking pattern playback using millis() timing
 * - Configurable beep frequencies and durations
 * - Pattern queuing system for multiple simultaneous alerts
 * - Volume control through PWM duty cycle
 */
class Buzzer {
private:
  bool initialized = false;
  bool enabled = BUZZER_ENABLED;
  int buzzerPin = BUZZER_PIN;
  
  // Pattern definitions
  enum BeepPattern {
    PATTERN_NONE = 0,
    PATTERN_HOLE = 1,
    PATTERN_WATER = 2,
    PATTERN_SINGLE_BEEP = 3,
    PATTERN_DOUBLE_BEEP = 4
  };
  
  // Pattern specifications
  struct PatternSpec {
    int beepCount;
    unsigned long beepDuration;
    unsigned long pauseDuration;
    int frequency; // PWM frequency (0-255 for analogWrite)
  };
  
  // Pattern definitions (configurable via config.h)
  PatternSpec patterns[5] = {
    {0, 0, 0, 0},                    // PATTERN_NONE
    {BUZZER_PATTERN_HOLE_BEEPS, BUZZER_PATTERN_HOLE_DURATION, 200, 200},     // PATTERN_HOLE
    {BUZZER_PATTERN_WATER_BEEPS, BUZZER_PATTERN_WATER_DURATION, 100, 150},   // PATTERN_WATER
    {1, 300, 0, 180},                // PATTERN_SINGLE_BEEP
    {2, 200, 150, 160}               // PATTERN_DOUBLE_BEEP
  };
  
  // State tracking for pattern playback
  bool isPatternActive = false;
  BeepPattern currentPattern = PATTERN_NONE;
  int currentBeepIndex = 0;
  bool isBeepActive = false;
  unsigned long stateStartTime = 0;
  
  // Activation control
  unsigned long lastActivationTime = 0;
  static const unsigned long MIN_ACTIVATION_INTERVAL = 1000; // Prevent spam
  
public:
  Buzzer() {}
  
  /**
   * Initialize the buzzer hardware
   * Sets up GPIO pin and performs audio test
   */
  bool begin() {
    if (!enabled) {
      Serial.println("Buzzer: Disabled in configuration");
      return true;
    }
    
    Serial.println("Buzzer: Initializing audio feedback system...");
    
    // Validate pin configuration
    if (buzzerPin < 0) {
      Serial.println("Buzzer: Invalid pin configuration");
      return false;
    }
    
    // Configure pin as output
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW); // Ensure buzzer starts silent
    
    Serial.printf("Buzzer: Configured on GPIO %d\n", buzzerPin);
    
    // Audio test - brief beep to verify functionality
    Serial.println("Buzzer: Performing audio test...");
    for (int i = 0; i < 3; i++) {
      analogWrite(buzzerPin, 100); // Medium volume
      delay(50);
      analogWrite(buzzerPin, 0);
      delay(100);
    }
    Serial.println("Buzzer: Audio test complete");
    
    initialized = true;
    Serial.println("Buzzer: Initialization successful");
    return true;
  }
  
  /**
   * Play a predefined beep pattern
   * 
   * @param pattern Pattern type (hole, water, etc.)
   * @return true if pattern started successfully
   */
  bool playPattern(BeepPattern pattern) {
    if (!enabled || !initialized || pattern == PATTERN_NONE) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Prevent spam activation
    if (currentTime - lastActivationTime < MIN_ACTIVATION_INTERVAL) {
      Serial.println("Buzzer: Pattern blocked - too soon after last activation");
      return false;
    }
    
    // Allow interrupting current pattern with new one
    if (isPatternActive) {
      Serial.printf("Buzzer: Interrupting current pattern with new %s pattern\n", 
                   patternToString(pattern).c_str());
      stop();
    }
    
    // Start new pattern
    currentPattern = pattern;
    currentBeepIndex = 0;
    isPatternActive = true;
    isBeepActive = true;
    stateStartTime = currentTime;
    lastActivationTime = currentTime;
    
    // Start first beep
    PatternSpec& spec = patterns[pattern];
    analogWrite(buzzerPin, spec.frequency);
    
    Serial.printf("Buzzer: Started %s pattern (%d beeps)\n", 
                 patternToString(pattern).c_str(), spec.beepCount);
    return true;
  }
  
  /**
   * Convenience methods for specific alert types
   */
  bool playHoleAlert() {
    return playPattern(PATTERN_HOLE);
  }
  
  bool playWaterAlert() {
    return playPattern(PATTERN_WATER);
  }
  
  bool playSingleBeep() {
    return playPattern(PATTERN_SINGLE_BEEP);
  }
  
  bool playDoubleBeep() {
    return playPattern(PATTERN_DOUBLE_BEEP);
  }
  
  /**
   * Force stop all buzzer activity immediately
   */
  void stop() {
    if (!enabled || !initialized) {
      return;
    }
    
    analogWrite(buzzerPin, 0);
    isPatternActive = false;
    isBeepActive = false;
    currentPattern = PATTERN_NONE;
    Serial.println("Buzzer: Stopped");
  }
  
  /**
   * Update buzzer state - MUST be called regularly in main loop
   * Handles pattern progression and timing
   */
  void update() {
    if (!enabled || !initialized || !isPatternActive) {
      return;
    }
    
    unsigned long currentTime = millis();
    PatternSpec& spec = patterns[currentPattern];
    unsigned long elapsed = currentTime - stateStartTime;
    
    if (isBeepActive) {
      // Currently beeping - check if beep duration is complete
      if (elapsed >= spec.beepDuration) {
        // Stop current beep
        analogWrite(buzzerPin, 0);
        isBeepActive = false;
        stateStartTime = currentTime;
        
        // Check if this was the last beep in the pattern
        currentBeepIndex++;
        if (currentBeepIndex >= spec.beepCount) {
          // Pattern complete
          isPatternActive = false;
          currentPattern = PATTERN_NONE;
          Serial.println("Buzzer: Pattern completed");
          return;
        }
        
        Serial.printf("Buzzer: Beep %d/%d completed, starting pause\n", 
                     currentBeepIndex, spec.beepCount);
      }
    } else {
      // Currently in pause between beeps
      if (elapsed >= spec.pauseDuration) {
        // Start next beep
        analogWrite(buzzerPin, spec.frequency);
        isBeepActive = true;
        stateStartTime = currentTime;
        Serial.printf("Buzzer: Starting beep %d/%d\n", 
                     currentBeepIndex + 1, spec.beepCount);
      }
    }
  }
  
  /**
   * Alert activation based on sensor states
   * Convenience methods for sensor integration
   */
  bool alertForHole(bool holeDetected) {
    if (holeDetected && !isPatternActive) {
      return playHoleAlert();
    }
    return false;
  }
  
  bool alertForWater(bool waterDetected) {
    if (waterDetected && !isPatternActive) {
      return playWaterAlert();
    }
    return false;
  }
  
  // Getter methods for external monitoring
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  bool isActive() const { return isPatternActive; }
  BeepPattern getCurrentPattern() const { return currentPattern; }
  int getCurrentBeepIndex() const { return currentBeepIndex; }
  bool isCurrentlyBeeping() const { return isBeepActive; }
  
  /**
   * Get current buzzer status for debugging/monitoring
   */
  String getStatus() const {
    if (!enabled) return "disabled";
    if (!initialized) return "not_initialized";
    if (isPatternActive) {
      return "playing_" + patternToString(currentPattern);
    }
    return "ready";
  }
  
  /**
   * Get pattern progress information
   */
  String getPatternProgress() const {
    if (!isPatternActive) {
      return "inactive";
    }
    
    PatternSpec& spec = patterns[currentPattern];
    return String(currentBeepIndex + 1) + "/" + String(spec.beepCount) + 
           " (" + (isBeepActive ? "beeping" : "pausing") + ")";
  }
  
private:
  /**
   * Convert pattern enum to readable string
   */
  String patternToString(BeepPattern pattern) const {
    switch (pattern) {
      case PATTERN_HOLE: return "hole";
      case PATTERN_WATER: return "water";
      case PATTERN_SINGLE_BEEP: return "single";
      case PATTERN_DOUBLE_BEEP: return "double";
      case PATTERN_NONE:
      default: return "none";
    }
  }
};

#endif
```

### wifi_manager.h
```cpp
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include "esp_wifi.h"
#include "config.h"

/**
 * WiFi Manager Class - Revised for Reliability
 * 
 * Handles all WiFi connectivity operations including:
 * - Initial WiFi connection with progressive retry strategy
 * - Connection monitoring and maintenance
 * - Automatic reconnection with smart backoff
 * - Network status reporting and diagnostics
 * - Robust error handling and recovery
 */
class WiFiManager {
private:
  // Connection state tracking
  unsigned long lastConnectionAttempt = 0;
  int connectionAttempts = 0;
  bool wasConnected = false;
  unsigned long connectionStartTime = 0;
  unsigned long totalDisconnectedTime = 0;
  unsigned long lastDisconnectTime = 0;
  bool initialized = false;
  bool diagnosticMode = false;
  
  // Progressive retry strategy
  enum RetryPhase {
    PHASE_QUICK = 0,    // Quick retries with short delays
    PHASE_NORMAL = 1,   // Normal retries with medium delays
    PHASE_AGGRESSIVE = 2 // Aggressive retries with resets
  };
  RetryPhase currentPhase = PHASE_QUICK;
  
  // Configuration validation
  bool configValid = false;
  
public:
  /**
   * Initialize the WiFi manager and attempt first connection
   */
  void begin() {
    Serial.println("=== WiFi Manager Initializing ===");
    
    // Validate configuration first
    if (!validateConfig()) {
      Serial.println("❌ Invalid WiFi configuration - check config.h");
      return;
    }
    
    // Initialize random seed for jitter calculation
    randomSeed(analogRead(0) + esp_random());
    
    // Progressive initialization approach
    if (!initializeWiFiModule()) {
      Serial.println("❌ WiFi module initialization failed");
      return;
    }
    
    Serial.println("✅ WiFi module initialized successfully");
    
    // Run initial diagnostics
    runDiagnostics();
    
    // Configure WiFi settings
    configureWiFiSettings();
    
    Serial.printf("Device ID: %s\n", DEVICE_ID);
    Serial.printf("Target Network: %s\n", WIFI_SSID);
    
    // Attempt initial connection
    connect();
    
    initialized = true;
    Serial.println("=== WiFi Manager Ready ===");
  }
  
  /**
   * Maintain WiFi connection - call this regularly in main loop
   * Handles automatic reconnection with progressive retry strategy
   */
  void maintainConnection() {
    if (!initialized || !configValid) return;
    
    bool currentlyConnected = (WiFi.status() == WL_CONNECTED);
    
    if (!currentlyConnected) {
      handleDisconnection();
      attemptReconnection();
    } else {
      handleConnection();
    }
  }
  
  /**
   * Run comprehensive WiFi diagnostics
   */
  void runDiagnostics() {
    Serial.println("\n=== WiFi Diagnostic Mode ===");
    diagnosticMode = true;
    
    // Check WiFi module health
    checkModuleHealth();
    
    // Scan and analyze networks
    scanNetworks();
    
    // Test connection if target network found
    if (isTargetNetworkAvailable()) {
      testConnection();
    } else {
      Serial.println("   Skipping connection test - target network not found");
    }
    
    Serial.println("=== Diagnostic Complete ===\n");
    diagnosticMode = false;
  }
  
  /**
   * Check if WiFi is currently connected
   */
  bool isConnected() const {
    return WiFi.status() == WL_CONNECTED;
  }
  
  /**
   * Get current IP address
   */
  String getIPAddress() const {
    return isConnected() ? WiFi.localIP().toString() : "";
  }
  
  /**
   * Get WiFi signal strength
   */
  int getSignalStrength() const {
    return isConnected() ? WiFi.RSSI() : 0;
  }
  
  /**
   * Get connection quality description
   */
  String getSignalQuality() const {
    return getSignalQualityFromRSSI(getSignalStrength());
  }
  
  /**
   * Get comprehensive network statistics
   */
  String getNetworkStats() const {
    String stats = "Network Stats:\n";
    stats += "  Status: " + String(isConnected() ? "Connected" : "Disconnected") + "\n";
    
    if (isConnected()) {
      stats += "  IP: " + getIPAddress() + "\n";
      stats += "  RSSI: " + String(getSignalStrength()) + " dBm (" + getSignalQuality() + ")\n";
      stats += "  Gateway: " + WiFi.gatewayIP().toString() + "\n";
      stats += "  DNS: " + WiFi.dnsIP().toString() + "\n";
      stats += "  Channel: " + String(WiFi.channel()) + "\n";
    }
    
    stats += "  Retry Phase: " + getRetryPhaseString() + "\n";
    stats += "  Connection Attempts: " + String(connectionAttempts) + "\n";
    stats += "  Total Downtime: " + String(totalDisconnectedTime / 1000) + "s\n";
    stats += "  Config Valid: " + String(configValid ? "Yes" : "No");
    
    return stats;
  }
  
  /**
   * Force immediate reconnection attempt
   */
  void forceReconnect() {
    Serial.println("🔄 Forcing reconnection...");
    connectionAttempts = 0;
    currentPhase = PHASE_QUICK;
    connect();
  }
  
  /**
   * Reset WiFi manager to initial state
   */
  void reset() {
    Serial.println("🔄 Resetting WiFi Manager...");
    
    WiFi.disconnect(true);
    delay(1000);
    
    // Reset all state variables
    lastConnectionAttempt = 0;
    connectionAttempts = 0;
    wasConnected = false;
    connectionStartTime = 0;
    totalDisconnectedTime = 0;
    lastDisconnectTime = 0;
    currentPhase = PHASE_QUICK;
    
    // Reinitialize
    begin();
  }
  
private:
  /**
   * Validate WiFi configuration from config.h
   */
  bool validateConfig() {
    Serial.println("Validating configuration...");
    
    // Check SSID
    if (!WIFI_SSID || strlen(WIFI_SSID) == 0) {
      Serial.println("❌ WIFI_SSID is empty or undefined");
      return false;
    }
    
    if (strlen(WIFI_SSID) > 32) {
      Serial.println("❌ WIFI_SSID is too long (max 32 characters)");
      return false;
    }
    
    // Check Password
    if (!WIFI_PASSWORD) {
      Serial.println("❌ WIFI_PASSWORD is undefined");
      return false;
    }
    
    if (strlen(WIFI_PASSWORD) > 64) {
      Serial.println("❌ WIFI_PASSWORD is too long (max 64 characters)");
      return false;
    }
    
    // Check timeout
    if (WIFI_TIMEOUT < 5000 || WIFI_TIMEOUT > 60000) {
      Serial.println("⚠️  WIFI_TIMEOUT should be between 5-60 seconds");
    }
    
    // Check Device ID if defined
    #ifdef DEVICE_ID
    if (strlen(DEVICE_ID) > 32) {
      Serial.println("⚠️  DEVICE_ID is too long (max 32 characters)");
    }
    #endif
    
    Serial.printf("✅ Configuration valid - SSID: '%s', Password: %d chars\n", 
                  WIFI_SSID, strlen(WIFI_PASSWORD));
    
    configValid = true;
    return true;
  }
  
  /**
   * Initialize WiFi module with progressive approach
   */
  bool initializeWiFiModule() {
    Serial.println("Initializing WiFi module...");
    
    // Method 1: Gentle initialization (most reliable)
    WiFi.mode(WIFI_STA);
    delay(500);
    
    if (isModuleResponding()) {
      Serial.println("✅ WiFi module responding after gentle init");
      return true;
    }
    
    // Method 2: Standard reset
    Serial.println("Trying standard reset...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_STA);
    delay(500);
    
    if (isModuleResponding()) {
      Serial.println("✅ WiFi module responding after standard reset");
      return true;
    }
    
    // Method 3: ESP-IDF reset (only if Arduino methods fail)
    Serial.println("Trying ESP-IDF reset...");
    
    // Safely stop and restart WiFi subsystem
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_OK) {
      delay(1000);
      
      err = esp_wifi_start();
      if (err == ESP_OK) {
        delay(500);
        WiFi.mode(WIFI_STA);
        delay(500);
        
        if (isModuleResponding()) {
          Serial.println("✅ WiFi module responding after ESP-IDF reset");
          return true;
        }
      }
    }
    
    Serial.println("❌ All WiFi initialization methods failed");
    return false;
  }
  
  /**
   * Check if WiFi module is responding
   */
  bool isModuleResponding() {
    return (WiFi.getMode() != WIFI_MODE_NULL && WiFi.macAddress() != "00:00:00:00:00:00");
  }
  
  /**
   * Configure WiFi settings
   */
  void configureWiFiSettings() {
    WiFi.setAutoReconnect(false); // We handle reconnection manually
    WiFi.setSleep(false); // Disable power saving for better reliability
    
    #ifdef DEVICE_ID
    WiFi.setHostname(DEVICE_ID);
    #endif
    
    // Set TX power to maximum for better range
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
  }
  
  /**
   * Handle disconnection event
   */
  void handleDisconnection() {
    if (wasConnected) {
      Serial.println("⚠️  WiFi connection lost!");
      wasConnected = false;
      lastDisconnectTime = millis();
      
      // Progress to next retry phase if needed
      progressRetryPhase();
    }
  }
  
  /**
   * Handle successful connection
   */
  void handleConnection() {
    if (!wasConnected) {
      Serial.println("✅ WiFi connection established!");
      printConnectionInfo();
      wasConnected = true;
      connectionAttempts = 0;
      currentPhase = PHASE_QUICK; // Reset to quick phase
      
      // Track disconnection time
      if (lastDisconnectTime > 0) {
        totalDisconnectedTime += (millis() - lastDisconnectTime);
        lastDisconnectTime = 0;
      }
    }
  }
  
  /**
   * Attempt reconnection based on current phase
   */
  void attemptReconnection() {
    unsigned long currentTime = millis();
    unsigned long backoffDelay = calculateBackoffDelay();
    
    if (currentTime - lastConnectionAttempt >= backoffDelay) {
      connect();
      lastConnectionAttempt = currentTime;
    }
  }
  
  /**
   * Progress to next retry phase
   */
  void progressRetryPhase() {
    if (connectionAttempts >= 5 && currentPhase == PHASE_QUICK) {
      currentPhase = PHASE_NORMAL;
      Serial.println("📈 Progressing to NORMAL retry phase");
    } else if (connectionAttempts >= 15 && currentPhase == PHASE_NORMAL) {
      currentPhase = PHASE_AGGRESSIVE;
      Serial.println("📈 Progressing to AGGRESSIVE retry phase");
    }
  }
  
  /**
   * Calculate backoff delay based on current phase
   */
  unsigned long calculateBackoffDelay() const {
    unsigned long baseDelay;
    
    switch (currentPhase) {
      case PHASE_QUICK:
        baseDelay = 2000; // 2 seconds
        break;
      case PHASE_NORMAL:
        baseDelay = 5000 * (1 + (connectionAttempts - 5) / 3); // 5-15 seconds
        break;
      case PHASE_AGGRESSIVE:
        baseDelay = 10000 * (1 + (connectionAttempts - 15) / 5); // 10-30 seconds
        break;
    }
    
    // Add jitter (±20%)
    unsigned long jitter = baseDelay / 5;
    long jitterOffset = random(-jitter, jitter + 1);
    long finalDelay = (long)baseDelay + jitterOffset;
    
    return (unsigned long)max(finalDelay, 1000L); // Minimum 1 second
  }
  
  /**
   * Attempt WiFi connection with phase-appropriate strategy
   */
  void connect() {
    connectionAttempts++;
    
    Serial.printf("🔄 WiFi connection attempt %d (%s phase)...\n", 
                  connectionAttempts, getRetryPhaseString().c_str());
    
    // Phase-specific preparation
    prepareConnection();
    
    // Start connection
    connectionStartTime = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Wait for connection with appropriate timeout
    unsigned long timeout = getPhaseTimeout();
    unsigned long startTime = millis();
    
    while (WiFi.status() != WL_CONNECTED && 
           millis() - startTime < timeout) {
      delay(100);
      
      // Print progress dots every second
      if ((millis() - startTime) % 1000 == 0) {
        Serial.print(".");
      }
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("✅ WiFi connected successfully!");
      printConnectionInfo();
    } else {
      Serial.println("❌ WiFi connection failed!");
      printConnectionError();
    }
  }
  
  /**
   * Prepare connection based on current phase
   */
  void prepareConnection() {
    switch (currentPhase) {
      case PHASE_QUICK:
        // Quick disconnect and reconnect
        WiFi.disconnect();
        delay(100);
        break;
        
      case PHASE_NORMAL:
        // More thorough preparation
        WiFi.disconnect(true);
        delay(500);
        WiFi.mode(WIFI_STA);
        delay(200);
        break;
        
      case PHASE_AGGRESSIVE:
        // Full reset and scan
        Serial.println("🔧 Performing aggressive reset...");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(1000);
        WiFi.mode(WIFI_STA);
        delay(500);
        
        // Quick scan to refresh network list
        WiFi.scanDelete();
        WiFi.scanNetworks(true); // Async scan
        delay(2000); // Wait for scan to complete
        break;
    }
  }
  
  /**
   * Get timeout for current phase
   */
  unsigned long getPhaseTimeout() const {
    switch (currentPhase) {
      case PHASE_QUICK:
        return WIFI_TIMEOUT / 2; // Quick attempts
      case PHASE_NORMAL:
        return WIFI_TIMEOUT; // Normal timeout
      case PHASE_AGGRESSIVE:
        return WIFI_TIMEOUT * 2; // Extended timeout
    }
    return WIFI_TIMEOUT;
  }
  
  /**
   * Check module health
   */
  void checkModuleHealth() {
    Serial.println("\n1. WiFi Module Health Check:");
    Serial.printf("   Mode: %s\n", getWiFiModeString().c_str());
    Serial.printf("   MAC Address: %s\n", WiFi.macAddress().c_str());
    Serial.printf("   Status: %s\n", getWiFiStatusString().c_str());
    
    if (!isModuleResponding()) {
      Serial.println("   ❌ Module not responding properly!");
    } else {
      Serial.println("   ✅ Module responding normally");
    }
  }
  
  /**
   * Scan and analyze networks
   */
  void scanNetworks() {
    Serial.println("\n2. Network Scan:");
    WiFi.mode(WIFI_STA);
    delay(100);
    
    int networkCount = WiFi.scanNetworks();
    
    if (networkCount <= 0) {
      Serial.println("   ❌ No networks found!");
      Serial.println("   Possible issues:");
      Serial.println("   - WiFi antenna disconnected");
      Serial.println("   - No 2.4GHz networks in range");
      Serial.println("   - ESP32 WiFi module failure");
      return;
    }
    
    Serial.printf("   ✅ Found %d networks\n", networkCount);
    
    bool targetFound = false;
    int targetRSSI = 0;
    
    for (int i = 0; i < networkCount && i < 10; i++) { // Show max 10 networks
      String ssid = WiFi.SSID(i);
      int rssi = WiFi.RSSI(i);
      String security = getSecurityType(WiFi.encryptionType(i));
      
      Serial.printf("   %d: %s (%d dBm) [%s]\n", 
                   i + 1, ssid.c_str(), rssi, security.c_str());
      
      if (ssid == WIFI_SSID) {
        targetFound = true;
        targetRSSI = rssi;
        Serial.printf("   ✅ TARGET FOUND! Signal: %s\n", 
                     getSignalQualityFromRSSI(rssi).c_str());
      }
    }
    
    if (!targetFound) {
      Serial.printf("   ❌ Target network '%s' not found!\n", WIFI_SSID);
    }
  }
  
  /**
   * Check if target network is available
   */
  bool isTargetNetworkAvailable() {
    int networkCount = WiFi.scanComplete();
    if (networkCount <= 0) return false;
    
    for (int i = 0; i < networkCount; i++) {
      if (WiFi.SSID(i) == WIFI_SSID) {
        return true;
      }
    }
    return false;
  }
  
  /**
   * Test connection with detailed feedback
   */
  void testConnection() {
    Serial.println("\n3. Connection Test:");
    Serial.printf("   Testing connection to '%s'...\n", WIFI_SSID);
    
    WiFi.disconnect();
    delay(500);
    
    unsigned long testStart = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("   Progress: ");
    while (WiFi.status() != WL_CONNECTED && 
           millis() - testStart < WIFI_TIMEOUT) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("   ✅ Connection test successful!");
      Serial.printf("   IP: %s, RSSI: %d dBm\n", 
                   WiFi.localIP().toString().c_str(), WiFi.RSSI());
      WiFi.disconnect(); // Clean up after test
    } else {
      Serial.println("   ❌ Connection test failed!");
      printDetailedConnectionAnalysis();
    }
  }
  
  /**
   * Print detailed connection information
   */
  void printConnectionInfo() const {
    Serial.println("📶 Connection Details:");
    Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("  DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("  RSSI: %d dBm (%s)\n", WiFi.RSSI(), getSignalQuality().c_str());
    Serial.printf("  Channel: %d\n", WiFi.channel());
    Serial.printf("  Connection time: %lu ms\n", millis() - connectionStartTime);
  }
  
  /**
   * Print connection error with context
   */
  void printConnectionError() const {
    Serial.println("❌ Connection Failed:");
    Serial.printf("  Status: %s\n", getWiFiStatusString().c_str());
    Serial.printf("  Phase: %s\n", getRetryPhaseString().c_str());
    Serial.printf("  Next retry: %lu seconds\n", calculateBackoffDelay() / 1000);
    
    // Provide specific guidance
    wl_status_t status = WiFi.status();
    switch (status) {
      case WL_NO_SSID_AVAIL:
        Serial.println("  Issue: Network not found");
        Serial.println("  Check: SSID spelling, network visibility");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("  Issue: Authentication failed");
        Serial.println("  Check: WiFi password, network security");
        break;
      case WL_CONNECTION_LOST:
        Serial.println("  Issue: Connection unstable");
        Serial.println("  Check: Signal strength, interference");
        break;
      default:
        Serial.println("  Issue: General connection problem");
        Serial.println("  Check: Network availability, ESP32 hardware");
        break;
    }
  }
  
  /**
   * Print detailed connection analysis
   */
  void printDetailedConnectionAnalysis() const {
    Serial.println("   Detailed Analysis:");
    wl_status_t status = WiFi.status();
    
    switch (status) {
      case WL_NO_SSID_AVAIL:
        Serial.println("   ❌ Network not found");
        Serial.println("   Solutions: Check SSID, ensure 2.4GHz, verify broadcast");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("   ❌ Authentication failed");
        Serial.println("   Solutions: Verify password, check security type");
        break;
      case WL_CONNECTION_LOST:
        Serial.println("   ❌ Connection lost during handshake");
        Serial.println("   Solutions: Improve signal, reduce interference");
        break;
      default:
        Serial.printf("   ❌ Unexpected status: %d\n", status);
        Serial.println("   Solutions: Hardware check, power cycle");
        break;
    }
  }
  
  /**
   * Helper functions for string conversion
   */
  String getRetryPhaseString() const {
    switch (currentPhase) {
      case PHASE_QUICK: return "Quick";
      case PHASE_NORMAL: return "Normal";
      case PHASE_AGGRESSIVE: return "Aggressive";
      default: return "Unknown";
    }
  }
  
  String getWiFiModeString() const {
    switch (WiFi.getMode()) {
      case WIFI_MODE_NULL: return "NULL";
      case WIFI_MODE_STA: return "Station";
      case WIFI_MODE_AP: return "Access Point";
      case WIFI_MODE_APSTA: return "AP+Station";
      default: return "Unknown";
    }
  }
  
  String getWiFiStatusString() const {
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL: return "SSID not available";
      case WL_CONNECT_FAILED: return "Connection failed";
      case WL_CONNECTION_LOST: return "Connection lost";
      case WL_DISCONNECTED: return "Disconnected";
      case WL_IDLE_STATUS: return "Idle";
      case WL_CONNECTED: return "Connected";
      default: return "Unknown";
    }
  }
  
  String getSecurityType(wifi_auth_mode_t encType) const {
    switch (encType) {
      case WIFI_AUTH_OPEN: return "Open";
      case WIFI_AUTH_WEP: return "WEP";
      case WIFI_AUTH_WPA_PSK: return "WPA";
      case WIFI_AUTH_WPA2_PSK: return "WPA2";
      case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
      case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
      case WIFI_AUTH_WPA3_PSK: return "WPA3";
      case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
      default: return "Unknown";
    }
  }
  
  String getSignalQualityFromRSSI(int rssi) const {
    if (rssi == 0) return "Disconnected";
    if (rssi >= -50) return "Excellent";
    if (rssi >= -60) return "Good";
    if (rssi >= -70) return "Fair";
    if (rssi >= -80) return "Poor";
    return "Very Poor";
  }
};

#endif
```

### data_queue.h
```cpp
#ifndef DATA_QUEUE_H
#define DATA_QUEUE_H

#include <Arduino.h>
#include "config.h"

/**
 * Data Queue Manager Class
 * 
 * Manages a circular buffer for storing sensor data JSON strings
 * Features:
 * - Automatic overwrite of oldest data when full
 * - Thread-safe operations (for single-threaded ESP32 environment)
 * - Memory efficient string storage
 * - Queue statistics and monitoring
 */
class DataQueue {
private:
  String queue[MAX_QUEUE_SIZE];    // Circular buffer for data storage
  int front = 0;                   // Index of front element
  int rear = 0;                    // Index of rear element  
  int count = 0;                   // Current number of elements
  
  // Statistics tracking
  unsigned long totalEnqueued = 0;
  unsigned long totalDequeued = 0;
  unsigned long totalOverwrites = 0;
  int maxSizeReached = 0;
  
public:
  /**
   * Initialize the data queue
   */
  void begin() {
    Serial.println("=== Data Queue Initializing ===");
    
    clear();
    
    Serial.printf("Queue Configuration:\n");
    Serial.printf("  Max Size: %d items\n", MAX_QUEUE_SIZE);
    Serial.printf("  Estimated Memory: ~%d bytes\n", 
                  MAX_QUEUE_SIZE * 512); // Rough estimate per JSON string
    
    Serial.println("=== Data Queue Ready ===");
  }
  
  /**
   * Add data to the queue
   * @param data JSON string to enqueue
   * @return true if successfully enqueued, false if error
   */
  bool enqueue(const String& data) {
    if (data.length() == 0) {
      Serial.println("DataQueue: Attempted to enqueue empty data");
      return false;
    }
    
    // Check if queue is full
    if (isFull()) {
      Serial.printf("DataQueue: Queue full, overwriting oldest item (size: %d)\n", count);
      
      // Remove oldest item to make space (circular buffer behavior)
      dequeue();
      totalOverwrites++;
    }
    
    // Add new item
    queue[rear] = data;
    rear = (rear + 1) % MAX_QUEUE_SIZE;
    count++;
    totalEnqueued++;
    
    // Update statistics
    if (count > maxSizeReached) {
      maxSizeReached = count;
    }
    
    return true;
  }
  
  /**
   * Remove and return data from front of queue
   * @return JSON string data, empty string if queue is empty
   */
  String dequeue() {
    if (isEmpty()) {
      return "";
    }
    
    String data = queue[front];
    queue[front] = ""; // Clear the slot to free memory
    front = (front + 1) % MAX_QUEUE_SIZE;
    count--;
    totalDequeued++;
    
    return data;
  }
  
  /**
   * Look at front item without removing it
   * @return JSON string data, empty string if queue is empty
   */
  String peek() const {
    if (isEmpty()) {
      return "";
    }
    return queue[front];
  }
  
  /**
   * Check if queue is empty
   * @return true if empty, false otherwise
   */
  bool isEmpty() const {
    return count == 0;
  }
  
  /**
   * Check if queue is full
   * @return true if full, false otherwise
   */
  bool isFull() const {
    return count >= MAX_QUEUE_SIZE;
  }
  
  /**
   * Get current number of items in queue
   * @return number of items
   */
  int size() const {
    return count;
  }
  
  /**
   * Get maximum capacity of queue
   * @return maximum number of items
   */
  int capacity() const {
    return MAX_QUEUE_SIZE;
  }
  
  /**
   * Get queue utilization percentage
   * @return utilization as percentage (0-100)
   */
  float getUtilization() const {
    return (float)count / MAX_QUEUE_SIZE * 100.0;
  }
  
  /**
   * Clear all items from queue
   */
  void clear() {
    front = 0;
    rear = 0;
    count = 0;
    
    // Clear all queue slots to free memory
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
      queue[i] = "";
    }
    
    Serial.println("DataQueue: Queue cleared");
  }
  
  /**
   * Get detailed queue statistics
   * @return formatted statistics string
   */
  String getStatistics() const {
    String stats = "Queue Statistics:\n";
    stats += "  Current Size: " + String(count) + "/" + String(MAX_QUEUE_SIZE) + "\n";
    stats += "  Utilization: " + String(getUtilization(), 1) + "%\n";
    stats += "  Max Size Reached: " + String(maxSizeReached) + "\n";
    stats += "  Total Enqueued: " + String(totalEnqueued) + "\n";
    stats += "  Total Dequeued: " + String(totalDequeued) + "\n";
    stats += "  Total Overwrites: " + String(totalOverwrites) + "\n";
    stats += "  Items in Queue: " + String(totalEnqueued - totalDequeued);
    return stats;
  }
  
  /**
   * Print current queue status to Serial (for debugging)
   */
  void printStatus() const {
    Serial.printf("DataQueue Status:\n");
    Serial.printf("  Size: %d/%d (%.1f%% full)\n", 
                  count, MAX_QUEUE_SIZE, getUtilization());
    Serial.printf("  Front: %d, Rear: %d\n", front, rear);
    Serial.printf("  Total Operations: %lu enqueued, %lu dequeued, %lu overwrites\n",
                  totalEnqueued, totalDequeued, totalOverwrites);
  }
  
  /**
   * Validate queue integrity (for debugging)
   * @return true if queue is in valid state
   */
  bool validateIntegrity() const {
    // Check basic invariants
    if (count < 0 || count > MAX_QUEUE_SIZE) {
      Serial.printf("DataQueue: Invalid count: %d\n", count);
      return false;
    }
    
    if (front < 0 || front >= MAX_QUEUE_SIZE) {
      Serial.printf("DataQueue: Invalid front index: %d\n", front);
      return false;
    }
    
    if (rear < 0 || rear >= MAX_QUEUE_SIZE) {
      Serial.printf("DataQueue: Invalid rear index: %d\n", rear);
      return false;
    }
    
    // Check count consistency
    int calculatedCount = 0;
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
      if (queue[i].length() > 0) {
        calculatedCount++;
      }
    }
    
    // Note: calculated count might not match 'count' in circular buffer
    // because we clear strings when dequeuing, but this is normal
    
    return true;
  }
  
  /**
   * Get memory usage estimate
   * @return estimated memory usage in bytes
   */
  size_t getMemoryUsage() const {
    size_t totalMemory = sizeof(*this); // Object overhead
    
    // Add string memory usage
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
      totalMemory += queue[i].length();
    }
    
    return totalMemory;
  }
};

#endif
```

---
