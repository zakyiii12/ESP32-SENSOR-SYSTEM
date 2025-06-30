//esp32-sensor-client.ino - FIXED INFRARED BUZZER LOGIC
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
unsigned long lastFeedbackUpdate = 0;

// Enhanced sensor data storage for intelligent feedback
struct SensorReadings {
  // Ultrasonic data
  float distance = -1;
  float confidence = 0.0;
  float signalStability = 0.0;
  String distanceClassification = "unknown";
  bool validUltrasonicReading = false;
  
  // Infrared data - FIXED LOGIC
  bool surfaceDetected = true;  // TRUE = surface present, FALSE = hole/drop detected
  bool validInfraredReading = false;
  
  // Water sensor data - ENHANCED FOR FUZZY LOGIC
  bool waterDetected = false;
  bool validWaterReading = false;
  
  // NEW: Water sensor fuzzy logic output
  struct WaterFuzzyOutput {
    int alertLevel = 0;        // 0-3 (Silent, Gentle, Moderate, Urgent)
    int frequency = 0;         // Hz for buzzer
    String pattern = "silent"; // Beep pattern type
    float confidence = 0.0;    // 0.0-1.0 confidence in decision
    String interpretation = ""; // Human-readable interpretation
    String severity = "none";   // none, low, medium, high
  } waterFuzzyOutput;
  
  // Ultrasonic fuzzy logic output (existing)
  struct FuzzyOutput {
    float vibrationIntensity = 0.0;
    int vibrationPattern = 0;
    float vibrationFrequency = 0.0;
    bool fuzzyActive = false;
  } fuzzyOutput;
  
  // System state
  unsigned long readTime = 0;
  bool systemHealthy = true;
};

SensorReadings currentReadings;
SensorReadings previousReadings;

// Feedback control parameters
const unsigned long FEEDBACK_UPDATE_INTERVAL = 50;  // 20Hz feedback update rate
const unsigned long SENSOR_CONFIDENCE_TIMEOUT = 2000; // 2 seconds for sensor confidence
const float CONFIDENCE_THRESHOLD = 0.3; // Minimum confidence for fuzzy activation

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== ESP32 Enhanced Sensor Client with FIXED Infrared Logic ===");
  
  // Initialize core components
  Serial.println("Initializing core components...");
  wifiManager.begin();
  dataQueue.begin();
  
  // Initialize sensors
  Serial.println("Initializing sensors...");
  bool sensorInitSuccess = true;
  sensorInitSuccess &= ultrasonicSensor.begin();
  sensorInitSuccess &= infraredSensor.begin();
  sensorInitSuccess &= waterLevelSensor.begin();
  
  if (!sensorInitSuccess) {
    Serial.println("WARNING: Some sensors failed to initialize");
  }
  
  // Initialize feedback components
  Serial.println("Initializing feedback components...");
  bool feedbackInitSuccess = true;
  feedbackInitSuccess &= vibrationMotor.begin();
  feedbackInitSuccess &= buzzer.begin();
  
  if (!feedbackInitSuccess) {
    Serial.println("WARNING: Some feedback components failed to initialize");
  }
  
  // Setup time synchronization
  Serial.println("Synchronizing time...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  // ensureTimeSync();
  
  // Status LED setup
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // Initialize sensor readings structure
  initializeSensorReadings();
  
  Serial.println("=== Setup completed successfully! ===");
  Serial.println("Fuzzy Logic Vibration Control: ACTIVE");
  Serial.println("FIXED: Infrared buzzer logic - ACTIVE when NO surface detected");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Maintain WiFi connection
  wifiManager.maintainConnection();
  
  // Read sensors at specified interval
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readAllSensors();
    lastSensorRead = currentTime;
  }
  
  // FIXED: Update feedback systems at high frequency for IMMEDIATE infrared response
  updateIntelligentFeedback();
  
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
  
  // Small delay to prevent watchdog issues
  delay(20); // Reduced from 50ms for better feedback responsiveness
}

void initializeSensorReadings() {
  currentReadings = SensorReadings();
  previousReadings = SensorReadings();
  Serial.println("Sensor readings structure initialized");
}

void readAllSensors() {
  if (!wifiManager.isConnected()) {
    Serial.println("WiFi not connected, skipping sensor read");
    return;
  }
  
  Serial.println("--- Reading All Sensors ---");
  digitalWrite(LED_STATUS_PIN, HIGH);
  
  // Store previous readings for comparison
  previousReadings = currentReadings;
  currentReadings.readTime = millis();
  currentReadings.systemHealthy = true;
  
  // Create JSON document for data queue
  DynamicJsonDocument doc(1536); // Increased size for fuzzy logic data
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = getISOTimestamp();
  
  JsonObject data = doc.createNestedObject("data");
  
  // Read Ultrasonic sensor with fuzzy logic processing
  readUltrasonicWithFuzzyLogic(data);
  
  // FIXED: Read infrared sensor with corrected logic
  readInfraredSensorFixed(data);
  
  // Read water sensor
  readWaterLevelSensor(data);
  
  // Add system information
  addSystemInfo(data);
  
  // Add fuzzy logic performance data
  addFuzzyLogicInfo(data);
  
  // Convert to JSON string and queue
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (dataQueue.enqueue(jsonString)) {
    Serial.printf("✓ Sensor data queued (Queue size: %d)\n", dataQueue.size());
  } else {
    Serial.println("✗ Failed to queue sensor data - queue full");
  }
  
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // Print sensor summary
  printSensorSummary();
  Serial.println();
}

void readUltrasonicWithFuzzyLogic(JsonObject& data) {
  if (!ultrasonicSensor.isEnabled()) {
    currentReadings.validUltrasonicReading = false;
    return;
  }
  
  Serial.println("🔍 Reading Ultrasonic with Fuzzy Logic...");
  
  JsonObject ultrasonicData = data.createNestedObject("ultrasonic");
  
  if (ultrasonicSensor.readSensor(ultrasonicData)) {
    // Extract primary sensor data
    currentReadings.distance = ultrasonicData["distance_cm"];
    currentReadings.confidence = ultrasonicData["confidence"];
    currentReadings.distanceClassification = ultrasonicData["distance_classification"].as<String>();
    currentReadings.validUltrasonicReading = true;
    
    // Extract fuzzy logic output
    if (ultrasonicData.containsKey("fuzzy_logic")) {
      JsonObject fuzzyData = ultrasonicData["fuzzy_logic"];
      currentReadings.fuzzyOutput.vibrationIntensity = fuzzyData["vibration_intensity"];
      currentReadings.fuzzyOutput.vibrationPattern = fuzzyData["vibration_pattern"];
      currentReadings.fuzzyOutput.vibrationFrequency = fuzzyData["vibration_frequency"];
      currentReadings.fuzzyOutput.fuzzyActive = fuzzyData["enabled"];
      currentReadings.signalStability = fuzzyData["signal_stability"];
    }
    
    Serial.printf("✓ Ultrasonic: %.1f cm (conf: %.2f, class: %s)\n", 
                 currentReadings.distance, currentReadings.confidence, 
                 currentReadings.distanceClassification.c_str());
    Serial.printf("  Fuzzy Output: Intensity=%.0f, Pattern=%d, Freq=%.1f Hz\n",
                 currentReadings.fuzzyOutput.vibrationIntensity,
                 currentReadings.fuzzyOutput.vibrationPattern,
                 currentReadings.fuzzyOutput.vibrationFrequency);
  } else {
    Serial.println("✗ Ultrasonic read failed");
    currentReadings.validUltrasonicReading = false;
    currentReadings.systemHealthy = false;
    
    // Use fallback fuzzy inference for failed readings
    handleUltrasonicFailure();
  }
}

// FIXED: New infrared sensor reading with corrected logic
void readInfraredSensorFixed(JsonObject& data) {
  if (!infraredSensor.isEnabled()) {
    currentReadings.validInfraredReading = false;
    return;
  }
  
  JsonObject infraredData = data.createNestedObject("infrared");
  if (infraredSensor.readSensor(infraredData)) {
    // FIXED: Extract surface detection status correctly
    // Assuming infrared sensor returns "hole_detected" field
    bool holeDetected = infraredData["hole_detected"];
    currentReadings.surfaceDetected = !holeDetected;  // Surface detected = NOT hole detected
    currentReadings.validInfraredReading = true;
    
    // FIXED: Clear logging with correct interpretation
    if (!currentReadings.surfaceDetected) {  // No surface = hole/drop detected
      Serial.printf("✓ Infrared: NO SURFACE/HOLE DETECTED (Alert: ACTIVE)\n");
      Serial.printf("  Raw Value: %d, Threshold: %d, Surface: NO\n", 
                   infraredData["raw_value"].as<int>(),
                   infraredData["threshold"].as<int>());
      Serial.printf("  Interpretation: DANGER - No surface detected\n");
    } else {  // Surface present
      Serial.printf("✓ Infrared: Surface detected (Alert: INACTIVE)\n");
      Serial.printf("  Raw Value: %d, Surface: YES\n", 
                   infraredData["raw_value"].as<int>());
      Serial.printf("  Interpretation: SAFE - Surface present\n");
    }
  } else {
    Serial.println("✗ Infrared read failed");
    currentReadings.validInfraredReading = false;
    currentReadings.systemHealthy = false;
  }
}

void readWaterLevelSensor(JsonObject& data) {
  if (!waterLevelSensor.isEnabled()) {
    currentReadings.validWaterReading = false;
    return;
  }
  
  JsonObject waterData = data.createNestedObject("water_level");
  if (waterLevelSensor.readSensor(waterData)) {
    // Extract basic water detection (for backward compatibility)
    currentReadings.waterDetected = waterData["water_detected"];
    currentReadings.validWaterReading = true;
    
    // NEW: Extract fuzzy logic output from water sensor
    if (waterData.containsKey("fuzzy_alert_level")) {
      currentReadings.waterFuzzyOutput.alertLevel = waterData["fuzzy_alert_level"];
      currentReadings.waterFuzzyOutput.frequency = waterData["fuzzy_frequency"];
      currentReadings.waterFuzzyOutput.pattern = waterData["fuzzy_pattern"].as<String>();
      currentReadings.waterFuzzyOutput.confidence = waterData["fuzzy_confidence"];
      currentReadings.waterFuzzyOutput.interpretation = waterData["interpretation"].as<String>();
      currentReadings.waterFuzzyOutput.severity = waterData["severity"].as<String>();
      
      Serial.printf("✓ Water Level: %s (Alert Level: %d, Pattern: %s, Confidence: %.2f)\n", 
                   currentReadings.waterDetected ? "Water detected" : "Dry surface",
                   currentReadings.waterFuzzyOutput.alertLevel,
                   currentReadings.waterFuzzyOutput.pattern.c_str(),
                   currentReadings.waterFuzzyOutput.confidence);
    } else {
      Serial.printf("✓ Water Level: %s\n", currentReadings.waterDetected ? "Water detected" : "Dry surface");
    }
  } else {
    Serial.println("✗ Water Level read failed");
    currentReadings.validWaterReading = false;
    currentReadings.systemHealthy = false;
  }
}

void handleUltrasonicFailure() {
  // Implement intelligent fallback when ultrasonic sensor fails
  unsigned long timeSinceLastValid = currentReadings.readTime - previousReadings.readTime;
  
  if (timeSinceLastValid < SENSOR_CONFIDENCE_TIMEOUT && previousReadings.validUltrasonicReading) {
    // Use previous reading with reduced confidence
    currentReadings.distance = previousReadings.distance;
    currentReadings.confidence = previousReadings.confidence * 0.5; // Reduce confidence
    currentReadings.distanceClassification = "inferred";
    
    // Generate conservative fuzzy output
    currentReadings.fuzzyOutput.vibrationIntensity = 
        min(previousReadings.fuzzyOutput.vibrationIntensity * 0.7f, 150.0f);
    currentReadings.fuzzyOutput.vibrationPattern = 
        max(0, previousReadings.fuzzyOutput.vibrationPattern - 1);
    currentReadings.fuzzyOutput.vibrationFrequency = 
        previousReadings.fuzzyOutput.vibrationFrequency * 0.8f;
    currentReadings.fuzzyOutput.fuzzyActive = true;
    
    Serial.println("  Using inferred fuzzy output from previous reading");
  } else {
    // No recent valid data - use safe defaults
    currentReadings.fuzzyOutput.vibrationIntensity = 0.0;
    currentReadings.fuzzyOutput.vibrationPattern = 0;
    currentReadings.fuzzyOutput.vibrationFrequency = 0.0;
    currentReadings.fuzzyOutput.fuzzyActive = false;
    
    Serial.println("  No recent valid data - fuzzy output disabled");
  }
}

// FIXED: Updated feedback system with immediate infrared response
void updateIntelligentFeedback() {
  // Update vibration motor with fuzzy logic output
  updateVibrationMotorFeedback();
  
  // FIXED: Update buzzer with immediate infrared response (no throttling)
  updateBuzzerFeedbackFixed();
  
  // Update feedback hardware state machines
  vibrationMotor.update();
  buzzer.update();
}

void updateVibrationMotorFeedback() {
  if (!vibrationMotor.isEnabled()) {
    return;
  }
  
  // REMOVE CONFIDENCE GATING - Let fuzzy logic handle confidence internally
  if (currentReadings.validUltrasonicReading) { 
    
    // Always apply fuzzy output - no confidence threshold here
    vibrationMotor.setFuzzyOutput(
      currentReadings.fuzzyOutput.vibrationIntensity,
      currentReadings.fuzzyOutput.vibrationPattern,
      currentReadings.fuzzyOutput.vibrationFrequency
    );
    
    // Enhanced debugging
    static float lastIntensity = -1;
    if (abs(currentReadings.fuzzyOutput.vibrationIntensity - lastIntensity) > 10) {
      Serial.printf("🔸 Vibration: %.0f PWM @ %.1fcm (conf: %.2f, pattern: %d)\n",
                   currentReadings.fuzzyOutput.vibrationIntensity,
                   currentReadings.distance,
                   currentReadings.confidence,
                   currentReadings.fuzzyOutput.vibrationPattern);
      lastIntensity = currentReadings.fuzzyOutput.vibrationIntensity;
    }
  } else {
    // Only turn off after extended sensor failure
    vibrationMotor.setFuzzyOutput(0.0, 0, 0.0);
  }
}

// FIXED: New buzzer feedback function with immediate infrared response
void updateBuzzerFeedbackFixed() {
  if (!buzzer.isEnabled()) {
    return;
  }
  
  // Priority 1: Handle infrared sensor with IMMEDIATE response (no cooldown)
  if (currentReadings.validInfraredReading) {
    // FIXED: Pass surfaceDetected directly to buzzer
    // buzzer.alertForHole() expects: TRUE = surface present (no alert), FALSE = no surface (alert)
    bool alertTriggered = buzzer.alertForHole(currentReadings.surfaceDetected);
    
    // Debug logging for immediate response
    static bool lastSurfaceState = true;
    if (currentReadings.surfaceDetected != lastSurfaceState) {
      lastSurfaceState = currentReadings.surfaceDetected;
      
      if (!currentReadings.surfaceDetected) {
        Serial.println("🔊 IMMEDIATE: NO SURFACE DETECTED - Buzzer ACTIVATED");
        Serial.println("    Binary Logic: Surface absent = BUZZER ON");
      } else {
        Serial.println("🔊 IMMEDIATE: Surface detected - Buzzer DEACTIVATED");
        Serial.println("    Binary Logic: Surface present = BUZZER OFF");
      }
    }
    
    // If no surface detected (hole), exit immediately - don't process water alerts
    if (!currentReadings.surfaceDetected) {
      return;
    }
  }
  
  // Priority 2: Handle water detection with fuzzy logic (only if surface is present)
  if (currentReadings.validWaterReading && currentReadings.waterFuzzyOutput.alertLevel > 0) {
    // Create fuzzy configuration from current readings
    Buzzer::FuzzyAlertConfig fuzzyConfig;
    fuzzyConfig.alertLevel = currentReadings.waterFuzzyOutput.alertLevel;
    fuzzyConfig.frequency = currentReadings.waterFuzzyOutput.frequency;
    fuzzyConfig.patternName = currentReadings.waterFuzzyOutput.pattern;
    fuzzyConfig.confidence = currentReadings.waterFuzzyOutput.confidence;
    
    // Use the enhanced buzzer's fuzzy alert method
    if (buzzer.alertForWaterFuzzy(fuzzyConfig)) {
      Serial.printf("🔊 WATER HAZARD - Fuzzy alert Level %d (%s)\n",
                   fuzzyConfig.alertLevel,
                   currentReadings.waterFuzzyOutput.severity.c_str());
      Serial.printf("    Pattern: %s, Frequency: %d Hz, Confidence: %.2f\n",
                   fuzzyConfig.patternName.c_str(),
                   fuzzyConfig.frequency,
                   fuzzyConfig.confidence);
    }
  }
  // Handle case where water alert level drops to 0 (clear water condition)
  else if (currentReadings.validWaterReading && 
           currentReadings.waterFuzzyOutput.alertLevel == 0 &&
           previousReadings.waterFuzzyOutput.alertLevel > 0) {
    // Water condition cleared - send silent alert to update buzzer state
    Buzzer::FuzzyAlertConfig clearConfig = {0, 0, "silent", 1.0};
    buzzer.alertForWaterFuzzy(clearConfig);
    Serial.println("🔊 Water hazard cleared - Fuzzy alert deactivated");
  }
}

void printSensorSummary() {
  Serial.println("--- Sensor Summary ---");
  Serial.printf("Distance: %.1f cm (Classification: %s, Confidence: %.2f)\n", 
               currentReadings.distance, 
               currentReadings.distanceClassification.c_str(), 
               currentReadings.confidence);
  
  // FIXED: Enhanced infrared reporting with correct binary logic
  Serial.printf("Infrared: %s (Binary Logic: Buzzer %s)\n", 
               currentReadings.surfaceDetected ? "Surface Present" : "NO SURFACE/HOLE DETECTED",
               currentReadings.surfaceDetected ? "INACTIVE" : "ACTIVE");
  
  Serial.printf("Water: %s (Alert Level: %d, Severity: %s)\n",
               currentReadings.waterDetected ? "WET" : "Dry",
               currentReadings.waterFuzzyOutput.alertLevel,
               currentReadings.waterFuzzyOutput.severity.c_str());
  
  Serial.printf("Ultrasonic Fuzzy: Intensity=%.0f, Pattern=%d, Freq=%.1f Hz\n",
               currentReadings.fuzzyOutput.vibrationIntensity,
               currentReadings.fuzzyOutput.vibrationPattern,
               currentReadings.fuzzyOutput.vibrationFrequency);
  
  Serial.printf("Water Fuzzy: Alert=%d, Pattern=%s, Freq=%d Hz, Conf=%.2f\n",
               currentReadings.waterFuzzyOutput.alertLevel,
               currentReadings.waterFuzzyOutput.pattern.c_str(),
               currentReadings.waterFuzzyOutput.frequency,
               currentReadings.waterFuzzyOutput.confidence);
  
  // FIXED: Enhanced alert status summary with correct logic
  String alertStatus = "SAFE";
  if (!currentReadings.surfaceDetected) {  // No surface = danger
    alertStatus = "CRITICAL: NO SURFACE DETECTED";
  } else if (currentReadings.waterFuzzyOutput.alertLevel > 0) {
    alertStatus = "WARNING: WATER HAZARD L" + String(currentReadings.waterFuzzyOutput.alertLevel);
  }
  
  Serial.printf("Alert Status: %s\n", alertStatus.c_str());
  Serial.printf("System Health: %s | Valid Readings: U:%s I:%s W:%s\n",
               currentReadings.systemHealthy ? "OK" : "DEGRADED",
               currentReadings.validUltrasonicReading ? "✓" : "✗",
               currentReadings.validInfraredReading ? "✓" : "✗",
               currentReadings.validWaterReading ? "✓" : "✗");
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
    Serial.printf("Queue remaining: %d items\n", dataQueue.size());
  }
  Serial.println();
}

bool sendDataToServer(const String& jsonData) {
  HTTPClient http;
  http.begin(SERVER_URL);
  
  // Set headers
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  http.addHeader("User-Agent", "ESP32-SensorClient-FixedInfrared/3.2");
  http.addHeader("X-Device-Features", "fixed-infrared-logic,fuzzy-logic,vibration-control,multi-sensor");
  
  // Set timeout
  http.setTimeout(15000); // 15 seconds
  
  // Send POST request
  int httpResponseCode = http.POST(jsonData);
  
  bool success = false;
  if (httpResponseCode > 0) {
    String response = http.getString();
    
    if (httpResponseCode == 201 || httpResponseCode == 200) {
      success = true;
      if (response.length() < 200) { // Only print short responses
        Serial.printf("Server response: %s\n", response.c_str());
      } else {
        Serial.println("✓ Server accepted data");
      }
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
  
  DynamicJsonDocument doc(768); // Increased for fuzzy logic data
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
  
  // Add fuzzy logic system status
  JsonObject fuzzyStatus = data.createNestedObject("fuzzy_logic");
  fuzzyStatus["enabled"] = currentReadings.fuzzyOutput.fuzzyActive;
  fuzzyStatus["last_confidence"] = currentReadings.confidence;
  fuzzyStatus["current_pattern"] = currentReadings.fuzzyOutput.vibrationPattern;
  fuzzyStatus["system_healthy"] = currentReadings.systemHealthy;
  
  // Add vibration motor status
  auto vibStatus = vibrationMotor.getStatus();
  JsonObject vibrationStatus = data.createNestedObject("vibration_motor");
  vibrationStatus["active"] = vibStatus.isActive;
  vibrationStatus["current_intensity"] = vibStatus.currentIntensity;
  vibrationStatus["current_pattern"] = vibStatus.currentPattern;
  vibrationStatus["activation_count"] = vibStatus.activationCount;
  vibrationStatus["total_runtime_ms"] = vibStatus.totalRunTime;
  
  // Add sensor health status
  JsonObject sensorHealth = data.createNestedObject("sensor_health");
  sensorHealth["ultrasonic_valid"] = currentReadings.validUltrasonicReading;
  sensorHealth["infrared_valid"] = currentReadings.validInfraredReading;
  sensorHealth["water_valid"] = currentReadings.validWaterReading;
  sensorHealth["last_valid_reading_age_ms"] = millis() - currentReadings.readTime;
  
  // FIXED: Add infrared binary state to heartbeat with correct logic
  JsonObject infraredStatus = data.createNestedObject("infrared_binary");
  infraredStatus["surface_detected"] = currentReadings.surfaceDetected;
  infraredStatus["hole_detected"] = !currentReadings.surfaceDetected;  // Opposite of surface detected
  infraredStatus["logic_type"] = "binary";
  infraredStatus["buzzer_active"] = !currentReadings.surfaceDetected;  // Buzzer active when no surface
  infraredStatus["threshold"] = infraredSensor.getThreshold();
  infraredStatus["last_raw_value"] = infraredSensor.getLastRawValue();

  String jsonString;
  serializeJson(doc, jsonString);
  
  if (sendDataToServer(jsonString)) {
    Serial.println("✓ Enhanced heartbeat with fixed infrared logic sent successfully");
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
  system["firmware_version"] = "3.2-fixed-infrared";
  system["build_features"] = "fixed-infrared-logic,immediate-response,fuzzy-logic,vibration-control,multi-sensor,queue-system";
}

void addFuzzyLogicInfo(JsonObject& data) {
  JsonObject fuzzyInfo = data.createNestedObject("fuzzy_performance");
  
  // Ultrasonic fuzzy logic info (existing)
  fuzzyInfo["ultrasonic_confidence_threshold"] = CONFIDENCE_THRESHOLD;
  fuzzyInfo["feedback_update_hz"] = 1000.0 / FEEDBACK_UPDATE_INTERVAL;
  fuzzyInfo["ultrasonic_signal_stability"] = currentReadings.signalStability;
  fuzzyInfo["ultrasonic_inference_active"] = currentReadings.fuzzyOutput.fuzzyActive;
  
  // NEW: Water fuzzy logic info
  fuzzyInfo["water_alert_level"] = currentReadings.waterFuzzyOutput.alertLevel;
  fuzzyInfo["water_pattern"] = currentReadings.waterFuzzyOutput.pattern;
  fuzzyInfo["water_confidence"] = currentReadings.waterFuzzyOutput.confidence;
  fuzzyInfo["water_severity"] = currentReadings.waterFuzzyOutput.severity;
  fuzzyInfo["water_interpretation"] = currentReadings.waterFuzzyOutput.interpretation;
  
  // Performance metrics (existing logic enhanced)
  static unsigned long totalInferences = 0;
  static unsigned long validInferences = 0;
  static unsigned long totalWaterInferences = 0;
  static unsigned long validWaterInferences = 0;
  
  // Count ultrasonic inferences
  if (currentReadings.validUltrasonicReading) {
    totalInferences++;
    if (currentReadings.confidence >= CONFIDENCE_THRESHOLD) {
      validInferences++;
    }
  }
  
  // NEW: Count water fuzzy inferences
  if (currentReadings.validWaterReading) {
    totalWaterInferences++;
    if (currentReadings.waterFuzzyOutput.confidence >= 0.5) { // 50% confidence threshold
      validWaterInferences++;
    }
  }
  
  fuzzyInfo["total_ultrasonic_inferences"] = totalInferences;
  fuzzyInfo["valid_ultrasonic_inferences"] = validInferences;
  fuzzyInfo["ultrasonic_success_rate"] = totalInferences > 0 ? 
    (float)validInferences / totalInferences : 0.0;
    
  fuzzyInfo["total_water_inferences"] = totalWaterInferences;
  fuzzyInfo["valid_water_inferences"] = validWaterInferences;
  fuzzyInfo["water_success_rate"] = totalWaterInferences > 0 ? 
    (float)validWaterInferences / totalWaterInferences : 0.0;
}

String getISOTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Warning: Failed to obtain time, using fallback timestamp");
    
    // Use a fixed recent timestamp as fallback
    return "2025-06-23T08:00:00Z";
  }
  
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
}

bool ensureTimeSync() {
  struct tm timeinfo;
  int retries = 0;
  const int maxRetries = 10;
  
  while (!getLocalTime(&timeinfo) && retries < maxRetries) {
    Serial.printf("Attempting NTP sync... (attempt %d/%d)\n", retries + 1, maxRetries);
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    delay(2000);
    retries++;
  }
  
  if (retries >= maxRetries) {
    Serial.println("Failed to sync time after maximum retries");
    return false;
  }
  
  Serial.println("Time synchronized successfully");
  return true;
}

// Enhanced error handling and recovery functions
void handleSystemErrors() {
  static unsigned long lastErrorCheck = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastErrorCheck < 5000) {
    return; // Check every 5 seconds
  }
  lastErrorCheck = currentTime;
  
  // Check for sensor communication failures
  if (!currentReadings.systemHealthy) {
    Serial.println("System health degraded - attempting recovery...");
    
    // Attempt sensor recovery
    if (!currentReadings.validUltrasonicReading) {
      Serial.println("Attempting ultrasonic sensor recovery...");
      ultrasonicSensor.begin(); // Re-initialize
    }
    
    if (!currentReadings.validInfraredReading) {
      Serial.println("Attempting infrared sensor recovery...");
      infraredSensor.begin();
    }
    
    if (!currentReadings.validWaterReading) {
      Serial.println("Attempting water sensor recovery...");
      waterLevelSensor.begin();
    }
  }
  
  // Check memory usage
  uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < 10000) { // Less than 10KB free
    Serial.printf("WARNING: Low memory - %d bytes free\n", freeHeap);
    
    // Emergency cleanup
    if (dataQueue.size() > 20) {
      Serial.println("Emergency: Clearing old data from queue");
      for (int i = 0; i < 10 && !dataQueue.isEmpty(); i++) {
        dataQueue.dequeue();
      }
    }
  }
}

// Optional: Advanced fuzzy logic debugging
void debugFuzzyLogicOutput() {
  #ifdef DEBUG_FUZZY_LOGIC
  static unsigned long lastDebugOutput = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastDebugOutput > 1000) { // Debug every second
    Serial.println("=== Fuzzy Logic Debug ===");
    Serial.printf("Input Distance: %.1f cm\n", currentReadings.distance);
    Serial.printf("Input Confidence: %.2f\n", currentReadings.confidence);
    Serial.printf("Signal Stability: %.2f\n", currentReadings.signalStability);
    Serial.printf("Classification: %s\n", currentReadings.distanceClassification.c_str());
    Serial.printf("Output Intensity: %.0f (0-255)\n", currentReadings.fuzzyOutput.vibrationIntensity);
    Serial.printf("Output Pattern: %d\n", currentReadings.fuzzyOutput.vibrationPattern);
    Serial.printf("Output Frequency: %.1f Hz\n", currentReadings.fuzzyOutput.vibrationFrequency);
    Serial.printf("Fuzzy Active: %s\n", currentReadings.fuzzyOutput.fuzzyActive ? "YES" : "NO");
    
    auto vibStatus = vibrationMotor.getStatus();
    Serial.printf("Motor State: %d (0=inactive, 1=pulse_on, 2=pulse_off, 3=continuous)\n", 
                 vibStatus.currentState);
    Serial.printf("Motor Active: %s\n", vibStatus.isActive ? "YES" : "NO");
    Serial.println("========================");
    
    lastDebugOutput = currentTime;
  }
  #endif
}