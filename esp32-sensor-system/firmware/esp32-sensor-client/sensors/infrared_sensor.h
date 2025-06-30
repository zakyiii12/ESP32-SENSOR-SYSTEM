#ifndef INFRARED_SENSOR_H
#define INFRARED_SENSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../config.h"

/**
 * Infrared Sensor Module for Binary Hole/Drop Detection
 * 
 * This sensor detects holes, drops, or absence of surface by measuring infrared reflection.
 * BINARY LOGIC: 
 * - Reading BELOW 3000 = Surface Present (normal reflection) = NO ALERT
 * - Reading ABOVE 3000 = No Surface/Hole/Drop (low/no reflection) = ALERT ACTIVATED
 * 
 * Connected to GPIO 36 (ADC1_CH0) for analog readings.
 */
class InfraredSensor {
private:
  bool initialized = false;
  bool enabled = INFRARED_ENABLED;
  int sensorPin = INFRARED_PIN;
  
  // Sensor state tracking - BINARY LOGIC
  int lastRawValue = 0;
  bool surfaceDetected = true;  // TRUE = surface present, FALSE = hole/drop detected
  unsigned long lastReadTime = 0;
  unsigned long lastValidReadTime = 0;
  int consecutiveErrors = 0;
  
  // Fixed threshold for immediate response
  int surfaceThreshold = 3000;  // Fixed threshold: below = surface, above = hole
  int baselineReading = 0;
  bool isCalibrated = false;
  
  // Reading history for display purposes only
  static const int HISTORY_SIZE = 5;
  int readingHistory[HISTORY_SIZE];
  int historyIndex = 0;
  int historyCount = 0;
  
  // Removed stability checking for immediate response
  
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
   * Initialize the infrared sensor with binary detection logic
   */
  bool begin() {
    if (!enabled) {
      Serial.println("Infrared: Sensor disabled in configuration");
      return true;
    }
    
    Serial.println("Infrared: Initializing binary hole/drop detection sensor...");
    
    // Validate pin configuration
    if (sensorPin < 0) {
      Serial.println("Infrared: Invalid pin configuration");
      return false;
    }
    
    // Configure pin (ADC pins don't need explicit pinMode)
    Serial.printf("Infrared: Configured on GPIO %d\n", sensorPin);
    
    // Set fixed threshold for immediate response
    surfaceThreshold = 3000;
    isCalibrated = true;
    
    initialized = true;
    Serial.printf("Infrared: Binary detection ready - Surface Threshold: %d\n", surfaceThreshold);
    Serial.println("Infrared: LOGIC - Reading < 3000 = Surface Present (NO ALERT)");
    Serial.println("Infrared: LOGIC - Reading >= 3000 = Hole/Drop (ALERT ACTIVATED)");
    return true;
  }
  
  /**
   * Read sensor and perform IMMEDIATE binary surface detection
   * Returns true if reading was successful
   */
  bool readSensor(JsonObject& data) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Enforce minimum read interval
    if (currentTime - lastReadTime < MIN_READ_INTERVAL) {
      // Return last known state
      populateJsonData(data, lastRawValue, !surfaceDetected, "rate_limited", currentTime);
      return true;
    }
    
    Serial.println("Infrared: Reading sensor for immediate binary surface detection...");
    
    // Take fewer samples for faster response
    int samples[3];  // Reduced from INFRARED_SAMPLES for speed
    int validSamples = 0;
    
    for (int i = 0; i < 3; i++) {
      int reading = analogRead(sensorPin);
      
      // Validate reading (ADC should return 0-4095)
      if (reading >= 0 && reading <= 4095) {
        samples[validSamples] = reading;
        validSamples++;
        Serial.printf("Infrared: Sample %d: %d\n", i + 1, reading);
      } else {
        Serial.printf("Infrared: Sample %d: invalid (%d)\n", i + 1, reading);
      }
      
      if (i < 2) {
        delay(10); // Shorter delay for faster response
      }
    }
    
    lastReadTime = currentTime;
    
    if (validSamples == 0) {
      Serial.println("Infrared: All samples invalid");
      consecutiveErrors++;
      populateJsonData(data, lastRawValue, !surfaceDetected, "read_error", currentTime);
      return false;
    }
    
    // Calculate average of valid samples
    int sum = 0;
    for (int i = 0; i < validSamples; i++) {
      sum += samples[i];
    }
    int averageReading = sum / validSamples;
    
    // Update reading history for display
    updateReadingHistory(averageReading);
    
    // IMMEDIATE binary surface detection - NO stability checking
    // Fixed logic: Below 3000 = Surface, Above/Equal 3000 = Hole
    bool newSurfaceDetected = (averageReading < surfaceThreshold);
    
    // IMMEDIATE state update - no waiting for consecutive readings
    if (newSurfaceDetected != surfaceDetected) {
      surfaceDetected = newSurfaceDetected;
      
      if (surfaceDetected) {
        Serial.printf("Infrared: SURFACE DETECTED (Reading: %d < %d) - Buzzer OFF\n", 
                     averageReading, surfaceThreshold);
      } else {
        Serial.printf("Infrared: HOLE/DROP DETECTED (Reading: %d >= %d) - Buzzer ON\n", 
                     averageReading, surfaceThreshold);
      }
    }
    
    // Update state
    lastRawValue = averageReading;
    lastValidReadTime = currentTime;
    consecutiveErrors = 0;
    
    // Determine status
    String status = "normal";
    if (validSamples < 3) {
      status = "partial_samples";
    }
    
    // Populate JSON data - hole_detected is the inverse of surfaceDetected
    bool holeDetected = !surfaceDetected;
    populateJsonData(data, averageReading, holeDetected, status, currentTime);
    
    Serial.printf("Infrared: IMMEDIATE Result - Raw: %d, Surface: %s, Buzzer: %s\n", 
                 averageReading,
                 surfaceDetected ? "DETECTED" : "NOT_DETECTED",
                 holeDetected ? "ACTIVE" : "INACTIVE");
    
    return true;
  }
  
  // Getter methods for external access - BINARY LOGIC
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  bool isHealthy() const { return consecutiveErrors < MAX_CONSECUTIVE_ERRORS; }
  bool isHoleDetected() const { return !surfaceDetected; }  // Alert when NO surface
  bool isSurfaceDetected() const { return surfaceDetected; }  // Surface present
  int getLastRawValue() const { return lastRawValue; }
  int getThreshold() const { return surfaceThreshold; }
  
  // Method for immediate buzzer control
  bool shouldActivateBuzzer() const {
    return !surfaceDetected;  // Buzzer active when NO surface detected
  }
  
private:
  /**
   * Update the reading history buffer (for display only)
   */
  void updateReadingHistory(int reading) {
    readingHistory[historyIndex] = reading;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    
    if (historyCount < HISTORY_SIZE) {
      historyCount++;
    }
  }
  
  /**
   * Populate JSON data for reporting - CORRECTED BINARY LOGIC
   */
  void populateJsonData(JsonObject& data, int rawValue, bool holeDetected, 
                       String status, unsigned long timestamp) const {
    data["raw_value"] = rawValue;
    data["hole_detected"] = holeDetected;  // TRUE when NO surface (alert condition)
    data["surface_detected"] = !holeDetected;  // TRUE when surface present (safe condition)
    data["sensor_type"] = "Infrared_Binary_Fixed";
    data["pin"] = sensorPin;
    data["threshold"] = surfaceThreshold;
    data["baseline"] = baselineReading;
    data["calibrated"] = isCalibrated;
    data["read_time"] = timestamp;
    data["status"] = status;
    data["consecutive_errors"] = consecutiveErrors;
    data["data_age_ms"] = timestamp - lastValidReadTime;
    
    // Add binary interpretation with CORRECTED logic
    if (holeDetected) {
      data["interpretation"] = "DANGER: No surface detected - hole or drop ahead";
      data["alert_level"] = "critical";
      data["buzzer_action"] = "ACTIVE";  // Buzzer ON when hole detected
    } else {
      data["interpretation"] = "SAFE: Normal surface detected";
      data["alert_level"] = "normal";
      data["buzzer_action"] = "INACTIVE";  // Buzzer OFF when surface detected
    }
    
    // Add detection logic info
    data["detection_logic"] = "Fixed: Reading < 3000 = Surface (Buzzer OFF), Reading >= 3000 = Hole (Buzzer ON)";
    data["immediate_response"] = true;
  }
};

#endif