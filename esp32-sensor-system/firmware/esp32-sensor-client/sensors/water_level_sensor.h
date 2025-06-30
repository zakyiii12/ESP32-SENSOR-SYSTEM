  #ifndef WATER_LEVEL_SENSOR_H
  #define WATER_LEVEL_SENSOR_H

  #include <Arduino.h>
  #include <ArduinoJson.h>
  #include "../config.h"

  /**
  * Water Level Sensor Module with Fuzzy Logic for Wet Surface Detection
  * 
  * This enhanced sensor uses fuzzy logic to intelligently detect water/wet surfaces
  * by analyzing moisture level, persistence, and reading trends. It provides
  * graduated responses rather than simple binary detection.
  * 
  * Fuzzy Variables:
  * - Input: Moisture Level (DRY, DAMP, WET)
  * - Input: Persistence (TRANSIENT, PERSISTENT) 
  * - Input: Trend (rate of change)
  * - Output: Alert Level (0-3), Frequency (Hz), Pattern Type
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
    
    // Fuzzy Logic Components
    struct FuzzyOutput {
      int alertLevel;        // 0-3 (Silent, Gentle, Moderate, Urgent)
      int frequency;         // Hz for buzzer
      String pattern;        // Beep pattern type
      float confidence;      // 0.0-1.0 confidence in decision
    };
    
    // Persistence tracking for fuzzy logic
    static const int PERSISTENCE_WINDOW = 5;
    float persistenceWeights[PERSISTENCE_WINDOW] = {1.0, 0.9, 0.81, 0.729, 0.6561}; // 0.9^k
    bool wetFlags[PERSISTENCE_WINDOW] = {false, false, false, false, false};
    int persistenceIndex = 0;
    
    // Trend analysis
    int previousReading = 0;
    float trendMagnitude = 0.0;
    
    // Water detection confidence
    int consecutiveWaterReadings = 0;
    int consecutiveDryReadings = 0;
    static const int CONFIDENCE_THRESHOLD = 3;
    
    // Error handling
    static const int MAX_CONSECUTIVE_ERRORS = 10;
    static const int MIN_READ_INTERVAL = 100; // ms
    
    // Fuzzy Logic Parameters (based on technical documentation)
    static const int DRY_MAX = 200;
    static const int DAMP_START = 150;
    static const int DAMP_PEAK_START = 250;
    static const int DAMP_PEAK_END = 300;
    static const int DAMP_END = 400;
    static const int WET_START = 350;
    static const int WET_FULL = 550;
    
  public:
    WaterLevelSensor() {
      // Initialize reading history
      for (int i = 0; i < HISTORY_SIZE; i++) {
        readingHistory[i] = 0;
      }
      // Initialize persistence tracking
      for (int i = 0; i < PERSISTENCE_WINDOW; i++) {
        wetFlags[i] = false;
      }
    }
    
    /**
    * Initialize the water level sensor with fuzzy logic capabilities
    */
    bool begin() {
      if (!enabled) {
        Serial.println("WaterLevel: Sensor disabled in configuration");
        return true;
      }
      
      Serial.println("WaterLevel: Initializing fuzzy logic wet surface detection sensor...");
      
      // Validate pin configuration
      if (sensorPin < 0) {
        Serial.println("WaterLevel: Invalid pin configuration");
        return false;
      }
      
      // Configure pin (ADC pins don't need explicit pinMode)
      Serial.printf("WaterLevel: Configured on GPIO %d\n", sensorPin);
      
      // Perform calibration
      Serial.println("WaterLevel: Performing dry surface calibration for fuzzy logic...");
      Serial.println("WaterLevel: Ensure sensor is on dry surface during calibration!");
      
      delay(2000); // Give time to position sensor
      
      if (!performCalibration()) {
        Serial.println("WaterLevel: Calibration failed");
        return false;
      }
      
      initialized = true;
      Serial.printf("WaterLevel: Fuzzy logic initialization complete - Dry baseline: %d, Threshold: %d\n", 
                  dryBaseline, dynamicThreshold);
      return true;
    }
    
    /**
    * Read sensor and perform fuzzy logic analysis for water detection
    */
    bool readSensor(JsonObject& data) {
      if (!enabled || !initialized) {
        return false;
      }
      
      unsigned long currentTime = millis();
      
      // Enforce minimum read interval
      if (currentTime - lastReadTime < MIN_READ_INTERVAL) {
        // Return last known state with fuzzy analysis
        FuzzyOutput fuzzyResult = performFuzzyAnalysis(lastRawValue);
        populateJsonData(data, lastRawValue, lastWaterDetected, "rate_limited", currentTime, 0.0, fuzzyResult);
        return true;
      }
      
      Serial.println("WaterLevel: Reading sensor for fuzzy water detection...");
      
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
          delay(50);
        }
      }
      
      lastReadTime = currentTime;
      
      if (validSamples == 0) {
        Serial.println("WaterLevel: All samples invalid");
        consecutiveErrors++;
        FuzzyOutput fuzzyResult = {0, 0, "silent", 0.0};
        populateJsonData(data, lastRawValue, lastWaterDetected, "read_error", currentTime, 0.0, fuzzyResult);
        return false;
      }
      
      // Calculate average of valid samples
      int sum = 0;
      for (int i = 0; i < validSamples; i++) {
        sum += samples[i];
      }
      int averageReading = sum / validSamples;
      
      // Update reading history and trend analysis
      updateReadingHistory(averageReading);
      updateTrendAnalysis(averageReading);
      
      // Apply smoothing filter
      int smoothedReading = getSmoothedReading();
      
      // Perform fuzzy logic analysis
      FuzzyOutput fuzzyResult = performFuzzyAnalysis(smoothedReading);
      
      // Update persistence tracking
      updatePersistenceTracking(smoothedReading);
      
      // Determine binary water detection for compatibility
      bool waterDetected = (fuzzyResult.alertLevel > 0);
      
      // Calculate water level percentage
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
      
      // Populate JSON data with fuzzy results
      populateJsonData(data, smoothedReading, waterDetected, status, currentTime, waterLevelPercent, fuzzyResult);
      
      Serial.printf("WaterLevel: Fuzzy Result - Raw: %d, Smoothed: %d, Alert Level: %d, Freq: %d Hz, Pattern: %s, Confidence: %.2f\n", 
                  averageReading, smoothedReading, fuzzyResult.alertLevel, fuzzyResult.frequency, 
                  fuzzyResult.pattern.c_str(), fuzzyResult.confidence);
      
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
    
    /**
    * Get fuzzy logic analysis results for external use (buzzer control)
    */
    FuzzyOutput getFuzzyAnalysis() const {
      if (!initialized) {
        return {0, 0, "silent", 0.0};
      }
      return performFuzzyAnalysis(lastRawValue);
    }
    
  private:
    /**
    * Perform fuzzy logic analysis based on technical documentation
    */
    FuzzyOutput performFuzzyAnalysis(int reading) const {
      // Calculate membership functions
      float dryMembership = calculateDryMembership(reading);
      float dampMembership = calculateDampMembership(reading);
      float wetMembership = calculateWetMembership(reading);
      
      // Calculate persistence membership
      float persistenceValue = calculatePersistenceValue();
      float transientMembership = calculateTransientMembership(persistenceValue);
      float persistentMembership = calculatePersistentMembership(persistenceValue);
      
      // Apply fuzzy rules (from technical documentation)
      FuzzyOutput result = {0, 0, "silent", 0.0};
      float maxActivation = 0.0;
      
      // Rule 1: IF (r is WET) AND (P is PERSISTENT) THEN A = 3, f = 1500, Pattern = Continuous
      float rule1Activation = min(wetMembership, persistentMembership);
      if (rule1Activation > maxActivation) {
        maxActivation = rule1Activation;
        result = {3, 1500, "continuous", rule1Activation};
      }
      
      // Rule 2: IF (r is WET) AND (P is TRANSIENT) THEN A = 2, f = 1000, Pattern = Double_Beep
      float rule2Activation = min(wetMembership, transientMembership);
      if (rule2Activation > maxActivation) {
        maxActivation = rule2Activation;
        result = {2, 1000, "double_beep", rule2Activation};
      }
      
      // Rule 3: IF (r is DAMP) AND (P is PERSISTENT) THEN A = 1, f = 500, Pattern = Single_Beep
      float rule3Activation = min(dampMembership, persistentMembership);
      if (rule3Activation > maxActivation) {
        maxActivation = rule3Activation;
        result = {1, 500, "single_beep", rule3Activation};
      }
      
      // Rule 4: IF (r is DAMP) AND (P is TRANSIENT) THEN A = 0, f = 0, Pattern = Silent
      float rule4Activation = min(dampMembership, transientMembership);
      if (rule4Activation > maxActivation) {
        maxActivation = rule4Activation;
        result = {0, 0, "silent", rule4Activation};
      }
      
      // Rule 5: IF (r is DRY) THEN A = 0, f = 0, Pattern = Silent
      if (dryMembership > maxActivation) {
        result = {0, 0, "silent", dryMembership};
      }
      
      return result;
    }
    
    /**
    * Membership functions based on technical documentation
    */
    float calculateDryMembership(int reading) const {
      if (reading <= 100) return 1.0;
      if (reading > DRY_MAX) return 0.0;
      return (DRY_MAX - reading) / 100.0;
    }
    
    float calculateDampMembership(int reading) const {
      if (reading <= DAMP_START || reading >= DAMP_END) return 0.0;
      if (reading > DAMP_START && reading <= DAMP_PEAK_START) return (reading - DAMP_START) / 100.0;
      if (reading > DAMP_PEAK_START && reading <= DAMP_PEAK_END) return 1.0;
      return (DAMP_END - reading) / 100.0;
    }
    
    float calculateWetMembership(int reading) const {
      if (reading <= WET_START) return 0.0;
      if (reading > WET_FULL) return 1.0;
      return (reading - WET_START) / 200.0;
    }
    
    /**
    * Calculate persistence value using exponential decay weights
    */
    float calculatePersistenceValue() const {
      float persistenceSum = 0.0;
      for (int i = 0; i < PERSISTENCE_WINDOW; i++) {
        if (wetFlags[i]) {
          persistenceSum += persistenceWeights[i];
        }
      }
      return persistenceSum;
    }
    
    float calculateTransientMembership(float persistenceValue) const {
      return exp(-persistenceValue / 0.5);
    }
    
    float calculatePersistentMembership(float persistenceValue) const {
      return 1.0 - exp(-persistenceValue / 0.5);
    }
    
    /**
    * Update persistence tracking for fuzzy logic
    */
    void updatePersistenceTracking(int reading) {
      bool isWet = (reading > dynamicThreshold);
      wetFlags[persistenceIndex] = isWet;
      persistenceIndex = (persistenceIndex + 1) % PERSISTENCE_WINDOW;
    }
    
    /**
    * Update trend analysis for future enhancements
    */
    void updateTrendAnalysis(int reading) {
      if (previousReading != 0) {
        trendMagnitude = abs(reading - previousReading);
      }
      previousReading = reading;
    }
    
    /**
    * Perform initial calibration to establish dry baseline
    */
    bool performCalibration() {
      Serial.println("WaterLevel: Starting dry surface calibration for fuzzy logic...");
      
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
        
        delay(200);
      }
      
      if (validReadings < calibrationSamples / 2) {
        Serial.println("WaterLevel: Insufficient valid calibration samples");
        return false;
      }
      
      // Calculate dry baseline
      long sum = 0;
      int minReading = 4095, maxReading = 0;
      
      for (int i = 0; i < validReadings; i++) {
        sum += calibrationReadings[i];
        if (calibrationReadings[i] < minReading) minReading = calibrationReadings[i];
        if (calibrationReadings[i] > maxReading) maxReading = calibrationReadings[i];
      }
      
      dryBaseline = sum / validReadings;
      int readingRange = maxReading - minReading;
      
      // Set dynamic threshold for fuzzy logic
      dynamicThreshold = dryBaseline + (readingRange * 2) + 300;
      
      // Ensure threshold is within reasonable bounds
      if (dynamicThreshold > 3800) dynamicThreshold = 3800;
      if (dynamicThreshold < (dryBaseline + 100)) dynamicThreshold = dryBaseline + 100;
      
      isCalibrated = true;
      Serial.printf("WaterLevel: Fuzzy calibration complete - Dry: %d, Range: %d, Threshold: %d\n", 
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
      int count = min(historyCount, 8);
      
      // Apply weights (more recent readings have higher weight)
      for (int i = 0; i < count; i++) {
        int idx = (historyIndex - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
        int weight = count - i;
        weightedSum += readingHistory[idx] * weight;
        totalWeight += weight;
      }
      
      return weightedSum / totalWeight;
    }
    
    /**
    * Calculate water level as percentage (0-100%)
    */
    float calculateWaterLevelPercent(int reading) {
      if (!isCalibrated) {
        return (reading / 4095.0) * 100.0;
      }
      
      if (reading <= dryBaseline) {
        return 0.0;
      }
      
      int maxWaterReading = 3800;
      int waterRange = maxWaterReading - dryBaseline;
      int currentWaterLevel = reading - dryBaseline;
      
      float percent = (currentWaterLevel / (float)waterRange) * 100.0;
      
      if (percent < 0) percent = 0;
      if (percent > 100) percent = 100;
      
      return percent;
    }
    
    /**
    * Populate JSON data with fuzzy logic results
    */
    void populateJsonData(JsonObject& data, int rawValue, bool waterDetected, 
                        String status, unsigned long timestamp, float waterLevelPercent = 0.0,
                        FuzzyOutput fuzzyResult = {0, 0, "silent", 0.0}) const {
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
      data["data_age_ms"] = timestamp - lastValidReadTime;
      
      // Fuzzy logic results
      data["fuzzy_alert_level"] = fuzzyResult.alertLevel;
      data["fuzzy_frequency"] = fuzzyResult.frequency;
      data["fuzzy_pattern"] = fuzzyResult.pattern;
      data["fuzzy_confidence"] = round(fuzzyResult.confidence * 1000) / 1000.0;
      
      // Add interpretation based on fuzzy results
      if (fuzzyResult.alertLevel == 0) {
        data["interpretation"] = "Dry surface";
        data["alert_level"] = "normal";
        data["severity"] = "none";
      } else if (fuzzyResult.alertLevel == 1) {
        data["interpretation"] = "Damp surface";
        data["alert_level"] = "caution";
        data["severity"] = "low";
      } else if (fuzzyResult.alertLevel == 2) {
        data["interpretation"] = "Wet surface detected";
        data["alert_level"] = "warning";
        data["severity"] = "medium";
      } else {
        data["interpretation"] = "Significant water hazard";
        data["alert_level"] = "danger";
        data["severity"] = "high";
      }
    }
  };

  #endif