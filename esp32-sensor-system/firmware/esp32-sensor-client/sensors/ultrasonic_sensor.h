#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../config.h"

/**
 * HC-SR04 Ultrasonic Distance Sensor with Smooth Fuzzy Logic Control
 * 
 * Implements gentle, progressive fuzzy logic for comfortable obstacle detection
 * with smooth vibration motor response that escalates gradually with proximity.
 * 
 * Features:
 * - Smooth Fuzzy Logic Controller with gentle transitions
 * - Progressive vibration intensity without time-based escalation
 * - Support for very large distances (up to 4000cm) with extended timeout
 * - Comfortable mid-range feedback (20-80cm range)
 * - Database-compatible output structure
 * - Non-blocking operation with improved pattern analysis
 */
class UltrasonicSensor {
private:
  bool initialized = false;
  bool enabled = HCSR04_ENABLED;
  
  // Pin configuration
  int triggerPin = HCSR04_TRIGGER_PIN;
  int echoPin = HCSR04_ECHO_PIN;
  
  // Fuzzy Logic System Components
  struct FuzzyInput {
    float distance;      // Primary distance measurement d(t)
    float confidence;    // Reading confidence c(t)
    float stability;     // Signal stability σ(t)
  };
  
  struct FuzzyOutput {
    float vibrationIntensity;  // PWM value [0-255]
    int vibrationPattern;      // Pattern type [0-3]
    float vibrationFrequency;  // Frequency [0-50] Hz
  };

  float cachedMean = -1.0;
  float cachedVariance = -1.0;
  bool historyChanged = false;
  
  // Historical data for confidence calculation
  static const int WINDOW_SIZE = 5;
  float distanceHistory[WINDOW_SIZE];
  int historyIndex = 0;
  int historyCount = 0;
  bool hasStableHistory = false;
  
  // Sensor state tracking
  float lastValidDistance = -1.0;
  unsigned long lastReadTime = 0;
  unsigned long lastSuccessfulReadTime = 0;
  int consecutiveErrors = 0;
  
 static constexpr float D_MAX = 150.0; // Reduced for better confidence scaling
  static constexpr float MIN_DISTANCE = 5.0;
  static constexpr float MAX_DISTANCE = 4000.0; // Extended range support
  static constexpr float LARGE_DISTANCE_THRESHOLD = 500.0;

  // SMOOTH MEMBERSHIP ZONES - Gentle, Progressive Response
  // IMMEDIATE ZONE: Very close proximity (0-15cm) - Gentle but clear
  static constexpr float IMMEDIATE_LOW      = 2.0;   // Sensor minimum
  static constexpr float IMMEDIATE_HIGH     = 15.0;  // Reduced for gentler start

  // CLOSE ZONE: Close proximity (10-35cm) - Smooth escalation
  static constexpr float CLOSE_LOW          = 10.0;  // Overlap for smoothness
  static constexpr float CLOSE_MID          = 20.0;  // Peak close response
  static constexpr float CLOSE_HIGH         = 35.0;  // Gentle transition

  // NEAR ZONE: Mid-range detection (25-65cm) - Comfortable feedback
  static constexpr float NEAR_LOW           = 25.0;  // Smooth overlap
  static constexpr float NEAR_MID           = 45.0;  // Optimal mid-range
  static constexpr float NEAR_HIGH          = 65.0;  // Gradual fade

  // MODERATE ZONE: Awareness range (50-100cm) - Subtle indication
  static constexpr float MODERATE_LOW       = 50.0;  // Overlap for continuity
  static constexpr float MODERATE_MID       = 75.0;  // Light feedback
  static constexpr float MODERATE_HIGH      = 100.0; // Fade to safe

  // SAFE ZONE: Comfortable distance (85cm+)
  static constexpr float SAFE_LOW           = 85.0;  // Overlap with moderate
  static constexpr float SAFE_HIGH          = 150.0; // Full comfort zone

  // GENTLE CONFIDENCE CALCULATION - More forgiving
  static constexpr float CONFIDENCE_THRESHOLD = 0.5; // Much lower threshold
  static constexpr float MIN_CONFIDENCE = 0.05;      // Very permissive

  // SMOOTH VIBRATION OUTPUT RANGES - Progressive, Non-Jarring
  static constexpr float VIB_OFF_MAX = 5.0;        // Barely perceptible
  static constexpr float VIB_SUBTLE_MAX = 45.0;     // Gentle awareness
  static constexpr float VIB_LIGHT_MAX = 80.0;      // Comfortable indication
  static constexpr float VIB_MODERATE_MAX = 130.0;  // Clear but not harsh
  static constexpr float VIB_FIRM_MAX = 180.0;      // Strong but tolerable
  static constexpr float VIB_URGENT_MAX = 220.0;    // Maximum for emergencies

public:
  UltrasonicSensor() {
    // Initialize distance history
    for (int i = 0; i < WINDOW_SIZE; i++) {
      distanceHistory[i] = -1.0;
    }
  }
  
  bool begin() {
    if (!enabled) {
      Serial.println("HC-SR04: Sensor disabled in configuration");
      return true;
    }
    
    Serial.println("HC-SR04: Initializing Smooth Fuzzy Logic Ultrasonic Sensor...");
    
    // Validate pin configuration
    if (triggerPin < 0 || echoPin < 0 || triggerPin == echoPin) {
      Serial.println("HC-SR04: Invalid pin configuration");
      return false;
    }
    
    // Configure pins
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(triggerPin, LOW);
    
    Serial.printf("HC-SR04: Configured - Trigger: %d, Echo: %d\n", triggerPin, echoPin);
    
    delay(300);
    
    // Initial calibration
    Serial.println("HC-SR04: Performing smooth fuzzy logic calibration...");
    for (int i = 0; i < WINDOW_SIZE; i++) {
      float testDistance = measureSingleDistance();
      addToHistory(testDistance);
      
      if (testDistance > 0) {
        Serial.printf("HC-SR04: Calibration %d: %.1f cm\n", i + 1, testDistance);
      } else {
        Serial.printf("HC-SR04: Calibration %d: timeout/error\n", i + 1);
      }
      delay(200);
    }
    
    initialized = true;
    Serial.println("HC-SR04: Smooth Fuzzy Logic initialization complete");
    return true;
  }
  
  bool readSensor(JsonObject& data) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Rate limiting
    if (currentTime - lastReadTime < 30) { // Minimum 100ms interval
      populateJsonFromCache(data, currentTime);
      return true;
    }
    
    Serial.println("HC-SR04: Starting Smooth Fuzzy Logic measurement cycle...");
    
    // Take multiple readings for stability analysis
    float readings[SENSOR_RETRY_COUNT];
    int validCount = 0;
    
    // In readSensor() method, replace the measurement loop:
    for (int i = 0; i < SENSOR_RETRY_COUNT; i++) {
      readings[i] = measureSingleDistance();
      if (readings[i] > 0) {
        validCount++;
        Serial.printf("HC-SR04: Reading %d: %.1f cm ✓\n", i + 1, readings[i]);
      } else {
        Serial.printf("HC-SR04: Reading %d: timeout ⚠️\n", i + 1);
      }
      
      // CHANGE THIS: Reduce delay and make it adaptive
      if (i < SENSOR_RETRY_COUNT - 1) {
        delay(validCount > 0 ? 15 : 25); // Shorter delay if we have valid readings
      }
    }
    
    lastReadTime = currentTime;
    
    // Process readings through smooth fuzzy logic system
    float finalDistance = processReadings(readings, validCount);
    FuzzyInput fuzzyInput = calculateFuzzyInputs(finalDistance);
    FuzzyOutput fuzzyOutput = processFuzzyLogic(fuzzyInput);
    
    // Update tracking
    if (finalDistance > 0) {
      lastValidDistance = finalDistance;
      lastSuccessfulReadTime = currentTime;
      consecutiveErrors = 0;
      addToHistory(finalDistance);
    } else {
      consecutiveErrors++;
      // Use fuzzy inference even for failed readings
      finalDistance = inferDistanceFromHistory();
    }
    
    // Populate JSON response
    populateJsonData(data, finalDistance, fuzzyInput, fuzzyOutput, validCount, currentTime);
    
    Serial.printf("HC-SR04: Smooth Result - Distance: %.1f cm, Confidence: %.2f, Vibration: %.0f\n", 
                 finalDistance, fuzzyInput.confidence, fuzzyOutput.vibrationIntensity);
    
    return true;
  }
  
  // Getter methods
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  float getLastDistance() const { return lastValidDistance; }
  
  // Get fuzzy output for vibration motor
  FuzzyOutput getLastFuzzyOutput() {
    if (!hasStableHistory) {
      return {0.0, 0, 0.0}; // Safe default
    }
    
    FuzzyInput input = calculateFuzzyInputs(lastValidDistance);
    return processFuzzyLogic(input);
  }

private:
  /**
   * STEP 1: Enhanced Raw Distance Measurement with Extended Timeout
   * Single ultrasonic measurement supporting very large distances
   */
float measureSingleDistance() {
  // Trigger pulse
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  // Adaptive timeout based on expected distance
  unsigned long adaptiveTimeout;
  if (lastValidDistance > 0 && lastValidDistance < 100) {
    adaptiveTimeout = 20000; // 20ms for close distances
  } else if (lastValidDistance > 0 && lastValidDistance < 300) {
    adaptiveTimeout = 40000; // 40ms for medium distances  
  } else {
    adaptiveTimeout = 120000; // 120ms for far/unknown distances
  }
  
  unsigned long duration = pulseIn(echoPin, HIGH, adaptiveTimeout);
  
  if (duration == 0) {
    return -1.0;
  }
  
  float distance = (duration * 0.0343) / 2.0;
  return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) ? distance : -1.0;
}
  
  /**
   * STEP 2: Process Multiple Readings with Outlier Filtering
   * Enhanced processing for stability at large distances
   */
float processReadings(float readings[], int validCount) {
  if (validCount == 0) {
    return -1.0;
  }
  
  if (validCount <= 2) {
    // For 1-2 readings, just return the first valid one
    for (int i = 0; i < SENSOR_RETRY_COUNT; i++) {
      if (readings[i] > 0) {
        return readings[i];
      }
    }
  }
  
  // For 3+ readings, use simple median without complex outlier detection
  float validReadings[SENSOR_RETRY_COUNT];
  int validIndex = 0;
  
  for (int i = 0; i < SENSOR_RETRY_COUNT; i++) {
    if (readings[i] > 0) {
      validReadings[validIndex++] = readings[i];
    }
  }
  
  // Simple bubble sort for median
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = 0; j < validCount - i - 1; j++) {
      if (validReadings[j] > validReadings[j + 1]) {
        float temp = validReadings[j];
        validReadings[j] = validReadings[j + 1];
        validReadings[j + 1] = temp;
      }
    }
  }
  
  return validReadings[validCount / 2]; // Return median
}
  
  /**
   * STEP 3: Enhanced Fuzzy Logic Inputs with Stability Focus
   * Improved confidence calculation for smoother response
   */
    FuzzyInput calculateFuzzyInputs(float distance) {
      FuzzyInput input;
      input.distance = distance;
      
      // Enhanced signal stability calculation with caching
      if (hasStableHistory) {
        float mean, variance;
        
        // Use cached values if history hasn't changed
        if (!historyChanged && cachedMean >= 0) {
          mean = cachedMean;
          variance = cachedVariance;
        } else {
          mean = calculateMean();
          variance = 0.0;
          int validHistoryCount = 0;
          
          for (int i = 0; i < WINDOW_SIZE; i++) {
            if (distanceHistory[i] > 0) {
              float diff = distanceHistory[i] - mean;
              variance += diff * diff;
              validHistoryCount++;
            }
          }
          
          if (validHistoryCount > 0) {
            variance = variance / validHistoryCount;
          }
          
          // Cache the results
          cachedMean = mean;
          cachedVariance = variance;
          historyChanged = false;
        }
        
        input.stability = sqrt(variance);
      } else {
        input.stability = 25.0;
      }
    
    // Improved confidence calculation with distance-aware scaling
    // Larger distances naturally have more variance, so adjust accordingly
    float distanceNormalizationFactor = max(50.0f, min(distance, D_MAX));
    input.confidence = 1.0 / (1.0 + input.stability / distanceNormalizationFactor);
    
    // Apply confidence boost for stable readings
    if (input.stability < 10.0) {
      input.confidence = min(1.0f, input.confidence * 1.2f); // Boost stable readings
    }
    
    // Clamp confidence to reasonable range
    input.confidence = constrain(input.confidence, MIN_CONFIDENCE, 1.0);
    
    return input;
  }
  
  /**
   * STEP 4: Smooth Fuzzy Logic Processing (Enhanced Mamdani Method)
   * Implements gentle, progressive fuzzy inference system
   */
  FuzzyOutput processFuzzyLogic(const FuzzyInput& input) {
    // Step 4.1: Smooth Fuzzification - Calculate membership degrees
    
    // Distance membership functions (smooth, overlapping)
    float mu_immediate = calculateImmediateMembership(input.distance);
    float mu_close = calculateCloseMembership(input.distance);
    float mu_near = calculateNearMembership(input.distance);
    float mu_moderate = calculateModerateMembership(input.distance);
    float mu_safe = calculateSafeMembership(input.distance);
    
    // Enhanced confidence membership functions
    float mu_high_conf = input.confidence; // Linear instead of quadratic for gentleness
    float mu_medium_conf = 1.0 - abs(input.confidence - 0.5) * 2.0; // Peak at 0.5
    float mu_low_conf = 1.0 - input.confidence;
    
    // Clamp confidence memberships
    mu_high_conf = max(0.0f, mu_high_conf);
    mu_medium_conf = max(0.0f, mu_medium_conf);
    mu_low_conf = max(0.0f, mu_low_conf);
    
    // Step 4.2: Enhanced Rule Evaluation (Smooth AND operations)
    // Using product instead of minimum for smoother combinations
    float alpha1 = mu_immediate * mu_high_conf;    // R1: IMMEDIATE + HIGH_CONF
    float alpha2 = mu_immediate * mu_medium_conf;  // R2: IMMEDIATE + MED_CONF
    float alpha3 = mu_immediate * mu_low_conf;     // R3: IMMEDIATE + LOW_CONF
    
    float alpha4 = mu_close * mu_high_conf;        // R4: CLOSE + HIGH_CONF
    float alpha5 = mu_close * mu_medium_conf;      // R5: CLOSE + MED_CONF
    float alpha6 = mu_close * mu_low_conf;         // R6: CLOSE + LOW_CONF
    
    float alpha7 = mu_near * mu_high_conf;         // R7: NEAR + HIGH_CONF
    float alpha8 = mu_near * mu_medium_conf;       // R8: NEAR + MED_CONF
    
    float alpha9 = mu_moderate * mu_high_conf;     // R9: MODERATE + HIGH_CONF
    
    float alpha10 = mu_safe;                       // R10: SAFE (always gentle)
    
    // Step 4.3: Smooth Output Aggregation and Defuzzification
    float intensitySum = 0.0;
    float weightSum = 0.0;
    
    // Progressive rule outputs (smooth escalation)
    if (alpha1 > 0) { intensitySum += alpha1 * VIB_URGENT_MAX;   weightSum += alpha1; }  // Urgent but not harsh
    if (alpha2 > 0) { intensitySum += alpha2 * VIB_FIRM_MAX;     weightSum += alpha2; }  // Firm indication
    if (alpha3 > 0) { intensitySum += alpha3 * VIB_MODERATE_MAX; weightSum += alpha3; }  // Moderate with uncertainty
    
    if (alpha4 > 0) { intensitySum += alpha4 * VIB_FIRM_MAX;     weightSum += alpha4; }  // Close + confident
    if (alpha5 > 0) { intensitySum += alpha5 * VIB_MODERATE_MAX; weightSum += alpha5; }  // Close + medium conf
    if (alpha6 > 0) { intensitySum += alpha6 * VIB_LIGHT_MAX;    weightSum += alpha6; }  // Close + uncertain
    
    if (alpha7 > 0) { intensitySum += alpha7 * VIB_MODERATE_MAX; weightSum += alpha7; }  // Near + confident
    if (alpha8 > 0) { intensitySum += alpha8 * VIB_LIGHT_MAX;    weightSum += alpha8; }  // Near + medium conf
    
    if (alpha9 > 0) { intensitySum += alpha9 * VIB_LIGHT_MAX;    weightSum += alpha9; }  // Moderate distance
    
    if (alpha10 > 0) { intensitySum += alpha10 * VIB_OFF_MAX;    weightSum += alpha10; } // Safe - minimal
    
    FuzzyOutput output;
    
    // Final intensity (weighted average defuzzification)
    if (weightSum > 0) {
      output.vibrationIntensity = intensitySum / weightSum;
    } else {
      output.vibrationIntensity = 0.0;
    }
    
    // Smooth intensity scaling - apply gentle curve
    output.vibrationIntensity = sqrt(output.vibrationIntensity / 255.0) * 255.0;
    
    // Determine smooth vibration pattern based on distance zones
    if (mu_immediate > 0.7) {
      output.vibrationPattern = 3; // Continuous for very close
      output.vibrationFrequency = 25.0; // Moderate frequency
    } else if (mu_close > 0.5) {
      output.vibrationPattern = 2; // Fast pulse for close
      output.vibrationFrequency = 20.0;
    } else if (mu_near > 0.3) {
      output.vibrationPattern = 1; // Slow pulse for near
      output.vibrationFrequency = 15.0;
    } else if (output.vibrationIntensity > VIB_OFF_MAX) {
      output.vibrationPattern = 1; // Very slow pulse for distant
      output.vibrationFrequency = 10.0;
    } else {
      output.vibrationPattern = 0; // Off
      output.vibrationFrequency = 0.0;
    }
    
    return output;
  }
  
  /**
   * Smooth Distance Membership Functions
   * All functions use gentle curves and smooth transitions
   */
  float calculateImmediateMembership(float d) {
    if (d <= IMMEDIATE_LOW) {
      return 1.0;
    } else if (d <= IMMEDIATE_HIGH) {
      // Smooth exponential decay for gentle response
      float ratio = (d - IMMEDIATE_LOW) / (IMMEDIATE_HIGH - IMMEDIATE_LOW);
      return exp(-1.5 * ratio); // Gentler decay
    } else {
      return 0.0;
    }
  }
  
  float calculateCloseMembership(float d) {
    if (d < CLOSE_LOW) {
      return 0.0;
    } else if (d <= CLOSE_MID) {
      // Smooth rise using sine curve
      float ratio = (d - CLOSE_LOW) / (CLOSE_MID - CLOSE_LOW);
      return sin(ratio * PI / 2.0); // Smooth S-curve rise
    } else if (d <= CLOSE_HIGH) {
      // Smooth fall using cosine curve
      float ratio = (d - CLOSE_MID) / (CLOSE_HIGH - CLOSE_MID);
      return cos(ratio * PI / 2.0); // Smooth S-curve fall
    } else {
      return 0.0;
    }
  }
  
  float calculateNearMembership(float d) {
    if (d < NEAR_LOW) {
      return 0.0;
    } else if (d <= NEAR_MID) {
      // Gentle quadratic rise
      float ratio = (d - NEAR_LOW) / (NEAR_MID - NEAR_LOW);
      return ratio * ratio; // Smooth acceleration
    } else if (d <= NEAR_HIGH) {
      // Gentle quadratic fall
      float ratio = (d - NEAR_MID) / (NEAR_HIGH - NEAR_MID);
      return 1.0 - ratio * ratio; // Smooth deceleration
    } else {
      return 0.0;
    }
  }
  
  float calculateModerateMembership(float d) {
    if (d < MODERATE_LOW) {
      return 0.0;
    } else if (d <= MODERATE_MID) {
      // Very gentle linear rise
      return (d - MODERATE_LOW) / (MODERATE_MID - MODERATE_LOW);
    } else if (d <= MODERATE_HIGH) {
      // Very gentle linear fall
      return (MODERATE_HIGH - d) / (MODERATE_HIGH - MODERATE_MID);
    } else {
      return 0.0;
    }
  }
  
  float calculateSafeMembership(float d) {
    if (d <= SAFE_LOW) {
      return 0.0;
    } else if (d <= SAFE_HIGH) {
      // Gentle rise to safe zone
      float ratio = (d - SAFE_LOW) / (SAFE_HIGH - SAFE_LOW);
      return ratio; // Linear rise for predictability
    } else {
      return 1.0; // Fully safe at large distances
    }
  }
  
  /**
   * History Management (unchanged)
   */
void addToHistory(float distance) {
  distanceHistory[historyIndex] = distance;
  historyIndex = (historyIndex + 1) % WINDOW_SIZE;
  
  if (historyCount < WINDOW_SIZE) {
    historyCount++;
  } else {
    hasStableHistory = true;
  }
  
  // Mark history as changed for cache invalidation
  historyChanged = true;
}
  
  float calculateMean() {
    float sum = 0.0;
    int count = 0;
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
      if (distanceHistory[i] > 0) {
        sum += distanceHistory[i];
        count++;
      }
    }
    
    return (count > 0) ? (sum / count) : 0.0;
  }
  
  float inferDistanceFromHistory() {
    if (!hasStableHistory) {
      return 50.0; // More optimistic default during initialization
    }
    
    return calculateMean();
  }
  
  /**
   * JSON Data Population (Database Compatible) - Enhanced
   */
  void populateJsonData(JsonObject& data, float distance, const FuzzyInput& input, 
                       const FuzzyOutput& output, int validCount, unsigned long timestamp) {
    // Core distance data (used by frontend)
    data["distance_cm"] = round(distance * 10) / 10.0;
    data["distance_unit"] = "centimeters";
    
    // Enhanced distance classification
    if (distance > LARGE_DISTANCE_THRESHOLD) {
      data["distance_classification"] = "large";
    } else if (distance <= IMMEDIATE_HIGH) {
      data["distance_classification"] = "immediate";
    } else if (distance <= CLOSE_HIGH) {
      data["distance_classification"] = "close";
    } else if (distance <= NEAR_HIGH) {
      data["distance_classification"] = "near";
    } else if (distance <= MODERATE_HIGH) {
      data["distance_classification"] = "moderate";
    } else {
      data["distance_classification"] = "safe";
    }
    
    // Enhanced sensor metadata
    data["sensor_type"] = "HC-SR04";
    data["read_time"] = timestamp;
    data["max_range_cm"] = MAX_DISTANCE;
    data["min_range_cm"] = MIN_DISTANCE;
    data["large_distance_threshold"] = LARGE_DISTANCE_THRESHOLD;
    data["supports_large_distances"] = true;
    data["extended_timeout"] = true;
    
    // Status and confidence
    if (validCount > 0) {
      data["status"] = "valid_readings";
    } else {
      data["status"] = "inferred_from_history";
    }
    
    data["confidence"] = round(input.confidence * 100) / 100.0;
    data["consecutive_errors"] = consecutiveErrors;
    data["data_age_ms"] = timestamp - lastSuccessfulReadTime;
    data["stale_data_warning"] = (timestamp - lastSuccessfulReadTime) > 5000;
    
    // Enhanced fuzzy logic data
    JsonObject fuzzy = data.createNestedObject("fuzzy_logic");
    fuzzy["signal_stability"] = round(input.stability * 10) / 10.0;
    fuzzy["vibration_intensity"] = round(output.vibrationIntensity);
    fuzzy["vibration_pattern"] = output.vibrationPattern;
    fuzzy["vibration_frequency"] = round(output.vibrationFrequency * 10) / 10.0;
    fuzzy["algorithm"] = "smooth_progressive";
    fuzzy["enabled"] = true;
    
    // Enhanced sensor state
    if (!hasStableHistory) {
      data["sensor_state"] = "initializing";
    } else if (consecutiveErrors > 3) {
      data["sensor_state"] = "unstable";
    } else {
      data["sensor_state"] = "normal";
    }
    
    // Comfort metrics
    JsonObject comfort = data.createNestedObject("comfort_metrics");
    comfort["intensity_level"] = output.vibrationIntensity <= VIB_SUBTLE_MAX ? "subtle" :
                                output.vibrationIntensity <= VIB_LIGHT_MAX ? "light" :
                                output.vibrationIntensity <= VIB_MODERATE_MAX ? "moderate" :
                                output.vibrationIntensity <= VIB_FIRM_MAX ? "firm" : "urgent";
    comfort["escalation_free"] = true; // No time-based escalation
    comfort["smooth_transitions"] = true;
  }
  
  void populateJsonFromCache(JsonObject& data, unsigned long timestamp) {
    // Provide cached data when rate limited
    float cachedDistance = lastValidDistance > 0 ? lastValidDistance : 50.0;
    FuzzyInput cachedInput = calculateFuzzyInputs(cachedDistance);
    FuzzyOutput cachedOutput = processFuzzyLogic(cachedInput);
    
    populateJsonData(data, cachedDistance, cachedInput, cachedOutput, 1, timestamp);
    data["status"] = "cached_reading";
  }
};

#endif