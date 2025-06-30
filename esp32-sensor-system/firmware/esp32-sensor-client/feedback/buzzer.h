#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "../config.h"

/**
 * Enhanced Buzzer Audio Feedback Module with Binary and Fuzzy Logic Support
 * 
 * Provides audio alerts for:
 * 1. BINARY infrared sensor (hole/drop detection) - NEW LOGIC: Alert when NO surface
 * 2. FUZZY water sensor (unchanged) - Graduated responses based on water detection
 * 
 * BINARY LOGIC FOR INFRARED:
 * - Surface Present = NO ALERT (buzzer inactive)
 * - No Surface/Hole/Drop = ALERT (buzzer active)
 * 
 * Features:
 * - Binary pattern support (hole detection with reversed logic)
 * - Fuzzy logic pattern support (water detection - unchanged)
 * - Alert priority management
 * - Non-blocking pattern playback
 * - Configurable frequencies and durations
 * - Smart pattern interruption for urgent alerts
 */
class Buzzer {
public:
  // Alert types for different sensors
  enum AlertType {
    ALERT_NONE = 0,
    ALERT_HOLE_BINARY = 1,    // Binary infrared sensor (reversed logic)
    ALERT_WATER_FUZZY = 2     // Fuzzy logic water sensor (unchanged)
  };
  
  // Predefined beep patterns
  enum BeepPattern {
    PATTERN_NONE = 0,
    PATTERN_HOLE_BINARY = 1,  // Binary hole detection pattern
    PATTERN_WATER = 2,        // Water detection pattern (unchanged)
    PATTERN_SINGLE_BEEP = 3,
    PATTERN_DOUBLE_BEEP = 4,
    PATTERN_CONTINUOUS = 5,
    PATTERN_PULSED = 6
  };
  
  // Fuzzy logic output structure (unchanged for water sensor)
  struct FuzzyAlertConfig {
    int alertLevel;        // 0-3 (Silent, Gentle, Moderate, Urgent)
    int frequency;         // Hz for buzzer tone
    String patternName;    // Pattern identifier
    float confidence;      // 0.0-1.0 confidence level
  };

private:
  bool initialized = false;
  bool enabled = BUZZER_ENABLED;
  int buzzerPin = BUZZER_PIN;
  
  // Pattern specifications
  struct PatternSpec {
    int beepCount;
    unsigned long beepDuration;
    unsigned long pauseDuration;
    int pwmValue;      // PWM value (0-255 for analogWrite)
    int priority;      // Higher number = higher priority
  };
  
  // Enhanced pattern definitions
  PatternSpec patterns[7] = {
    {0, 0, 0, 0, 0},                      // PATTERN_NONE
    {BUZZER_PATTERN_HOLE_BEEPS, BUZZER_PATTERN_HOLE_DURATION, 200, 220, 15},  // PATTERN_HOLE_BINARY (highest priority)
    {BUZZER_PATTERN_WATER_BEEPS, BUZZER_PATTERN_WATER_DURATION, 100, 150, 5}, // PATTERN_WATER (unchanged)
    {1, 300, 0, 180, 3},                  // PATTERN_SINGLE_BEEP (low priority)
    {2, 200, 150, 160, 4},                // PATTERN_DOUBLE_BEEP (low-medium priority)
    {1, 2000, 0, 140, 8},                 // PATTERN_CONTINUOUS (high priority)
    {6, 100, 200, 120, 6}                 // PATTERN_PULSED (medium-high priority)
  };
  
  // State tracking for pattern playback
  bool isPatternActive = false;
  BeepPattern currentPattern = PATTERN_NONE;
  AlertType currentAlertType = ALERT_NONE;
  int currentBeepIndex = 0;
  bool isBeepActive = false;
  unsigned long stateStartTime = 0;
  int currentPWMValue = 0;
  
  // Priority and activation control
  unsigned long lastActivationTime = 0;
  unsigned long lastHoleAlertTime = 0;
  unsigned long lastWaterAlertTime = 0;
  static const unsigned long MIN_ACTIVATION_INTERVAL = 300;   // Faster response for binary sensor
  static const unsigned long MIN_HOLE_ALERT_INTERVAL = 800;  // Reduced cooldown for binary sensor
  static const unsigned long MIN_WATER_ALERT_INTERVAL = 2000; // Water sensor cooldown (unchanged)
  
  // Binary infrared sensor state tracking
  bool lastSurfaceState = true;  // TRUE = surface present, FALSE = hole detected
  bool continuousAlertActive = false;
  unsigned long continuousAlertStartTime = 0;
  static const unsigned long MAX_CONTINUOUS_ALERT_DURATION = 5000; // 5 seconds max continuous alert
  
  // Fuzzy logic integration (unchanged)
  FuzzyAlertConfig lastFuzzyConfig = {0, 0, "silent", 0.0};
  bool fuzzyAlertActive = false;
  
public:
  Buzzer() {}
  
  /**
   * Initialize the enhanced buzzer hardware
   */
  bool begin() {
    if (!enabled) {
      Serial.println("Buzzer: Disabled in configuration");
      return true;
    }
    
    Serial.println("Buzzer: Initializing binary + fuzzy audio feedback system...");
    
    // Validate pin configuration
    if (buzzerPin < 0) {
      Serial.println("Buzzer: Invalid pin configuration");
      return false;
    }
    
    // Configure pin as output
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
    
    Serial.printf("Buzzer: Configured on GPIO %d\n", buzzerPin);
    
    // Audio test with different tones
    Serial.println("Buzzer: Performing enhanced audio test...");
    testAudioSystem();
    
    initialized = true;
    Serial.println("Buzzer: Binary + Fuzzy initialization successful");
    Serial.println("Buzzer: BINARY LOGIC - Surface Present = NO alert, No Surface = ALERT");
    return true;
  }
  
  /**
   * UPDATED: Binary alert for infrared sensor with REVERSED LOGIC
   * surfaceDetected = TRUE means surface is present (NO alert)
   * surfaceDetected = FALSE means no surface/hole detected (ALERT active)
   */
  bool alertForHole(bool surfaceDetected) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Binary logic implementation
    bool shouldAlert = !surfaceDetected;  // Alert when NO surface detected
    
    // State change detection
    if (surfaceDetected != lastSurfaceState) {
      lastSurfaceState = surfaceDetected;
      
      if (shouldAlert) {
        // Surface lost - activate alert
        Serial.println("Buzzer: BINARY ALERT ACTIVATED - No surface detected!");
        lastHoleAlertTime = currentTime;
        continuousAlertActive = true;
        continuousAlertStartTime = currentTime;
        return playAlertWithPriority(ALERT_HOLE_BINARY, PATTERN_HOLE_BINARY);
      } else {
        // Surface found - deactivate alert
        Serial.println("Buzzer: BINARY ALERT DEACTIVATED - Surface detected!");
        continuousAlertActive = false;
        stop(); // Immediately stop buzzer when surface is detected
        return true;
      }
    }
    
    // Handle continuous alert while no surface is detected
    if (shouldAlert && continuousAlertActive) {
      // Check for maximum continuous alert duration
      if (currentTime - continuousAlertStartTime > MAX_CONTINUOUS_ALERT_DURATION) {
        Serial.println("Buzzer: Maximum continuous alert duration reached");
        continuousAlertActive = false;
        stop();
        return false;
      }
      
      // Check cooldown for repeated alerts
      if (currentTime - lastHoleAlertTime < MIN_HOLE_ALERT_INTERVAL) {
        return false;
      }
      
      // Reactivate alert pattern
      lastHoleAlertTime = currentTime;
      return playAlertWithPriority(ALERT_HOLE_BINARY, PATTERN_HOLE_BINARY);
    }
    
    return false;
  }
  
  /**
   * DEPRECATED: Legacy method name for backward compatibility
   * This now uses the correct binary logic
   */
  bool alertForHoleDetection(bool holeDetected) {
    // Convert old logic to new logic
    bool surfaceDetected = !holeDetected;
    return alertForHole(surfaceDetected);
  }
  
  /**
   * Fuzzy logic alert for water sensor (UNCHANGED)
   */
  bool alertForWaterFuzzy(const FuzzyAlertConfig& fuzzyConfig) {
    if (!enabled || !initialized) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Store fuzzy configuration
    lastFuzzyConfig = fuzzyConfig;
    
    // Silent alert level means no sound
    if (fuzzyConfig.alertLevel == 0) {
      fuzzyAlertActive = false;
      return true;
    }
    
    // Check cooldown for water alerts
    if (currentTime - lastWaterAlertTime < MIN_WATER_ALERT_INTERVAL) {
      return false;
    }
    
    // Determine pattern based on fuzzy output
    BeepPattern pattern = mapFuzzyToPattern(fuzzyConfig);
    
    // Set custom PWM value based on frequency
    if (pattern != PATTERN_NONE) {
      // Override default PWM value with frequency-based value
      int patternIndex = (int)pattern;
      if (patternIndex < 7) {
        patterns[patternIndex].pwmValue = mapFrequencyToPWM(fuzzyConfig.frequency);
      }
    }
    
    lastWaterAlertTime = currentTime;
    fuzzyAlertActive = true;
    
    return playAlertWithPriority(ALERT_WATER_FUZZY, pattern);
  }
  
  /**
   * Simplified water alert for backward compatibility (UNCHANGED)
   */
  bool alertForWater(bool waterDetected) {
    if (waterDetected) {
      FuzzyAlertConfig basicConfig = {2, 1000, "double_beep", 1.0};
      return alertForWaterFuzzy(basicConfig);
    }
    return false;
  }
  
  /**
   * Play predefined patterns
   */
  bool playPattern(BeepPattern pattern) {
    return playAlertWithPriority(ALERT_NONE, pattern);
  }
  
  bool playHoleAlert() {
    return playAlertWithPriority(ALERT_HOLE_BINARY, PATTERN_HOLE_BINARY);
  }
  
  bool playWaterAlert() {
    return playAlertWithPriority(ALERT_WATER_FUZZY, PATTERN_WATER);
  }
  
  bool playSingleBeep() {
    return playPattern(PATTERN_SINGLE_BEEP);
  }
  
  bool playDoubleBeep() {
    return playPattern(PATTERN_DOUBLE_BEEP);
  }
  
  /**
   * Force stop all buzzer activity
   */
  void stop() {
    if (!enabled || !initialized) {
      return;
    }
    
    analogWrite(buzzerPin, 0);
    isPatternActive = false;
    isBeepActive = false;
    currentPattern = PATTERN_NONE;
    currentAlertType = ALERT_NONE;
    continuousAlertActive = false;
    fuzzyAlertActive = false;
    Serial.println("Buzzer: All activity stopped");
  }
  
  /**
   * Update buzzer state - must be called regularly in main loop
   */
  void update() {
    if (!enabled || !initialized || !isPatternActive) {
      return;
    }
    
    unsigned long currentTime = millis();
    PatternSpec& spec = patterns[(int)currentPattern];
    
    if (isBeepActive) {
      // Check if current beep should end
      if (currentTime - stateStartTime >= spec.beepDuration) {
        // End beep
        analogWrite(buzzerPin, 0);
        isBeepActive = false;
        stateStartTime = currentTime;
        
        // Check if pattern is complete
        if (currentBeepIndex >= spec.beepCount - 1) {
          // Pattern complete
          isPatternActive = false;
          currentPattern = PATTERN_NONE;
          currentAlertType = ALERT_NONE;
          Serial.println("Buzzer: Pattern completed");
        } else {
          // Move to next beep after pause
          currentBeepIndex++;
        }
      }
    } else {
      // Check if pause should end (start next beep)
      if (currentTime - stateStartTime >= spec.pauseDuration) {
        // Start next beep
        analogWrite(buzzerPin, currentPWMValue);
        isBeepActive = true;
        stateStartTime = currentTime;
      }
    }
  }
  
  /**
   * Get current binary sensor state
   */
  bool isSurfaceDetected() const {
    return lastSurfaceState;
  }
  
  bool isHoleDetected() const {
    return !lastSurfaceState;  // Hole detected when no surface
  }
  
  bool isContinuousAlertActive() const {
    return continuousAlertActive;
  }
  
  /**
   * Get current fuzzy alert configuration (UNCHANGED)
   */
  FuzzyAlertConfig getFuzzyConfig() const {
    return lastFuzzyConfig;
  }
  
  /**
   * Check if buzzer is currently active
   */
  bool isActive() const {
    return isPatternActive;
  }
  
  /**
   * Get current alert type
   */
  AlertType getCurrentAlertType() const {
    return currentAlertType;
  }
  
  /**
   * Enable/disable buzzer
   */
  void setEnabled(bool enable) {
    enabled = enable;
    if (!enabled) {
      stop();
    }
  }
  
  bool isEnabled() const {
    return enabled;
  }

private:
  /**
   * Play alert with priority management
   */
  bool playAlertWithPriority(AlertType alertType, BeepPattern pattern) {
    if (!enabled || !initialized || pattern == PATTERN_NONE) {
      return false;
    }
    
    unsigned long currentTime = millis();
    
    // Check minimum activation interval
    if (currentTime - lastActivationTime < MIN_ACTIVATION_INTERVAL) {
      return false;
    }
    
    // Priority management - Binary hole alerts have highest priority
    if (isPatternActive) {
      int newPriority = patterns[(int)pattern].priority;
      int currentPriority = patterns[(int)currentPattern].priority;
      
      // Binary hole alerts can always interrupt other patterns
      if (alertType == ALERT_HOLE_BINARY) {
        Serial.printf("Buzzer: BINARY HOLE ALERT interrupting pattern %d\n", (int)currentPattern);
      } else if (newPriority <= currentPriority) {
        return false;
      } else {
        Serial.printf("Buzzer: Pattern %d (priority %d) interrupting pattern %d (priority %d)\n",
                     (int)pattern, newPriority, (int)currentPattern, currentPriority);
      }
    }
    
    // Start new pattern
    currentPattern = pattern;
    currentAlertType = alertType;
    currentBeepIndex = 0;
    isBeepActive = true;
    isPatternActive = true;
    stateStartTime = currentTime;
    lastActivationTime = currentTime;
    
    // Set PWM value for this pattern
    currentPWMValue = patterns[(int)pattern].pwmValue;
    
    // Start first beep
    analogWrite(buzzerPin, currentPWMValue);
    
    Serial.printf("Buzzer: Started pattern %d for alert type %d (PWM: %d)\n", 
                 (int)pattern, (int)alertType, currentPWMValue);
    
    return true;
  }
  
/**
   * Map fuzzy output to appropriate beep pattern (UNCHANGED)
   */
  BeepPattern mapFuzzyToPattern(const FuzzyAlertConfig& fuzzyConfig) {
    switch (fuzzyConfig.alertLevel) {
      case 0:
        return PATTERN_NONE;
      case 1:
        return PATTERN_SINGLE_BEEP;
      case 2:
        return PATTERN_DOUBLE_BEEP;
      case 3:
        return PATTERN_CONTINUOUS;
      default:
        return PATTERN_WATER; // Default water pattern
    }
  }
  
  /**
   * Map frequency to PWM value for analog buzzer control
   */
  int mapFrequencyToPWM(int frequency) {
    // Map frequency range to PWM values (0-255)
    // Lower frequencies = lower PWM values for softer tones
    if (frequency <= 0) return 0;
    if (frequency <= 500) return 120;
    if (frequency <= 1000) return 160;
    if (frequency <= 1500) return 200;
    return 240; // Higher frequencies
  }
  
  /**
   * Test audio system with different tones
   */
  void testAudioSystem() {
    Serial.println("Buzzer: Testing audio system...");
    
    // Test different PWM values
    int testPWMValues[] = {120, 160, 200, 240};
    int testDurations[] = {200, 200, 200, 200};
    
    for (int i = 0; i < 4; i++) {
      Serial.printf("Buzzer: Test tone %d (PWM: %d)\n", i + 1, testPWMValues[i]);
      analogWrite(buzzerPin, testPWMValues[i]);
      delay(testDurations[i]);
      analogWrite(buzzerPin, 0);
      delay(100);
    }
    
    Serial.println("Buzzer: Audio test completed");
  }
  
  /**
   * Get status information for debugging
   */
  void printStatus() {
    if (!enabled) {
      Serial.println("Buzzer: Status - DISABLED");
      return;
    }
    
    Serial.println("Buzzer: Status Report:");
    Serial.printf("  Initialized: %s\n", initialized ? "YES" : "NO");
    Serial.printf("  Pin: GPIO %d\n", buzzerPin);
    Serial.printf("  Pattern Active: %s\n", isPatternActive ? "YES" : "NO");
    Serial.printf("  Current Pattern: %d\n", (int)currentPattern);
    Serial.printf("  Current Alert Type: %d\n", (int)currentAlertType);
    Serial.printf("  Beep Active: %s\n", isBeepActive ? "YES" : "NO");
    Serial.printf("  Current PWM: %d\n", currentPWMValue);
    Serial.printf("  Fuzzy Alert Active: %s\n", fuzzyAlertActive ? "YES" : "NO");
    Serial.printf("  Binary Surface State: %s\n", lastSurfaceState ? "SURFACE DETECTED" : "NO SURFACE/HOLE");
    Serial.printf("  Continuous Alert: %s\n", continuousAlertActive ? "ACTIVE" : "INACTIVE");
    Serial.printf("  Last Fuzzy Config: Level=%d, Freq=%d Hz, Pattern=%s, Confidence=%.2f\n",
                 lastFuzzyConfig.alertLevel, lastFuzzyConfig.frequency, 
                 lastFuzzyConfig.patternName.c_str(), lastFuzzyConfig.confidence);
  }
};

#endif