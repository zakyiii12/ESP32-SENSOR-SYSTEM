//vibration_motor.h
#ifndef VIBRATION_MOTOR_H
#define VIBRATION_MOTOR_H

#include <Arduino.h>
#include "../config.h"

/**
 * Enhanced Vibration Motor Feedback Module with Fuzzy Logic Integration
 * 
 * Provides intelligent tactile feedback based on fuzzy logic classification from
 * ultrasonic sensor data. Supports graduated intensity, pattern-based feedback,
 * and frequency modulation for sophisticated user experience.
 * 
 * Features:
 * - PWM-controlled vibration intensity (0-255)
 * - Multiple vibration patterns (off, slow pulse, fast pulse, continuous)
 * - Frequency modulation for compatible actuators
 * - Smooth intensity transitions to prevent jarring changes
 * - Non-blocking pattern generation using state machine
 * - Emergency stop and force override capabilities
 * - Performance monitoring and debugging output
 */
class VibrationMotor {
public:
  // Vibration patterns matching fuzzy logic output
  enum VibrationPattern {
    PATTERN_OFF = 0,        // No vibration
    PATTERN_SLOW_PULSE = 1, // 200ms on, 800ms off - Gentle warning
    PATTERN_FAST_PULSE = 2, // 100ms on, 400ms off - Active warning  
    PATTERN_CONTINUOUS = 3  // Steady vibration - Immediate danger
  };
  
  // Vibration state for pattern generation
  enum VibrationState {
    STATE_INACTIVE,
    STATE_PULSE_ON,
    STATE_PULSE_OFF,
    STATE_CONTINUOUS
  };

private:
  bool initialized = false;
  bool enabled = VIBRATION_MOTOR_ENABLED;
  int motorPin = VIBRATION_MOTOR_PIN;
  int pwmChannel = 0; // PWM channel for ESP32
  
  // Current vibration parameters (from fuzzy logic)
  float currentIntensity = 0.0;      // PWM value [0-255]
  VibrationPattern currentPattern = PATTERN_OFF;
  float currentFrequency = 0.0;      // Hz [0-50]
  
  // Target parameters for smooth transitions
  float targetIntensity = 0.0;
  VibrationPattern targetPattern = PATTERN_OFF;
  float targetFrequency = 0.0;
  
  // State machine for pattern generation
  VibrationState vibrationState = STATE_INACTIVE;
  unsigned long stateStartTime = 0;
  unsigned long patternStartTime = 0;
  
  // Pattern timing constants (moved to private member variables)
  unsigned long slowPulseOnTime = 200;   // ms
  unsigned long slowPulseOffTime = 800;  // ms
  unsigned long fastPulseOnTime = 100;   // ms
  unsigned long fastPulseOffTime = 400;  // ms
  
  // Smooth transition parameters (moved to private member variables)
  float intensityTransitionRate = 15.0;    // PWM units per update
  unsigned long updateInterval = 10;       // ms (50Hz update rate)
  unsigned long lastUpdateTime = 0;
  
  // Performance tracking
  unsigned long totalActivationTime = 0;
  unsigned long activationCount = 0;
  unsigned long lastActivationTime = 0;
  
  // Safety limits (moved to private member variables)
  float maxSafeIntensity = 220.0;         // Prevent excessive vibration
  unsigned long maxContinuousTime = 5000; // 5 second safety limit
  unsigned long minRestTime = 100;        // Minimum time between pattern changes

public:
  VibrationMotor() {}
  
  /**
   * Initialize the vibration motor with PWM capability
   */
  bool begin() {
    if (!enabled) {
      Serial.println("VibrationMotor: Disabled in configuration");
      return true;
    }
    
    Serial.println("VibrationMotor: Initializing Fuzzy Logic Vibration Controller...");
    
    // Validate pin configuration
    if (motorPin < 0) {
      Serial.println("VibrationMotor: Invalid pin configuration");
      return false;
    }
    
    // Configure PWM channel for ESP32 (Arduino 2.x compatible)
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      // ESP32 Arduino Core 2.x
      if (!ledcAttach(motorPin, 1000, 8)) { // Pin, frequency, resolution
        Serial.println("VibrationMotor: Failed to attach PWM to pin");
        return false;
      }
    #else
      // ESP32 Arduino Core 1.x (legacy)
      ledcSetup(pwmChannel, 1000, 8); // Channel, frequency, resolution
      ledcAttachPin(motorPin, pwmChannel);
    #endif
    
    // Start with motor off
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 0);
    #else
      ledcWrite(pwmChannel, 0);
    #endif
    
    Serial.printf("VibrationMotor: PWM configured on GPIO %d (1kHz, 8-bit)\n", motorPin);
    
    // Perform calibration sequence
    Serial.println("VibrationMotor: Performing fuzzy logic calibration...");
    performCalibrationSequence();
    
    initialized = true;
    Serial.println("VibrationMotor: Fuzzy Logic initialization complete");
    return true;
  }
  
  /**
   * Set vibration parameters from fuzzy logic output
   * This is the main interface for the ultrasonic sensor
   */
  void setFuzzyOutput(float intensity, int pattern, float frequency) {
    if (!enabled || !initialized) {
      return;
    }
    
    // Validate and clamp inputs
    targetIntensity = constrain(intensity, 0.0, maxSafeIntensity);
    targetPattern = (VibrationPattern)constrain(pattern, 0, 3);
    targetFrequency = constrain(frequency, 0.0, 50.0);
    
    // Debug output for fuzzy logic integration
    Serial.printf("VibrationMotor: Fuzzy Input - Intensity: %.0f, Pattern: %d, Freq: %.1f Hz\n", 
                 targetIntensity, targetPattern, targetFrequency);
    
    // Handle pattern changes
    if (targetPattern != currentPattern) {
      handlePatternChange();
    }
  }
  
  /**
   * Update vibration motor state - MUST be called regularly in main loop
   * Handles pattern generation, smooth transitions, and safety monitoring
   */
  void update() {
    if (!enabled || !initialized) {
      return;
    }
    
    unsigned long currentTime = millis();
    
    // Rate limiting for smooth updates
    if (currentTime - lastUpdateTime < updateInterval) {
      return;
    }
    lastUpdateTime = currentTime;
    
    // Smooth intensity transitions
    updateIntensityTransition();
    
    // Pattern state machine
    updatePatternStateMachine(currentTime);
    
    // Safety monitoring
    performSafetyChecks(currentTime);
    
    // Apply final PWM output
    applyPWMOutput();
  }
  
  /**
   * Emergency stop - immediately halt all vibration
   */
  void emergencyStop() {
    if (!enabled || !initialized) {
      return;
    }
    
    targetIntensity = 0.0;
    currentIntensity = 0.0;
    targetPattern = PATTERN_OFF;
    currentPattern = PATTERN_OFF;
    vibrationState = STATE_INACTIVE;
    
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 0);
    #else
      ledcWrite(pwmChannel, 0);
    #endif
    
    Serial.println("VibrationMotor: EMERGENCY STOP activated");
  }
  
  /**
   * Get current vibration status for monitoring
   */
  struct VibrationStatus {
    float currentIntensity;
    VibrationPattern currentPattern;
    VibrationState currentState;
    float currentFrequency;
    bool isActive;
    unsigned long activationCount;
    unsigned long totalRunTime;
  };
  
  VibrationStatus getStatus() const {
    VibrationStatus status;
    status.currentIntensity = currentIntensity;
    status.currentPattern = currentPattern;
    status.currentState = vibrationState;
    status.currentFrequency = currentFrequency;
    status.isActive = (vibrationState != STATE_INACTIVE);
    status.activationCount = activationCount;
    status.totalRunTime = totalActivationTime;
    return status;
  }
  
  // Getter methods
  bool isEnabled() const { return enabled; }
  bool isInitialized() const { return initialized; }
  bool isActive() const { return vibrationState != STATE_INACTIVE; }
  float getCurrentIntensity() const { return currentIntensity; }
  VibrationPattern getCurrentPattern() const { return currentPattern; }

private:
  /**
   * Perform startup calibration to verify motor functionality
   */
  void performCalibrationSequence() {
    Serial.println("VibrationMotor: Testing intensity levels...");
    
    // Test low intensity
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 80);
    #else
      ledcWrite(pwmChannel, 80);
    #endif
    delay(200);
    
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 0);
    #else
      ledcWrite(pwmChannel, 0);
    #endif
    delay(200);
    
    // Test medium intensity
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 150);
    #else
      ledcWrite(pwmChannel, 150);
    #endif
    delay(200);
    
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 0);
    #else
      ledcWrite(pwmChannel, 0);
    #endif
    delay(200);
    
    // Test high intensity
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 220);
    #else
      ledcWrite(pwmChannel, 220);
    #endif
    delay(200);
    
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, 0);
    #else
      ledcWrite(pwmChannel, 0);
    #endif
    delay(300);
    
    Serial.println("VibrationMotor: Calibration sequence complete");
  }
  
  /**
   * Handle smooth intensity transitions to prevent jarring changes
   */
void updateIntensityTransition() {
  float difference = targetIntensity - currentIntensity;
  
  if (abs(difference) > 1.0) { // Increased threshold for faster response
    // Use exponential approach for faster response
    float transitionStep = abs(difference) * 0.3; // 30% of remaining difference
    transitionStep = max(intensityTransitionRate, transitionStep); // Minimum step size
    
    if (currentIntensity < targetIntensity) {
      currentIntensity += transitionStep;
    } else {
      currentIntensity -= transitionStep;
    }
  } else {
    currentIntensity = targetIntensity;
  }
  
  // Update frequency immediately for responsive feedback
  currentFrequency = targetFrequency;
}
  
  /**
   * Handle pattern changes with proper state transitions
   */
  void handlePatternChange() {
    unsigned long currentTime = millis();
    
    // Enforce minimum rest time between pattern changes
    if (currentTime - lastActivationTime < minRestTime) {
      return;
    }
    
    Serial.printf("VibrationMotor: Pattern change %d -> %d\n", currentPattern, targetPattern);
    
    currentPattern = targetPattern;
    patternStartTime = currentTime;
    lastActivationTime = currentTime;
    
    // Reset state machine
    switch (currentPattern) {
      case PATTERN_OFF:
        vibrationState = STATE_INACTIVE;
        break;
        
      case PATTERN_SLOW_PULSE:
      case PATTERN_FAST_PULSE:
        vibrationState = STATE_PULSE_ON;
        stateStartTime = currentTime;
        activationCount++;
        break;
        
      case PATTERN_CONTINUOUS:
        vibrationState = STATE_CONTINUOUS;
        stateStartTime = currentTime;
        activationCount++;
        break;
    }
  }
  
  /**
   * State machine for pattern generation
   */
  void updatePatternStateMachine(unsigned long currentTime) {
    switch (vibrationState) {
      case STATE_INACTIVE:
        // Do nothing - motor should be off
        break;
        
      case STATE_PULSE_ON:
        handlePulseOnState(currentTime);
        break;
        
      case STATE_PULSE_OFF:
        handlePulseOffState(currentTime);
        break;
        
      case STATE_CONTINUOUS:
        // Continuous vibration - just track time
        totalActivationTime += (currentTime - stateStartTime);
        stateStartTime = currentTime;
        break;
    }
  }
  
  /**
   * Handle pulse ON state (motor active during pulse)
   */
  void handlePulseOnState(unsigned long currentTime) {
    unsigned long onTime = (currentPattern == PATTERN_SLOW_PULSE) ? 
                          slowPulseOnTime : fastPulseOnTime;
    
    if (currentTime - stateStartTime >= onTime) {
      // Transition to OFF state
      vibrationState = STATE_PULSE_OFF;
      stateStartTime = currentTime;
      totalActivationTime += onTime;
    }
  }
  
  /**
   * Handle pulse OFF state (motor inactive during pulse)
   */
  void handlePulseOffState(unsigned long currentTime) {
    unsigned long offTime = (currentPattern == PATTERN_SLOW_PULSE) ? 
                           slowPulseOffTime : fastPulseOffTime;
    
    if (currentTime - stateStartTime >= offTime) {
      // Check if pattern is still active
      if (currentPattern == PATTERN_SLOW_PULSE || currentPattern == PATTERN_FAST_PULSE) {
        // Continue pulsing
        vibrationState = STATE_PULSE_ON;
        stateStartTime = currentTime;
      } else {
        // Pattern changed, go inactive
        vibrationState = STATE_INACTIVE;
      }
    }
  }
  
  /**
   * Safety monitoring and limits
   */
  void performSafetyChecks(unsigned long currentTime) {
    // Check for excessive continuous operation
    if (vibrationState == STATE_CONTINUOUS && 
        currentTime - patternStartTime > maxContinuousTime) {
      Serial.println("VibrationMotor: Safety limit reached - forcing pause");
      emergencyStop();
      return;
    }
    
    // Monitor total activation time for thermal protection
    if (totalActivationTime > 30000) { // 30 seconds total in recent period
      Serial.println("VibrationMotor: Thermal protection - reducing intensity");
      if (targetIntensity > 150) {
        targetIntensity = 150;
      }
    }
  }
  
  /**
   * Apply final PWM output based on current state
   */
  void applyPWMOutput() {
    float outputIntensity = 0.0;
    
    switch (vibrationState) {
      case STATE_INACTIVE:
        outputIntensity = 0.0;
        break;
        
      case STATE_PULSE_ON:
      case STATE_CONTINUOUS:
        outputIntensity = currentIntensity;
        break;
        
      case STATE_PULSE_OFF:
        outputIntensity = 0.0;
        break;
    }
    
    // Apply PWM
    int pwmValue = (int)constrain(outputIntensity, 0, 255);
    
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
      ledcWrite(motorPin, pwmValue);
    #else
      ledcWrite(pwmChannel, pwmValue);
    #endif
    
    // Optional: Frequency modulation for advanced actuators
    // This would require additional hardware support
    if (currentFrequency > 0 && vibrationState != STATE_INACTIVE) {
      // Could modify PWM frequency here for frequency-capable actuators
      // For ESP32 Arduino 2.x: ledcChangeFrequency would need pin parameter
      // For ESP32 Arduino 1.x: ledcChangeFrequency(pwmChannel, (uint32_t)currentFrequency);
    }
  }
};

#endif