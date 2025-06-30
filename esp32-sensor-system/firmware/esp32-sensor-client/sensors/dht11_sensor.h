#ifndef DHT11_SENSOR_H
#define DHT11_SENSOR_H

#include <DHT.h>
#include <ArduinoJson.h>
#include "../config.h"

/**
 * DHT11 Temperature and Humidity Sensor Module
 * 
 * This class handles all DHT11 sensor operations including:
 * - Sensor initialization and configuration
 * - Temperature and humidity readings
 * - Data validation and error handling
 * - JSON data formatting
 */
class DHT11Sensor {
private:
  DHT* dht = nullptr;              // DHT sensor object
  bool initialized = false;        // Initialization status
  bool enabled = DHT11_ENABLED;    // Enable/disable sensor
  
  // Sensor state tracking
  float lastTemperature = NAN;
  float lastHumidity = NAN;
  unsigned long lastReadTime = 0;
  int consecutiveErrors = 0;
  
  // Constants
  static const int MIN_READ_INTERVAL = 2000;  // DHT11 minimum read interval (2 seconds)
  static const int MAX_CONSECUTIVE_ERRORS = 5;
  
public:
  /**
   * Constructor
   */
  DHT11Sensor() {}
  
  /**
   * Destructor - cleanup DHT object
   */
  ~DHT11Sensor() {
    if (dht != nullptr) {
      delete dht;
      dht = nullptr;
    }
  }
  
  /**
   * Initialize the DHT11 sensor
   * @return true if initialization successful, false otherwise
   */
  bool begin() {
    if (!enabled) {
      Serial.println("DHT11: Sensor disabled in configuration");
      return true; // Not an error if disabled
    }
    
    Serial.println("DHT11: Initializing sensor...");
    
    // Create DHT object
    dht = new DHT(DHT11_DATA_PIN, DHT11);
    
    if (dht == nullptr) {
      Serial.println("DHT11: Failed to create DHT object");
      return false;
    }
    
    // Initialize DHT sensor
    dht->begin();
    
    // Optional: Enable power pin if configured
    if (DHT11_POWER_PIN >= 0) {
      pinMode(DHT11_POWER_PIN, OUTPUT);
      digitalWrite(DHT11_POWER_PIN, HIGH);
      delay(1000); // Wait for sensor to power up
    }
    
    // Perform initial test read
    delay(2000); // DHT11 needs time to stabilize
    float testTemp = dht->readTemperature();
    float testHum = dht->readHumidity();
    
    if (isnan(testTemp) || isnan(testHum)) {
      Serial.println("DHT11: Warning - Initial test read failed, but continuing...");
      // Don't fail initialization on first read error
    } else {
      Serial.printf("DHT11: Test read successful - Temp: %.1f°C, Humidity: %.1f%%\n", 
                    testTemp, testHum);
    }
    
    initialized = true;
    Serial.println("DHT11: Initialization completed");
    
    return true;
  }
  
  /**
   * Read sensor data and populate JSON object
   * @param data JsonObject to populate with sensor data
   * @return true if read successful, false otherwise
   */
  bool readSensor(JsonObject& data) {
    if (!enabled || !initialized) {
      return false;
    }
    
    // Check if enough time has passed since last read
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime < MIN_READ_INTERVAL) {
      // Use last valid readings if available
      if (!isnan(lastTemperature) && !isnan(lastHumidity)) {
        populateJsonData(data, lastTemperature, lastHumidity);
        return true;
      }
      return false;
    }
    
    // Attempt to read sensor with retries
    float temperature, humidity;
    bool readSuccess = false;
    
    for (int attempt = 0; attempt < SENSOR_RETRY_COUNT; attempt++) {
      temperature = dht->readTemperature();
      humidity = dht->readHumidity();
      
      if (!isnan(temperature) && !isnan(humidity)) {
        // Validate readings are within reasonable ranges
        if (isValidReading(temperature, humidity)) {
          readSuccess = true;
          break;
        } else {
          Serial.printf("DHT11: Invalid readings - Temp: %.1f, Humidity: %.1f (attempt %d)\n", 
                       temperature, humidity, attempt + 1);
        }
      } else {
        Serial.printf("DHT11: Read failed (attempt %d) - Temp: %.1f, Humidity: %.1f\n", 
                     attempt + 1, temperature, humidity);
      }
      
      if (attempt < SENSOR_RETRY_COUNT - 1) {
        delay(SENSOR_RETRY_DELAY);
      }
    }
    
    lastReadTime = currentTime;
    
    if (readSuccess) {
      // Update state
      lastTemperature = temperature;
      lastHumidity = humidity;
      consecutiveErrors = 0;
      
      // Populate JSON data
      populateJsonData(data, temperature, humidity);
      
      Serial.printf("DHT11: Read successful - Temp: %.1f°C, Humidity: %.1f%%\n", 
                   temperature, humidity);
      
      return true;
    } else {
      consecutiveErrors++;
      Serial.printf("DHT11: All read attempts failed (consecutive errors: %d)\n", 
                   consecutiveErrors);
      
      // Add error info to JSON
      data["error"] = "read_failed";
      data["consecutive_errors"] = consecutiveErrors;
      
      return false;
    }
  }
  
  /**
   * Check if sensor is enabled
   * @return true if enabled, false otherwise
   */
  bool isEnabled() const {
    return enabled;
  }
  
  /**
   * Check if sensor is initialized
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const {
    return initialized;
  }
  
  /**
   * Get sensor health status
   * @return true if sensor is healthy, false if experiencing issues
   */
  bool isHealthy() const {
    return consecutiveErrors < MAX_CONSECUTIVE_ERRORS;
  }
  
  /**
   * Get last valid temperature reading
   * @return temperature in Celsius, NAN if no valid reading
   */
  float getLastTemperature() const {
    return lastTemperature;
  }
  
  /**
   * Get last valid humidity reading
   * @return humidity in percentage, NAN if no valid reading
   */
  float getLastHumidity() const {
    return lastHumidity;
  }
  
private:
  /**
   * Validate sensor readings are within reasonable ranges
   * @param temp Temperature reading
   * @param hum Humidity reading
   * @return true if readings are valid, false otherwise
   */
  bool isValidReading(float temp, float hum) const {
    // DHT11 specifications:
    // Temperature: 0 to 50°C (±2°C accuracy)
    // Humidity: 20% to 95% RH (±5% accuracy)
    // Allow some tolerance for readings slightly outside spec
    return (temp >= -10.0 && temp <= 60.0) && 
           (hum >= 0.0 && hum <= 100.0);
  }
  
  /**
   * Populate JSON object with sensor data
   * @param data JsonObject to populate
   * @param temp Temperature value
   * @param hum Humidity value
   */
  void populateJsonData(JsonObject& data, float temp, float hum) const {
    data["temperature"] = round(temp * 10) / 10.0; // Round to 1 decimal place
    data["temperature_unit"] = "celsius";
    data["humidity"] = round(hum * 10) / 10.0;     // Round to 1 decimal place
    data["humidity_unit"] = "percent";
    data["sensor_type"] = "DHT11";
    data["read_time"] = millis();
  }
};

#endif