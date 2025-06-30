#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include "esp_wifi.h"
#include "config.h"

/**
 * WiFi Manager Class
 * 
 * Handles all WiFi connectivity operations including:
 * - Initial WiFi connection with progressive retry strategy
 * - Connection monitoring and maintenance
 * - Automatic reconnection with backoff
 * - Network status reporting and diagnostics
 * - Error handling and recovery
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