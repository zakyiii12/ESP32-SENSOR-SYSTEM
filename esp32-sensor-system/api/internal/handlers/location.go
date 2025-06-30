// handlers/location.go
package handlers

import (
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
	"time"

	"esp32-sensor-system/internal/repository"

	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
)

type LocationHandler struct {
	LocationRepo *repository.LocationRepository // Make it exported (uppercase)
}

func NewLocationHandler(LocationRepo *repository.LocationRepository) *LocationHandler {
	return &LocationHandler{
		LocationRepo: LocationRepo, // Update this too
	}
}

// OwnTracksPayload represents the JSON payload from OwnTracks
type OwnTracksPayload struct {
	Type             string   `json:"_type"`          // "location"
	Timestamp        int64    `json:"tst"`            // Unix timestamp
	Latitude         float64  `json:"lat"`            // Latitude
	Longitude        float64  `json:"lon"`            // Longitude
	Altitude         *int     `json:"alt,omitempty"`  // Altitude in meters (optional)
	Accuracy         *int     `json:"acc,omitempty"`  // Accuracy in meters (optional)
	Battery          *int     `json:"batt,omitempty"` // Battery level (optional)
	Speed            *int     `json:"vel,omitempty"`  // Velocity in km/h (optional)
	Course           *int     `json:"cog,omitempty"`  // Course over ground (optional)
	Trigger          *string  `json:"t,omitempty"`    // Trigger (optional)
	TrackerID        string   `json:"tid"`            // Tracker ID
	UserAgent        *string  `json:"_ua,omitempty"`  // User agent (optional)
	VerticalAccuracy *int     `json:"vac,omitempty"`  // Vertical accuracy (optional)
	Pressure         *float64 `json:"p,omitempty"`    // Pressure (optional)
	ConnectionType   *string  `json:"conn,omitempty"` // Connection type (optional)

	// Additional fields for regions/waypoints
	Description *string `json:"desc,omitempty"`  // Description
	Event       *string `json:"event,omitempty"` // Event type (enter/leave)
	Radius      *int    `json:"rad,omitempty"`   // Radius for regions

	// Custom fields
	DeviceID   *string `json:"device_id,omitempty"`   // Custom device identifier
	DeviceName *string `json:"device_name,omitempty"` // Custom device name
}

// PostLocation handles location data from OwnTracks
func (h *LocationHandler) PostLocation(c *gin.Context) {
	var payload OwnTracksPayload

	// Parse JSON payload
	if err := c.ShouldBindJSON(&payload); err != nil {
		logrus.WithError(err).Error("Failed to parse location payload")
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Invalid JSON payload",
		})
		return
	}

	// Validate required fields
	if payload.Type != "location" {
		logrus.WithField("type", payload.Type).Warn("Invalid payload type")
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Invalid payload type, expected 'location'",
		})
		return
	}

	if payload.Latitude == 0 && payload.Longitude == 0 {
		logrus.Warn("Invalid coordinates (0,0)")
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Invalid coordinates",
		})
		return
	}

	// Process and store location data
	locationData := h.processLocationPayload(&payload, c)

	// Store in database
	err := h.LocationRepo.CreateLocation(locationData)
	if err != nil {
		logrus.WithError(err).Error("Failed to store location data")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to store location data",
		})
		return
	}

	// Log successful location update
	logrus.WithFields(logrus.Fields{
		"device_id":  locationData.DeviceID,
		"tracker_id": locationData.TrackerID,
		"latitude":   locationData.Latitude,
		"longitude":  locationData.Longitude,
		"timestamp":  locationData.Timestamp,
		"auth_type":  c.GetString("auth_type"),
	}).Info("Location data stored successfully")

	// Return success response (OwnTracks expects HTTP 200)
	c.JSON(http.StatusOK, gin.H{
		"status":    "success",
		"message":   "Location data received",
		"timestamp": locationData.Timestamp,
		"device_id": locationData.DeviceID,
	})
}

// GetLocations retrieves location history
func (h *LocationHandler) GetLocations(c *gin.Context) {
	deviceID := c.Query("device_id")
	limit := c.DefaultQuery("limit", "100")

	locations, err := h.LocationRepo.GetLocations(deviceID, limit)
	if err != nil {
		logrus.WithError(err).Error("Failed to retrieve locations")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to retrieve locations",
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status":    "success",
		"count":     len(locations),
		"locations": locations,
	})
}

// GetLatestLocation retrieves the latest location for a device
func (h *LocationHandler) GetLatestLocation(c *gin.Context) {
	deviceID := c.Param("device_id")
	if deviceID == "" {
		deviceID = c.Query("device_id")
	}

	if deviceID == "" {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Device ID is required",
		})
		return
	}

	location, err := h.LocationRepo.GetLatestLocation(deviceID)
	if err != nil {
		logrus.WithError(err).Error("Failed to retrieve latest location")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to retrieve latest location",
		})
		return
	}

	if location == nil {
		c.JSON(http.StatusNotFound, gin.H{
			"error": "No location data found for device",
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status":   "success",
		"location": location,
	})
}

// GetDeviceList retrieves list of devices with location data
func (h *LocationHandler) GetDeviceList(c *gin.Context) {
	devices, err := h.LocationRepo.GetDevicesWithLocation()
	if err != nil {
		logrus.WithError(err).Error("Failed to retrieve device list")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to retrieve device list",
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status":  "success",
		"count":   len(devices),
		"devices": devices,
	})
}

// processLocationPayload converts OwnTracks payload to LocationData
func (h *LocationHandler) processLocationPayload(payload *OwnTracksPayload, c *gin.Context) *repository.LocationData {
	// Generate device ID from various sources
	deviceID := h.generateDeviceID(payload, c)

	// Generate device name
	deviceName := h.generateDeviceName(payload, c)

	// Convert timestamp
	timestamp := time.Unix(payload.Timestamp, 0)
	if payload.Timestamp == 0 {
		timestamp = time.Now()
	}

	// Convert payload to JSON for raw storage
	rawPayload, _ := json.Marshal(payload)

	return &repository.LocationData{
		DeviceID:   deviceID,
		DeviceName: deviceName,
		Timestamp:  timestamp,
		Latitude:   payload.Latitude,
		Longitude:  payload.Longitude,
		Altitude:   payload.Altitude,
		Accuracy:   payload.Accuracy,
		Battery:    payload.Battery,
		Speed:      payload.Speed,
		Course:     payload.Course,
		Trigger:    payload.Trigger,
		TrackerID:  payload.TrackerID,
		UserAgent:  payload.UserAgent,
		RawPayload: string(rawPayload),
	}
}

// generateDeviceID creates a unique device identifier
func (h *LocationHandler) generateDeviceID(payload *OwnTracksPayload, c *gin.Context) string {
	// Priority order for device ID:
	// 1. Custom device_id from payload
	// 2. Tracker ID (tid)
	// 3. User agent + IP combination
	// 4. IP address as fallback

	if payload.DeviceID != nil && *payload.DeviceID != "" {
		return *payload.DeviceID
	}

	if payload.TrackerID != "" {
		return fmt.Sprintf("owntracks_%s", payload.TrackerID)
	}

	if payload.UserAgent != nil && *payload.UserAgent != "" {
		// Extract app name from user agent
		userAgent := strings.ToLower(*payload.UserAgent)
		if strings.Contains(userAgent, "owntracks") {
			return fmt.Sprintf("owntracks_%s", c.ClientIP())
		}
	}

	// Fallback to IP-based ID
	return fmt.Sprintf("location_%s", strings.ReplaceAll(c.ClientIP(), ".", "_"))
}

// generateDeviceName creates a human-readable device name
func (h *LocationHandler) generateDeviceName(payload *OwnTracksPayload, c *gin.Context) string {
	if payload.DeviceName != nil && *payload.DeviceName != "" {
		return *payload.DeviceName
	}

	if payload.UserAgent != nil && *payload.UserAgent != "" {
		return *payload.UserAgent
	}

	return fmt.Sprintf("OwnTracks Device (%s)", payload.TrackerID)
}
