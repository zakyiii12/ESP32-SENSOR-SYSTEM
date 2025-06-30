package handlers

import (
	"net/http"
	"strings"

	"esp32-sensor-system/internal/models"
	"esp32-sensor-system/internal/repository"

	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
)

type DataHandler struct {
	repo *repository.SensorRepository
}

func NewDataHandler(repo *repository.SensorRepository) *DataHandler {
	return &DataHandler{repo: repo}
}

func (h *DataHandler) PostData(c *gin.Context) {
	var req models.SensorDataRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		logrus.WithError(err).Warn("Invalid request payload")
		c.JSON(http.StatusBadRequest, gin.H{
			"error":   "Invalid request payload",
			"details": err.Error(),
		})
		return
	}

	// Validate required fields
	if req.DeviceID == "" {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "device_id is required",
		})
		return
	}

	if len(req.Data) == 0 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "data object is required and cannot be empty",
		})
		return
	}

	// Insert data
	if err := h.repo.Insert(&req); err != nil {
		logrus.WithError(err).Error("Failed to insert sensor data")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to store sensor data",
		})
		return
	}

	logrus.WithFields(logrus.Fields{
		"device_id": req.DeviceID,
		"timestamp": req.Timestamp,
		"data_keys": getKeys(req.Data),
	}).Info("Sensor data received successfully")

	c.JSON(http.StatusCreated, gin.H{
		"message":   "Data received successfully",
		"device_id": req.DeviceID,
		"timestamp": req.Timestamp,
	})
}

func (h *DataHandler) GetData(c *gin.Context) {
	var params models.QueryParams

	// Parse query parameters
	if err := c.ShouldBindQuery(&params); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{
			"error":   "Invalid query parameters",
			"details": err.Error(),
		})
		return
	}

	// Parse fields parameter
	if fieldsStr := c.Query("fields"); fieldsStr != "" {
		params.Fields = strings.Split(fieldsStr, ",")
		// Trim whitespace from field names
		for i, field := range params.Fields {
			params.Fields[i] = strings.TrimSpace(field)
		}
	}

	// Query data
	response, err := h.repo.Query(&params)
	if err != nil {
		logrus.WithError(err).Error("Failed to query sensor data")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to retrieve sensor data",
		})
		return
	}

	c.JSON(http.StatusOK, response)
}

func (h *DataHandler) GetDevices(c *gin.Context) {
	devices, err := h.repo.GetDevices()
	if err != nil {
		logrus.WithError(err).Error("Failed to get devices")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to retrieve devices",
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"devices": devices,
		"count":   len(devices),
	})
}

func (h *DataHandler) GetLatestData(c *gin.Context) {
	deviceID := c.Param("device_id")
	if deviceID == "" {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "device_id parameter is required",
		})
		return
	}

	data, err := h.repo.GetLatestData(deviceID)
	if err != nil {
		logrus.WithError(err).Error("Failed to get latest data")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to retrieve latest data",
		})
		return
	}

	if data == nil {
		c.JSON(http.StatusNotFound, gin.H{
			"error": "No data found for device",
		})
		return
	}

	c.JSON(http.StatusOK, data)
}

func getKeys(m map[string]interface{}) []string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	return keys
}
