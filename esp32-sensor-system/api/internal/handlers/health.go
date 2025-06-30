package handlers

import (
	"database/sql"
	"net/http"

	"github.com/gin-gonic/gin"
)

type HealthHandler struct {
	db *sql.DB
}

func NewHealthHandler(db *sql.DB) *HealthHandler {
	return &HealthHandler{db: db}
}

func (h *HealthHandler) HealthCheck(c *gin.Context) {
	// Check database connection
	if err := h.db.Ping(); err != nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{
			"status":   "unhealthy",
			"database": "disconnected",
			"error":    err.Error(),
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status":   "healthy",
		"database": "connected",
		"version":  "1.0.0",
	})
}

func (h *HealthHandler) ReadinessCheck(c *gin.Context) {
	// More thorough checks for readiness
	if err := h.db.Ping(); err != nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{
			"ready": false,
			"error": "Database not available",
		})
		return
	}

	// Test database query
	var count int
	err := h.db.QueryRow("SELECT COUNT(*) FROM sensor_data LIMIT 1").Scan(&count)
	if err != nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{
			"ready": false,
			"error": "Database query failed",
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"ready": true,
	})
}
