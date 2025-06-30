// router/router.go
package router

import (
	"database/sql"
	"fmt"
	"time"

	"esp32-sensor-system/internal/config"
	"esp32-sensor-system/internal/handlers"
	"esp32-sensor-system/internal/middleware"
	"esp32-sensor-system/internal/repository"
	"esp32-sensor-system/internal/service"

	"github.com/gin-gonic/gin"
)

func SetupRouter(db *sql.DB, cfg *config.Config) *gin.Engine {
	// Set Gin mode
	if cfg.Environment == "production" {
		gin.SetMode(gin.ReleaseMode)
	}

	r := gin.New()
	r.GET("/route-check", func(c *gin.Context) {
		c.JSON(200, gin.H{"message": "route-check is working"})
	})

	// Global middleware
	r.Use(gin.Recovery())
	r.Use(middleware.LoggingMiddleware())
	r.Use(middleware.CORSMiddleware())
	r.Use(middleware.RateLimitMiddleware(cfg.RateLimit))

	// Initialize repositories
	sensorRepo := repository.NewSensorRepository(db)
	userRepo := repository.NewUserRepository(db)
	locationRepo := repository.NewLocationRepository(db)

	// Initialize services (only if user auth is enabled)
	var authService *service.AuthService
	if cfg.EnableUserAuth {
		jwtService := service.NewJWTService(cfg.JWTSecret, cfg.JWTDuration)
		authService = service.NewAuthService(userRepo, jwtService)
	}

	// Initialize handlers
	dataHandler := handlers.NewDataHandler(sensorRepo)
	healthHandler := handlers.NewHealthHandler(db)
	locationHandler := handlers.NewLocationHandler(locationRepo)

	var authHandler *handlers.AuthHandler
	if authService != nil {
		authHandler = handlers.NewAuthHandler(authService)
		fmt.Println("→ AuthHandler initialized")
	}

	// Public routes (no auth required)
	r.GET("/health", healthHandler.HealthCheck)
	r.GET("/ready", healthHandler.ReadinessCheck)

	// Location endpoint with URL-based API key authentication
	// This is specifically designed for OwnTracks and similar services
	// that can't easily set custom headers but can modify URLs
	location := r.Group("/location")
	location.Use(middleware.URLAPIKeyOnlyMiddleware(cfg))
	{
		location.POST("/", locationHandler.PostLocation)          // OwnTracks HTTP endpoint
		location.POST("/owntracks", locationHandler.PostLocation) // Alternative endpoint name

		// ADD THESE NEW GET ENDPOINTS:
		location.GET("/", locationHandler.GetLocations) // Get all locations
		location.GET("/latest", func(c *gin.Context) {  // Get latest location (any device)
			// Get the latest location from any device
			devices, err := locationHandler.LocationRepo.GetDevicesWithLocation()
			if err != nil {
				c.JSON(500, gin.H{"error": "Failed to get devices"})
				return
			}

			if len(devices) == 0 {
				c.JSON(404, gin.H{"error": "No location data found"})
				return
			}

			// Get latest location from the first device (or you can modify this logic)
			latestLoc, err := locationHandler.LocationRepo.GetLatestLocation(devices[0].DeviceID)
			if err != nil {
				c.JSON(500, gin.H{"error": "Failed to get latest location"})
				return
			}

			// Format response to match your Python service expectations
			c.JSON(200, gin.H{
				"data":      latestLoc,
				"timestamp": time.Now().Unix(),
				"status":    "success",
			})
		})
		location.GET("/:device_id/latest", locationHandler.GetLatestLocation) // Get latest by device

		location.GET("/ping", func(c *gin.Context) {
			c.JSON(200, gin.H{"message": "location endpoint active", "timestamp": time.Now()})
		})
	}

	// Authentication routes (only if user auth is enabled)
	if authHandler != nil {
		auth := r.Group("/auth")
		{
			auth.POST("/register", authHandler.Register)
			auth.POST("/login", authHandler.Login)
			auth.POST("/logout", authHandler.Logout)
		}

		// User-only routes (JWT required)
		user := r.Group("/user")
		user.Use(middleware.JWTOnlyMiddleware(authService))
		{
			user.GET("/profile", authHandler.GetProfile)
			user.PUT("/password", authHandler.ChangePassword)
			user.POST("/refresh", authHandler.RefreshToken)
		}

		// Mixed auth API routes (accepts both API key and JWT, including URL-based API keys)
		api := r.Group("/api")
		api.Use(middleware.AuthMiddleware(cfg, authService))
		{
			// Sensor data endpoints - accessible by both devices (API key) and users (JWT)
			api.POST("/data", dataHandler.PostData)
			api.GET("/data", dataHandler.GetData)
			api.GET("/devices", dataHandler.GetDevices)
			api.GET("/devices/:device_id/latest", dataHandler.GetLatestData)

			// Location data endpoints - accessible by both devices and users
			api.GET("/locations", locationHandler.GetLocations)
			api.GET("/locations/devices", locationHandler.GetDeviceList)
			api.GET("/locations/:device_id/latest", locationHandler.GetLatestLocation)
		}

		// Device-only routes (API key only - for ESP32 devices and other IoT devices)
		device := r.Group("/device")
		device.Use(middleware.APIKeyOnlyMiddleware(cfg))
		{
			device.POST("/data", dataHandler.PostData)
			device.POST("/location", locationHandler.PostLocation) // Alternative location endpoint
			device.GET("/ping", func(c *gin.Context) {
				c.JSON(200, gin.H{"message": "pong", "timestamp": time.Now()})
			})
		}
	} else {
		// Fallback to original API key only authentication
		api := r.Group("/api")
		api.Use(middleware.APIKeyOnlyMiddleware(cfg))
		{
			// Sensor data endpoints
			api.POST("/data", dataHandler.PostData)
			api.GET("/data", dataHandler.GetData)
			api.GET("/devices", dataHandler.GetDevices)
			api.GET("/devices/:device_id/latest", dataHandler.GetLatestData)

			// Location data endpoints
			api.POST("/location", locationHandler.PostLocation)
			api.GET("/locations", locationHandler.GetLocations)
			api.GET("/locations/devices", locationHandler.GetDeviceList)
			api.GET("/locations/:device_id/latest", locationHandler.GetLatestLocation)
		}
	}

	// Print all registered routes for debugging
	fmt.Println("\n=== Registered Routes ===")
	for _, ri := range r.Routes() {
		fmt.Printf("%-6s %s\n", ri.Method, ri.Path)
	}

	return r
}
