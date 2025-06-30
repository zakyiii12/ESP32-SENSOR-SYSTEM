// middleware/auth.go
package middleware

import (
	"net/http"
	"strings"

	"esp32-sensor-system/internal/config"
	"esp32-sensor-system/internal/models"
	"esp32-sensor-system/internal/service"

	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
)

// AuthType represents the type of authentication used
type AuthType string

const (
	AuthTypeAPIKey    AuthType = "api_key"
	AuthTypeJWT       AuthType = "jwt"
	AuthTypeURLAPIKey AuthType = "url_api_key"
)

// AuthMiddleware handles both API key and JWT authentication
func AuthMiddleware(cfg *config.Config, authService *service.AuthService) gin.HandlerFunc {
	return func(c *gin.Context) {
		// Try JWT authentication first
		if authService != nil {
			if user, authType := tryJWTAuth(c, authService); user != nil {
				// JWT authentication successful
				c.Set("user_id", user.ID)
				c.Set("username", user.Username)
				c.Set("auth_type", authType)
				c.Next()
				return
			}
		}

		// Try URL-based API key authentication
		if tryURLAPIKeyAuth(c, cfg) {
			c.Set("auth_type", AuthTypeURLAPIKey)
			c.Next()
			return
		}

		// Fall back to header-based API key authentication
		if tryAPIKeyAuth(c, cfg) {
			c.Set("auth_type", AuthTypeAPIKey)
			c.Next()
			return
		}

		// No valid authentication found
		logrus.WithField("ip", c.ClientIP()).Warn("Authentication failed")
		c.JSON(http.StatusUnauthorized, gin.H{
			"error": "Authentication required",
		})
		c.Abort()
	}
}

// APIKeyOnlyMiddleware only allows API key authentication (for device endpoints)
func APIKeyOnlyMiddleware(cfg *config.Config) gin.HandlerFunc {
	return func(c *gin.Context) {
		// Try URL-based API key first
		if tryURLAPIKeyAuth(c, cfg) {
			c.Set("auth_type", AuthTypeURLAPIKey)
			c.Next()
			return
		}

		// Fall back to header-based API key
		if tryAPIKeyAuth(c, cfg) {
			c.Set("auth_type", AuthTypeAPIKey)
			c.Next()
			return
		}

		logrus.WithField("ip", c.ClientIP()).Warn("API key authentication failed")
		c.JSON(http.StatusUnauthorized, gin.H{
			"error": "Valid API key required",
		})
		c.Abort()
	}
}

// URLAPIKeyOnlyMiddleware only allows URL-based API key authentication (for OwnTracks)
func URLAPIKeyOnlyMiddleware(cfg *config.Config) gin.HandlerFunc {
	return func(c *gin.Context) {
		if !tryURLAPIKeyAuth(c, cfg) {
			logrus.WithFields(logrus.Fields{
				"ip":   c.ClientIP(),
				"path": c.Request.URL.Path,
			}).Warn("URL API key authentication failed")

			c.JSON(http.StatusUnauthorized, gin.H{
				"error": "Valid API key required in URL",
			})
			c.Abort()
			return
		}

		c.Set("auth_type", AuthTypeURLAPIKey)
		c.Next()
	}
}

// JWTOnlyMiddleware only allows JWT authentication (for user endpoints)
func JWTOnlyMiddleware(authService *service.AuthService) gin.HandlerFunc {
	return func(c *gin.Context) {
		if authService == nil {
			c.JSON(http.StatusInternalServerError, gin.H{
				"error": "Authentication service not available",
			})
			c.Abort()
			return
		}

		user, authType := tryJWTAuth(c, authService)
		if user == nil {
			logrus.WithField("ip", c.ClientIP()).Warn("JWT authentication failed")
			c.JSON(http.StatusUnauthorized, gin.H{
				"error": "Valid JWT token required",
			})
			c.Abort()
			return
		}

		c.Set("user_id", user.ID)
		c.Set("username", user.Username)
		c.Set("auth_type", authType)
		c.Next()
	}
}

// tryJWTAuth attempts JWT authentication
func tryJWTAuth(c *gin.Context, authService *service.AuthService) (*models.User, AuthType) {
	// Get token from Authorization header
	auth := c.GetHeader("Authorization")
	if !strings.HasPrefix(auth, "Bearer ") {
		return nil, ""
	}

	token := strings.TrimPrefix(auth, "Bearer ")
	if token == "" {
		return nil, ""
	}

	// Validate JWT token
	user, err := authService.ValidateToken(token)
	if err != nil {
		logrus.WithError(err).Debug("JWT validation failed")
		return nil, ""
	}

	logrus.WithFields(logrus.Fields{
		"user_id":  user.ID,
		"username": user.Username,
	}).Debug("JWT authentication successful")

	return user, AuthTypeJWT
}

// tryAPIKeyAuth attempts API key authentication from headers
func tryAPIKeyAuth(c *gin.Context, cfg *config.Config) bool {
	// Get API key from header
	apiKey := c.GetHeader("X-API-Key")
	if apiKey == "" {
		// Also check Authorization header with Bearer token (for backward compatibility)
		auth := c.GetHeader("Authorization")
		if strings.HasPrefix(auth, "Bearer ") {
			potentialAPIKey := strings.TrimPrefix(auth, "Bearer ")
			// Check if it's a valid API key (not a JWT)
			if !strings.Contains(potentialAPIKey, ".") { // JWT tokens contain dots
				apiKey = potentialAPIKey
			}
		}
	}

	if apiKey == "" {
		return false
	}

	// Validate API key
	for _, validKey := range cfg.APIKeys {
		if apiKey == validKey {
			logrus.WithField("api_key", maskAPIKey(apiKey)).Debug("Header API key authentication successful")
			c.Set("api_key", maskAPIKey(apiKey))
			return true
		}
	}

	logrus.WithFields(logrus.Fields{
		"ip":      c.ClientIP(),
		"api_key": maskAPIKey(apiKey),
	}).Warn("Invalid header API key")

	return false
}

// tryURLAPIKeyAuth attempts API key authentication from URL parameters
func tryURLAPIKeyAuth(c *gin.Context, cfg *config.Config) bool {
	// Try different parameter names that OwnTracks might use
	apiKey := c.Query("key")
	if apiKey == "" {
		apiKey = c.Query("api_key")
	}
	if apiKey == "" {
		apiKey = c.Query("apikey")
	}
	if apiKey == "" {
		apiKey = c.Query("token")
	}

	if apiKey == "" {
		return false
	}

	// Validate API key
	for _, validKey := range cfg.APIKeys {
		if apiKey == validKey {
			logrus.WithFields(logrus.Fields{
				"api_key": maskAPIKey(apiKey),
				"method":  "url_param",
			}).Debug("URL API key authentication successful")
			c.Set("api_key", maskAPIKey(apiKey))
			return true
		}
	}

	logrus.WithFields(logrus.Fields{
		"ip":      c.ClientIP(),
		"api_key": maskAPIKey(apiKey),
		"method":  "url_param",
	}).Warn("Invalid URL API key")

	return false
}

func maskAPIKey(key string) string {
	if len(key) <= 8 {
		return "***"
	}
	return key[:4] + "***" + key[len(key)-4:]
}
