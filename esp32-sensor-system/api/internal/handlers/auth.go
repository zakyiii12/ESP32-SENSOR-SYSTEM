// internal/handlers/auth.go
package handlers

import (
	"errors"
	"net/http"
	"strings"

	"esp32-sensor-system/internal/models"
	"esp32-sensor-system/internal/service"

	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
)

type AuthHandler struct {
	authService *service.AuthService
}

func NewAuthHandler(authService *service.AuthService) *AuthHandler {
	return &AuthHandler{
		authService: authService,
	}
}

// Register handles user registration
func (h *AuthHandler) Register(c *gin.Context) {
	var req models.RegisterRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		logrus.WithError(err).Warn("Invalid registration request")
		c.JSON(http.StatusBadRequest, gin.H{
			"error":   "Invalid request payload",
			"details": err.Error(),
		})
		return
	}

	// Input validation
	if len(strings.TrimSpace(req.Username)) < 3 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Username must be at least 3 characters long",
		})
		return
	}

	if len(req.Password) < 6 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Password must be at least 6 characters long",
		})
		return
	}

	// Email validation (basic)
	if !strings.Contains(req.Email, "@") || len(strings.TrimSpace(req.Email)) == 0 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Valid email address is required",
		})
		return
	}

	user, err := h.authService.Register(&req)
	if err != nil {
		switch err {
		case service.ErrUserExists:
			c.JSON(http.StatusConflict, gin.H{
				"error": "Username or email already exists",
			})
		default:
			logrus.WithError(err).Error("Failed to register user")
			c.JSON(http.StatusInternalServerError, gin.H{
				"error": "Failed to create user account",
			})
		}
		return
	}

	logrus.WithFields(logrus.Fields{
		"user_id":  user.ID,
		"username": user.Username,
		"email":    user.Email,
	}).Info("User registered successfully")

	c.JSON(http.StatusCreated, gin.H{
		"message": "User registered successfully",
		"user": models.UserInfo{
			ID:       user.ID,
			Username: user.Username,
			Email:    user.Email,
			IsActive: user.IsActive,
		},
	})
}

// Login handles user authentication
func (h *AuthHandler) Login(c *gin.Context) {
	var req models.LoginRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		logrus.WithError(err).Warn("Invalid login request")
		c.JSON(http.StatusBadRequest, gin.H{
			"error":   "Invalid request payload",
			"details": err.Error(),
		})
		return
	}

	// Input validation
	if len(strings.TrimSpace(req.Username)) == 0 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Username is required",
		})
		return
	}

	if len(req.Password) == 0 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Password is required",
		})
		return
	}

	response, err := h.authService.Login(&req)
	if err != nil {
		switch err {
		case service.ErrUserNotFound, service.ErrInvalidPassword:
			// Don't differentiate between invalid username and password for security
			c.JSON(http.StatusUnauthorized, gin.H{
				"error": "Invalid username or password",
			})
		case service.ErrInactiveUser:
			c.JSON(http.StatusForbidden, gin.H{
				"error": "Account is inactive",
			})
		default:
			logrus.WithError(err).Error("Failed to authenticate user")
			c.JSON(http.StatusInternalServerError, gin.H{
				"error": "Authentication failed",
			})
		}
		return
	}

	logrus.WithFields(logrus.Fields{
		"user_id":  response.User.ID,
		"username": response.User.Username,
	}).Info("User logged in successfully")

	c.JSON(http.StatusOK, response)
}

// Logout handles user logout
func (h *AuthHandler) Logout(c *gin.Context) {
	// Get token from Authorization header
	auth := c.GetHeader("Authorization")
	if !strings.HasPrefix(auth, "Bearer ") {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Bearer token required",
		})
		return
	}

	token := strings.TrimPrefix(auth, "Bearer ")
	if token == "" {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Token cannot be empty",
		})
		return
	}

	err := h.authService.Logout(token)
	if err != nil {
		logrus.WithError(err).Warn("Failed to logout user")
		// Don't return error - logout should be idempotent
	}

	logrus.Info("User logged out successfully")
	c.JSON(http.StatusOK, gin.H{
		"message": "Logged out successfully",
	})
}

// GetProfile returns the current user's profile
func (h *AuthHandler) GetProfile(c *gin.Context) {
	userID, exists := c.Get("user_id")
	if !exists {
		c.JSON(http.StatusUnauthorized, gin.H{
			"error": "User not authenticated",
		})
		return
	}

	userInfo, err := h.authService.GetUserInfo(userID.(int))
	if err != nil {
		switch err {
		case service.ErrUserNotFound:
			c.JSON(http.StatusNotFound, gin.H{
				"error": "User not found",
			})
		default:
			logrus.WithError(err).Error("Failed to get user profile")
			c.JSON(http.StatusInternalServerError, gin.H{
				"error": "Failed to retrieve profile",
			})
		}
		return
	}

	c.JSON(http.StatusOK, userInfo)
}

// ChangePassword handles password change requests
func (h *AuthHandler) ChangePassword(c *gin.Context) {
	userID, exists := c.Get("user_id")
	if !exists {
		c.JSON(http.StatusUnauthorized, gin.H{
			"error": "User not authenticated",
		})
		return
	}

	var req models.ChangePasswordRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		logrus.WithError(err).Warn("Invalid change password request")
		c.JSON(http.StatusBadRequest, gin.H{
			"error":   "Invalid request payload",
			"details": err.Error(),
		})
		return
	}

	// Input validation
	if len(req.CurrentPassword) == 0 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Current password is required",
		})
		return
	}

	if len(req.NewPassword) < 6 {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "New password must be at least 6 characters long",
		})
		return
	}

	if req.CurrentPassword == req.NewPassword {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "New password must be different from current password",
		})
		return
	}

	err := h.authService.ChangePassword(userID.(int), &req)
	if err != nil {
		switch err {
		case service.ErrUserNotFound:
			c.JSON(http.StatusNotFound, gin.H{
				"error": "User not found",
			})
		case service.ErrInvalidPassword:
			c.JSON(http.StatusBadRequest, gin.H{
				"error": "Current password is incorrect",
			})
		default:
			logrus.WithError(err).Error("Failed to change password")
			c.JSON(http.StatusInternalServerError, gin.H{
				"error": "Failed to change password",
			})
		}
		return
	}

	logrus.WithField("user_id", userID).Info("Password changed successfully")

	c.JSON(http.StatusOK, gin.H{
		"message": "Password changed successfully",
	})
}

// RefreshToken handles token refresh requests
func (h *AuthHandler) RefreshToken(c *gin.Context) {
	userID, exists := c.Get("user_id")
	if !exists {
		c.JSON(http.StatusUnauthorized, gin.H{
			"error": "User not authenticated",
		})
		return
	}

	// Get current token for logout (optional - to invalidate old token)
	auth := c.GetHeader("Authorization")
	var currentToken string
	if strings.HasPrefix(auth, "Bearer ") {
		currentToken = strings.TrimPrefix(auth, "Bearer ")
	}

	userInfo, err := h.authService.GetUserInfo(userID.(int))
	if err != nil {
		switch err {
		case service.ErrUserNotFound:
			c.JSON(http.StatusNotFound, gin.H{
				"error": "User not found",
			})
		default:
			logrus.WithError(err).Error("Failed to refresh token")
			c.JSON(http.StatusInternalServerError, gin.H{
				"error": "Failed to refresh token",
			})
		}
		return
	}

	// Convert UserInfo to User model for token generation
	// You'll need to create a User from UserInfo or modify generateTokenForUser
	user := &models.User{
		ID:       userInfo.ID,
		Username: userInfo.Username,
		Email:    userInfo.Email,
		IsActive: userInfo.IsActive,
	}

	response, err := h.generateTokenForUser(user)
	if err != nil {
		logrus.WithError(err).Error("Failed to generate refresh token")
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Token refresh not fully implemented - please add RefreshToken method to AuthService",
		})
		return
	}

	// Optionally logout the old token
	if currentToken != "" {
		h.authService.Logout(currentToken)
	}

	logrus.WithField("user_id", userID).Info("Token refreshed successfully")

	c.JSON(http.StatusOK, gin.H{
		"token":      response.Token,
		"expires_at": response.ExpiresAt,
		"user":       response.User,
	})
}

// Helper method to generate token for user (TEMP)
func (h *AuthHandler) generateTokenForUser(user *models.User) (*models.LoginResponse, error) {
	return nil, errors.New("refresh token generation not fully implemented - please add RefreshToken method to AuthService")
}

// Health check endpoint for auth service
func (h *AuthHandler) Health(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status":  "ok",
		"service": "auth",
	})
}
