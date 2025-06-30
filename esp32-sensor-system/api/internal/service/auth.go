// internal/service/auth.go
package service

import (
	"errors"
	"esp32-sensor-system/internal/models"
	"esp32-sensor-system/internal/repository"

	"golang.org/x/crypto/bcrypt"
)

var (
	ErrUserNotFound    = errors.New("user not found")
	ErrInvalidPassword = errors.New("invalid password")
	ErrUserExists      = errors.New("user already exists")
	ErrInactiveUser    = errors.New("user account is inactive")
)

type AuthService struct {
	userRepo   *repository.UserRepository
	jwtService *JWTService
}

func NewAuthService(userRepo *repository.UserRepository, jwtService *JWTService) *AuthService {
	return &AuthService{
		userRepo:   userRepo,
		jwtService: jwtService,
	}
}

// Register creates a new user account
func (s *AuthService) Register(req *models.RegisterRequest) (*models.User, error) {
	// Check if username already exists
	usernameExists, err := s.userRepo.CheckUsernameExists(req.Username)
	if err != nil {
		return nil, err
	}
	if usernameExists {
		return nil, ErrUserExists
	}

	// Check if email already exists
	emailExists, err := s.userRepo.CheckEmailExists(req.Email)
	if err != nil {
		return nil, err
	}
	if emailExists {
		return nil, ErrUserExists
	}

	// Hash password
	hashedPassword, err := bcrypt.GenerateFromPassword([]byte(req.Password), bcrypt.DefaultCost)
	if err != nil {
		return nil, err
	}

	// Create user
	user := &models.User{
		Username:     req.Username,
		Email:        req.Email,
		PasswordHash: string(hashedPassword),
		IsActive:     true,
	}

	err = s.userRepo.CreateUser(user)
	if err != nil {
		return nil, err
	}

	return user, nil
}

// RefreshToken generates a new token for an authenticated user
func (s *AuthService) RefreshToken(user *models.User) (*models.LoginResponse, error) {
	// Generate JWT token
	token, expiresAt, err := s.jwtService.GenerateToken(user)
	if err != nil {
		return nil, err
	}

	// Optionally store new session in database
	tokenHash := s.jwtService.HashToken(token)
	session := &models.UserSession{
		UserID:    user.ID,
		TokenHash: tokenHash,
		ExpiresAt: expiresAt,
	}
	s.userRepo.CreateSession(session)

	return &models.LoginResponse{
		Token:     token,
		ExpiresAt: expiresAt,
		User: models.UserInfo{
			ID:       user.ID,
			Username: user.Username,
			Email:    user.Email,
			IsActive: user.IsActive,
		},
	}, nil
}

// Login authenticates a user and returns a token
func (s *AuthService) Login(req *models.LoginRequest) (*models.LoginResponse, error) {
	// Get user by username
	user, err := s.userRepo.GetUserByUsername(req.Username)
	if err != nil {
		return nil, err
	}
	if user == nil {
		return nil, ErrUserNotFound
	}

	if !user.IsActive {
		return nil, ErrInactiveUser
	}

	// Verify password
	err = bcrypt.CompareHashAndPassword([]byte(user.PasswordHash), []byte(req.Password))
	if err != nil {
		return nil, ErrInvalidPassword
	}

	// Generate JWT token
	token, expiresAt, err := s.jwtService.GenerateToken(user)
	if err != nil {
		return nil, err
	}

	// Optionally store session in database
	tokenHash := s.jwtService.HashToken(token)
	session := &models.UserSession{
		UserID:    user.ID,
		TokenHash: tokenHash,
		ExpiresAt: expiresAt,
	}

	// Create session (optional, for session tracking)
	s.userRepo.CreateSession(session)

	return &models.LoginResponse{
		Token:     token,
		ExpiresAt: expiresAt,
		User: models.UserInfo{
			ID:       user.ID,
			Username: user.Username,
			Email:    user.Email,
			IsActive: user.IsActive,
		},
	}, nil
}

// ValidateToken validates a JWT token and returns user info
func (s *AuthService) ValidateToken(tokenString string) (*models.User, error) {
	claims, err := s.jwtService.ValidateToken(tokenString)
	if err != nil {
		return nil, err
	}

	// Get user from database to ensure user still exists and is active
	user, err := s.userRepo.GetUserByID(claims.UserID)
	if err != nil {
		return nil, err
	}
	if user == nil {
		return nil, ErrUserNotFound
	}

	if !user.IsActive {
		return nil, ErrInactiveUser
	}

	// Optionally update session last used time
	tokenHash := s.jwtService.HashToken(tokenString)
	session, err := s.userRepo.GetSessionByTokenHash(tokenHash)
	if err == nil && session != nil {
		s.userRepo.UpdateSessionLastUsed(session.ID)
	}

	return user, nil
}

// Logout invalidates a user's session
func (s *AuthService) Logout(tokenString string) error {
	tokenHash := s.jwtService.HashToken(tokenString)
	return s.userRepo.DeleteSession(tokenHash)
}

// ChangePassword changes a user's password
func (s *AuthService) ChangePassword(userID int, req *models.ChangePasswordRequest) error {
	user, err := s.userRepo.GetUserByID(userID)
	if err != nil {
		return err
	}
	if user == nil {
		return ErrUserNotFound
	}

	// Verify current password
	err = bcrypt.CompareHashAndPassword([]byte(user.PasswordHash), []byte(req.CurrentPassword))
	if err != nil {
		return ErrInvalidPassword
	}

	// Hash new password
	hashedPassword, err := bcrypt.GenerateFromPassword([]byte(req.NewPassword), bcrypt.DefaultCost)
	if err != nil {
		return err
	}

	// Update password
	return s.userRepo.UpdateUserPassword(userID, string(hashedPassword))
}

// GetUserInfo returns user information
func (s *AuthService) GetUserInfo(userID int) (*models.UserInfo, error) {
	user, err := s.userRepo.GetUserByID(userID)
	if err != nil {
		return nil, err
	}
	if user == nil {
		return nil, ErrUserNotFound
	}

	return &models.UserInfo{
		ID:       user.ID,
		Username: user.Username,
		Email:    user.Email,
		IsActive: user.IsActive,
	}, nil
}
