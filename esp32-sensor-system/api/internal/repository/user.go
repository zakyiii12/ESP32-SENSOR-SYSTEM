// internal/repository/user.go
package repository

import (
	"database/sql"
	"esp32-sensor-system/internal/models"
)

type UserRepository struct {
	db *sql.DB
}

func NewUserRepository(db *sql.DB) *UserRepository {
	return &UserRepository{db: db}
}

// CreateUser creates a new user
func (r *UserRepository) CreateUser(user *models.User) error {
	query := `
		INSERT INTO users (username, email, password_hash, is_active) 
		VALUES ($1, $2, $3, $4) 
		RETURNING id, created_at, updated_at`

	err := r.db.QueryRow(query, user.Username, user.Email, user.PasswordHash, user.IsActive).
		Scan(&user.ID, &user.CreatedAt, &user.UpdatedAt)

	return err
}

// GetUserByUsername retrieves a user by username
func (r *UserRepository) GetUserByUsername(username string) (*models.User, error) {
	var user models.User
	query := `
		SELECT id, username, email, password_hash, is_active, created_at, updated_at 
		FROM users 
		WHERE username = $1 AND is_active = true`

	err := r.db.QueryRow(query, username).Scan(
		&user.ID, &user.Username, &user.Email, &user.PasswordHash,
		&user.IsActive, &user.CreatedAt, &user.UpdatedAt,
	)

	if err == sql.ErrNoRows {
		return nil, nil
	}

	return &user, err
}

// GetUserByEmail retrieves a user by email
func (r *UserRepository) GetUserByEmail(email string) (*models.User, error) {
	var user models.User
	query := `
		SELECT id, username, email, password_hash, is_active, created_at, updated_at 
		FROM users 
		WHERE email = $1 AND is_active = true`

	err := r.db.QueryRow(query, email).Scan(
		&user.ID, &user.Username, &user.Email, &user.PasswordHash,
		&user.IsActive, &user.CreatedAt, &user.UpdatedAt,
	)

	if err == sql.ErrNoRows {
		return nil, nil
	}

	return &user, err
}

// GetUserByID retrieves a user by ID
func (r *UserRepository) GetUserByID(id int) (*models.User, error) {
	var user models.User
	query := `
		SELECT id, username, email, password_hash, is_active, created_at, updated_at 
		FROM users 
		WHERE id = $1 AND is_active = true`

	err := r.db.QueryRow(query, id).Scan(
		&user.ID, &user.Username, &user.Email, &user.PasswordHash,
		&user.IsActive, &user.CreatedAt, &user.UpdatedAt,
	)

	if err == sql.ErrNoRows {
		return nil, nil
	}

	return &user, err
}

// UpdateUserPassword updates a user's password
func (r *UserRepository) UpdateUserPassword(userID int, passwordHash string) error {
	query := `UPDATE users SET password_hash = $1, updated_at = NOW() WHERE id = $2`
	_, err := r.db.Exec(query, passwordHash, userID)
	return err
}

// CreateSession creates a new user session
func (r *UserRepository) CreateSession(session *models.UserSession) error {
	query := `
		INSERT INTO user_sessions (user_id, token_hash, expires_at) 
		VALUES ($1, $2, $3) 
		RETURNING id, created_at, last_used_at`

	err := r.db.QueryRow(query, session.UserID, session.TokenHash, session.ExpiresAt).
		Scan(&session.ID, &session.CreatedAt, &session.LastUsedAt)

	return err
}

// GetSessionByTokenHash retrieves a session by token hash
func (r *UserRepository) GetSessionByTokenHash(tokenHash string) (*models.UserSession, error) {
	var session models.UserSession
	query := `
		SELECT id, user_id, token_hash, expires_at, created_at, last_used_at 
		FROM user_sessions 
		WHERE token_hash = $1 AND expires_at > NOW()`

	err := r.db.QueryRow(query, tokenHash).Scan(
		&session.ID, &session.UserID, &session.TokenHash,
		&session.ExpiresAt, &session.CreatedAt, &session.LastUsedAt,
	)

	if err == sql.ErrNoRows {
		return nil, nil
	}

	return &session, err
}

// UpdateSessionLastUsed updates the last used timestamp of a session
func (r *UserRepository) UpdateSessionLastUsed(sessionID int) error {
	query := `UPDATE user_sessions SET last_used_at = NOW() WHERE id = $1`
	_, err := r.db.Exec(query, sessionID)
	return err
}

// DeleteSession deletes a session (logout)
func (r *UserRepository) DeleteSession(tokenHash string) error {
	query := `DELETE FROM user_sessions WHERE token_hash = $1`
	_, err := r.db.Exec(query, tokenHash)
	return err
}

// DeleteExpiredSessions cleans up expired sessions
func (r *UserRepository) DeleteExpiredSessions() error {
	query := `DELETE FROM user_sessions WHERE expires_at <= NOW()`
	_, err := r.db.Exec(query)
	return err
}

// DeleteUserSessions deletes all sessions for a user (logout all devices)
func (r *UserRepository) DeleteUserSessions(userID int) error {
	query := `DELETE FROM user_sessions WHERE user_id = $1`
	_, err := r.db.Exec(query, userID)
	return err
}

// CheckUsernameExists checks if username already exists
func (r *UserRepository) CheckUsernameExists(username string) (bool, error) {
	var exists bool
	query := `SELECT EXISTS(SELECT 1 FROM users WHERE username = $1)`
	err := r.db.QueryRow(query, username).Scan(&exists)
	return exists, err
}

// CheckEmailExists checks if email already exists
func (r *UserRepository) CheckEmailExists(email string) (bool, error) {
	var exists bool
	query := `SELECT EXISTS(SELECT 1 FROM users WHERE email = $1)`
	err := r.db.QueryRow(query, email).Scan(&exists)
	return exists, err
}
