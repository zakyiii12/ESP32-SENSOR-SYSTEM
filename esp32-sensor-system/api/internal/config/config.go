// internal/config/config.go
package config

import (
	"os"
	"strconv"
	"strings"
	"time"
)

type Config struct {
	Port           string
	DatabaseURL    string
	APIKeys        []string
	LogLevel       string
	RateLimit      int
	Environment    string
	JWTSecret      string
	JWTDuration    time.Duration
	EnableUserAuth bool
}

func Load() *Config {
	// Parse JWT duration
	jwtDurationStr := getEnv("JWT_DURATION", "24h")
	jwtDuration, err := time.ParseDuration(jwtDurationStr)
	if err != nil {
		jwtDuration = 24 * time.Hour // Default to 24 hours
	}

	return &Config{
		Port:           getEnv("PORT", "8080"),
		DatabaseURL:    getEnv("DATABASE_URL", "postgres://esp32user:sherlockpipe@localhost:5432/sensordb?sslmode=disable"),
		APIKeys:        strings.Split(getEnv("API_KEYS", "default-api-key"), ","),
		LogLevel:       getEnv("LOG_LEVEL", "info"),
		RateLimit:      getEnvAsInt("RATE_LIMIT", 100),
		Environment:    getEnv("ENVIRONMENT", "development"),
		JWTSecret:      getEnv("JWT_SECRET", "your-secret-key-change-this-in-production"),
		JWTDuration:    jwtDuration,
		EnableUserAuth: getEnvAsBool("ENABLE_USER_AUTH", true),
	}
}

func getEnv(key, defaultValue string) string {
	if value, exists := os.LookupEnv(key); exists {
		return value
	}
	return defaultValue
}

func getEnvAsInt(key string, defaultValue int) int {
	if valueStr, exists := os.LookupEnv(key); exists {
		if value, err := strconv.Atoi(valueStr); err == nil {
			return value
		}
	}
	return defaultValue
}

func getEnvAsBool(key string, defaultValue bool) bool {
	if valueStr, exists := os.LookupEnv(key); exists {
		if value, err := strconv.ParseBool(valueStr); err == nil {
			return value
		}
	}
	return defaultValue
}
