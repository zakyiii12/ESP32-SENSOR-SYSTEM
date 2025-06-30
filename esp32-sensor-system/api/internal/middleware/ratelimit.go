package middleware

import (
	"net/http"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"golang.org/x/time/rate"
)

type RateLimiter struct {
	clients map[string]*rate.Limiter
	mu      sync.RWMutex
	rate    rate.Limit
	burst   int
}

func NewRateLimiter(rps int, burst int) *RateLimiter {
	return &RateLimiter{
		clients: make(map[string]*rate.Limiter),
		rate:    rate.Limit(rps),
		burst:   burst,
	}
}

func (rl *RateLimiter) getLimiter(key string) *rate.Limiter {
	rl.mu.RLock()
	limiter, exists := rl.clients[key]
	rl.mu.RUnlock()

	if !exists {
		rl.mu.Lock()
		// Double-check locking
		if limiter, exists = rl.clients[key]; !exists {
			limiter = rate.NewLimiter(rl.rate, rl.burst)
			rl.clients[key] = limiter
		}
		rl.mu.Unlock()
	}

	return limiter
}

func (rl *RateLimiter) cleanup() {
	// Clean up old limiters periodically
	go func() {
		ticker := time.NewTicker(time.Hour)
		defer ticker.Stop()

		for range ticker.C {
			rl.mu.Lock()
			// Remove limiters that haven't been used recently
			for key, limiter := range rl.clients {
				if limiter.Tokens() == float64(rl.burst) {
					delete(rl.clients, key)
				}
			}
			rl.mu.Unlock()
		}
	}()
}

func RateLimitMiddleware(rps int) gin.HandlerFunc {
	limiter := NewRateLimiter(rps, rps*2) // Allow burst of 2x the rate
	limiter.cleanup()

	return func(c *gin.Context) {
		// Use IP address as the key for rate limiting
		key := c.ClientIP()

		if !limiter.getLimiter(key).Allow() {
			c.JSON(http.StatusTooManyRequests, gin.H{
				"error":       "Rate limit exceeded",
				"retry_after": "60s",
			})
			c.Abort()
			return
		}

		c.Next()
	}
}
