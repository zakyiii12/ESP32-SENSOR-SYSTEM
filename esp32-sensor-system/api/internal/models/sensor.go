// internal\models\sensor.go

package models

import (
	"encoding/json"
	"time"
)

type SensorData struct {
	ID        int                    `json:"id" db:"id"`
	DeviceID  string                 `json:"device_id" db:"device_id"`
	Timestamp time.Time              `json:"timestamp" db:"timestamp"`
	Data      map[string]interface{} `json:"data" db:"data"`
	CreatedAt time.Time              `json:"created_at" db:"created_at"`
}

type SensorDataRequest struct {
	DeviceID  string                 `json:"device_id" binding:"required"`
	Timestamp time.Time              `json:"timestamp" binding:"required"`
	Data      map[string]interface{} `json:"data" binding:"required"`
}

type SensorDataResponse struct {
	Data       []SensorData `json:"data"`
	TotalCount int          `json:"total_count"`
	Page       int          `json:"page"`
	PageSize   int          `json:"page_size"`
}

type QueryParams struct {
	DeviceID  string    `form:"device_id"`
	StartTime time.Time `form:"start_time" time_format:"2006-01-02T15:04:05Z07:00"`
	EndTime   time.Time `form:"end_time" time_format:"2006-01-02T15:04:05Z07:00"`
	Fields    []string  `form:"fields"`
	Page      int       `form:"page"`
	PageSize  int       `form:"page_size"`
}

func (s *SensorData) MarshalJSON() ([]byte, error) {
	type Alias SensorData
	return json.Marshal(&struct {
		Timestamp string `json:"timestamp"`
		CreatedAt string `json:"created_at"`
		*Alias
	}{
		Timestamp: s.Timestamp.Format(time.RFC3339),
		CreatedAt: s.CreatedAt.Format(time.RFC3339),
		Alias:     (*Alias)(s),
	})
}
