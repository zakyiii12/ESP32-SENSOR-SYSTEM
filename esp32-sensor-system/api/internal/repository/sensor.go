// api\internal\repository\sensor.go
package repository

import (
	"database/sql"
	"encoding/json"
	"fmt"
	"strings"

	"esp32-sensor-system/internal/models"

	"github.com/sirupsen/logrus"
)

type SensorRepository struct {
	db *sql.DB
}

func NewSensorRepository(db *sql.DB) *SensorRepository {
	return &SensorRepository{db: db}
}

func (r *SensorRepository) Insert(data *models.SensorDataRequest) error {
	dataJSON, err := json.Marshal(data.Data)
	if err != nil {
		return fmt.Errorf("failed to marshal data: %w", err)
	}

	query := `
        INSERT INTO sensor_data (device_id, timestamp, data)
        VALUES ($1, $2, $3)
    `

	_, err = r.db.Exec(query, data.DeviceID, data.Timestamp, dataJSON)
	if err != nil {
		logrus.WithError(err).Error("Failed to insert sensor data")
		return fmt.Errorf("failed to insert sensor data: %w", err)
	}

	logrus.WithFields(logrus.Fields{
		"device_id": data.DeviceID,
		"":          data.Timestamp,
	}).Info("Sensor data inserted successfully")

	return nil
}

func (r *SensorRepository) Query(params *models.QueryParams) (*models.SensorDataResponse, error) {
	// Build query conditions
	var conditions []string
	var args []interface{}
	argCount := 0

	if params.DeviceID != "" {
		argCount++
		conditions = append(conditions, fmt.Sprintf("device_id = $%d", argCount))
		args = append(args, params.DeviceID)
	}

	if !params.StartTime.IsZero() {
		argCount++
		conditions = append(conditions, fmt.Sprintf("timestamp >= $%d", argCount))
		args = append(args, params.StartTime)
	}

	if !params.EndTime.IsZero() {
		argCount++
		conditions = append(conditions, fmt.Sprintf("timestamp <= $%d", argCount))
		args = append(args, params.EndTime)
	}

	// Set default pagination
	if params.Page <= 0 {
		params.Page = 1
	}
	if params.PageSize <= 0 {
		params.PageSize = 50
	}
	if params.PageSize > 1000 {
		params.PageSize = 1000
	}

	// Build WHERE clause
	whereClause := ""
	if len(conditions) > 0 {
		whereClause = "WHERE " + strings.Join(conditions, " AND ")
	}

	// Count total records
	countQuery := fmt.Sprintf("SELECT COUNT(*) FROM sensor_data %s", whereClause)
	var totalCount int
	err := r.db.QueryRow(countQuery, args...).Scan(&totalCount)
	if err != nil {
		return nil, fmt.Errorf("failed to count records: %w", err)
	}

	// Build main query with pagination
	offset := (params.Page - 1) * params.PageSize
	argCount++
	limitArg := argCount
	argCount++
	offsetArg := argCount

	query := fmt.Sprintf(`
        SELECT id, device_id, timestamp, data, created_at
        FROM sensor_data
        %s
        ORDER BY timestamp DESC
        LIMIT $%d OFFSET $%d
    `, whereClause, limitArg, offsetArg)

	args = append(args, params.PageSize, offset)

	rows, err := r.db.Query(query, args...)
	if err != nil {
		return nil, fmt.Errorf("failed to query sensor data: %w", err)
	}
	defer rows.Close()

	var sensorData []models.SensorData
	for rows.Next() {
		var data models.SensorData
		var dataJSON []byte

		err := rows.Scan(&data.ID, &data.DeviceID, &data.Timestamp, &dataJSON, &data.CreatedAt)
		if err != nil {
			return nil, fmt.Errorf("failed to scan row: %w", err)
		}

		if err := json.Unmarshal(dataJSON, &data.Data); err != nil {
			return nil, fmt.Errorf("failed to unmarshal data: %w", err)
		}

		// Filter fields if specified
		if len(params.Fields) > 0 {
			filteredData := make(map[string]interface{})
			for _, field := range params.Fields {
				if value, exists := data.Data[field]; exists {
					filteredData[field] = value
				}
			}
			data.Data = filteredData
		}

		sensorData = append(sensorData, data)
	}

	if err := rows.Err(); err != nil {
		return nil, fmt.Errorf("error iterating rows: %w", err)
	}

	return &models.SensorDataResponse{
		Data:       sensorData,
		TotalCount: totalCount,
		Page:       params.Page,
		PageSize:   params.PageSize,
	}, nil
}

func (r *SensorRepository) GetDevices() ([]string, error) {
	query := `
        SELECT DISTINCT device_id 
        FROM sensor_data 
        ORDER BY device_id
    `

	rows, err := r.db.Query(query)
	if err != nil {
		return nil, fmt.Errorf("failed to query devices: %w", err)
	}
	defer rows.Close()

	var devices []string
	for rows.Next() {
		var deviceID string
		if err := rows.Scan(&deviceID); err != nil {
			return nil, fmt.Errorf("failed to scan device: %w", err)
		}
		devices = append(devices, deviceID)
	}

	return devices, nil
}

func (r *SensorRepository) GetLatestData(deviceID string) (*models.SensorData, error) {
	query := `
        SELECT id, device_id, timestamp, data, created_at
        FROM sensor_data
        WHERE device_id = $1
        ORDER BY timestamp DESC
        LIMIT 1
    `

	var data models.SensorData
	var dataJSON []byte

	err := r.db.QueryRow(query, deviceID).Scan(
		&data.ID, &data.DeviceID, &data.Timestamp, &dataJSON, &data.CreatedAt,
	)
	if err != nil {
		if err == sql.ErrNoRows {
			return nil, nil
		}
		return nil, fmt.Errorf("failed to get latest data: %w", err)
	}

	if err := json.Unmarshal(dataJSON, &data.Data); err != nil {
		return nil, fmt.Errorf("failed to unmarshal data: %w", err)
	}

	return &data, nil
}
