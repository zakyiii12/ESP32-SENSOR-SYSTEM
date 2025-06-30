// repository/location.go
package repository

import (
	"database/sql"
	"fmt"
	"strconv"
	"time"

	"github.com/sirupsen/logrus"
)

type LocationRepository struct {
	db *sql.DB
}

func NewLocationRepository(db *sql.DB) *LocationRepository {
	return &LocationRepository{db: db}
}

// LocationData represents location data in the database
type LocationData struct {
	ID         int       `json:"id"`
	DeviceID   string    `json:"device_id"`
	DeviceName string    `json:"device_name,omitempty"`
	Timestamp  time.Time `json:"timestamp"`
	Latitude   float64   `json:"latitude"`
	Longitude  float64   `json:"longitude"`
	Altitude   *int      `json:"altitude,omitempty"`
	Accuracy   *int      `json:"accuracy,omitempty"`
	Battery    *int      `json:"battery,omitempty"`
	Speed      *int      `json:"speed,omitempty"`
	Course     *int      `json:"course,omitempty"`
	Trigger    *string   `json:"trigger,omitempty"`
	TrackerID  string    `json:"tracker_id"`
	UserAgent  *string   `json:"user_agent,omitempty"`
	RawPayload string    `json:"raw_payload,omitempty"`
	CreatedAt  time.Time `json:"created_at"`
}

// DeviceLocationInfo represents device summary with location data
type DeviceLocationInfo struct {
	DeviceID      string    `json:"device_id"`
	DeviceName    string    `json:"device_name"`
	TrackerID     string    `json:"tracker_id"`
	LastLatitude  float64   `json:"last_latitude"`
	LastLongitude float64   `json:"last_longitude"`
	LastTimestamp time.Time `json:"last_timestamp"`
	TotalRecords  int       `json:"total_records"`
	FirstSeen     time.Time `json:"first_seen"`
	LastSeen      time.Time `json:"last_seen"`
}

// CreateLocation stores a new location record
func (r *LocationRepository) CreateLocation(data *LocationData) error {
	query := `
		INSERT INTO locations (
			device_id, device_name, timestamp, latitude, longitude, 
			altitude, accuracy, battery, speed, course, trigger, 
			tracker_id, user_agent, raw_payload, created_at
		) VALUES (
			$1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15
		)`

	_, err := r.db.Exec(query,
		data.DeviceID, data.DeviceName, data.Timestamp, data.Latitude, data.Longitude,
		data.Altitude, data.Accuracy, data.Battery, data.Speed, data.Course, data.Trigger,
		data.TrackerID, data.UserAgent, data.RawPayload, time.Now(),
	)

	if err != nil {
		logrus.WithError(err).Error("Failed to insert location data")
		return fmt.Errorf("failed to insert location data: %w", err)
	}

	return nil
}

// GetLocations retrieves location history with optional filtering
func (r *LocationRepository) GetLocations(deviceID, limitStr string) ([]*LocationData, error) {
	var query string
	var args []interface{}
	argIndex := 1

	baseQuery := `
		SELECT id, device_id, device_name, timestamp, latitude, longitude, 
			   altitude, accuracy, battery, speed, course, trigger, 
			   tracker_id, user_agent, raw_payload, created_at 
		FROM locations`

	if deviceID != "" {
		query = baseQuery + " WHERE device_id = $" + strconv.Itoa(argIndex)
		args = append(args, deviceID)
		argIndex++
	} else {
		query = baseQuery
	}

	query += " ORDER BY timestamp DESC"

	// Add limit
	limit, err := strconv.Atoi(limitStr)
	if err != nil || limit <= 0 {
		limit = 100
	}
	if limit > 1000 {
		limit = 1000 // Max limit
	}

	query += " LIMIT $" + strconv.Itoa(argIndex)
	args = append(args, limit)

	rows, err := r.db.Query(query, args...)
	if err != nil {
		logrus.WithError(err).Error("Failed to query locations")
		return nil, fmt.Errorf("failed to query locations: %w", err)
	}
	defer rows.Close()

	var locations []*LocationData
	for rows.Next() {
		location := &LocationData{}
		err := rows.Scan(
			&location.ID, &location.DeviceID, &location.DeviceName, &location.Timestamp,
			&location.Latitude, &location.Longitude, &location.Altitude, &location.Accuracy,
			&location.Battery, &location.Speed, &location.Course, &location.Trigger,
			&location.TrackerID, &location.UserAgent, &location.RawPayload, &location.CreatedAt,
		)
		if err != nil {
			logrus.WithError(err).Error("Failed to scan location row")
			continue
		}
		locations = append(locations, location)
	}

	return locations, nil
}

// GetLatestLocation retrieves the most recent location for a device
func (r *LocationRepository) GetLatestLocation(deviceID string) (*LocationData, error) {
	query := `
		SELECT id, device_id, device_name, timestamp, latitude, longitude, 
			   altitude, accuracy, battery, speed, course, trigger, 
			   tracker_id, user_agent, raw_payload, created_at 
		FROM locations 
		WHERE device_id = $1 
		ORDER BY timestamp DESC 
		LIMIT 1`

	row := r.db.QueryRow(query, deviceID)

	location := &LocationData{}
	err := row.Scan(
		&location.ID, &location.DeviceID, &location.DeviceName, &location.Timestamp,
		&location.Latitude, &location.Longitude, &location.Altitude, &location.Accuracy,
		&location.Battery, &location.Speed, &location.Course, &location.Trigger,
		&location.TrackerID, &location.UserAgent, &location.RawPayload, &location.CreatedAt,
	)

	if err != nil {
		if err == sql.ErrNoRows {
			return nil, nil
		}
		logrus.WithError(err).Error("Failed to query latest location")
		return nil, fmt.Errorf("failed to query latest location: %w", err)
	}

	return location, nil
}

// GetDevicesWithLocation retrieves list of devices that have location data
func (r *LocationRepository) GetDevicesWithLocation() ([]*DeviceLocationInfo, error) {
	query := `
		SELECT 
			device_id,
			device_name,
			tracker_id,
			latitude as last_latitude,
			longitude as last_longitude,
			timestamp as last_timestamp,
			total_records,
			first_seen,
			last_seen
		FROM (
			SELECT 
				device_id,
				device_name,
				tracker_id,
				latitude,
				longitude,
				timestamp,
				COUNT(*) OVER (PARTITION BY device_id) as total_records,
				MIN(timestamp) OVER (PARTITION BY device_id) as first_seen,
				MAX(timestamp) OVER (PARTITION BY device_id) as last_seen,
				ROW_NUMBER() OVER (PARTITION BY device_id ORDER BY timestamp DESC) as rn
			FROM locations
		) ranked
		WHERE rn = 1
		ORDER BY last_seen DESC`

	rows, err := r.db.Query(query)
	if err != nil {
		logrus.WithError(err).Error("Failed to query devices with location")
		return nil, fmt.Errorf("failed to query devices: %w", err)
	}
	defer rows.Close()

	var devices []*DeviceLocationInfo
	for rows.Next() {
		device := &DeviceLocationInfo{}
		err := rows.Scan(
			&device.DeviceID, &device.DeviceName, &device.TrackerID,
			&device.LastLatitude, &device.LastLongitude, &device.LastTimestamp,
			&device.TotalRecords, &device.FirstSeen, &device.LastSeen,
		)
		if err != nil {
			logrus.WithError(err).Error("Failed to scan device row")
			continue
		}
		devices = append(devices, device)
	}

	return devices, nil
}

// GetLocationStats retrieves statistics for location data
func (r *LocationRepository) GetLocationStats(deviceID string) (*LocationStats, error) {
	var query string
	var args []interface{}

	if deviceID != "" {
		query = `
			SELECT 
				COUNT(*) as total_records,
				MIN(timestamp) as first_timestamp,
				MAX(timestamp) as last_timestamp,
				AVG(accuracy) as avg_accuracy
			FROM locations 
			WHERE device_id = $1`
		args = []interface{}{deviceID}
	} else {
		query = `
			SELECT 
				COUNT(*) as total_records,
				MIN(timestamp) as first_timestamp,
				MAX(timestamp) as last_timestamp,
				AVG(accuracy) as avg_accuracy
			FROM locations`
	}

	row := r.db.QueryRow(query, args...)

	stats := &LocationStats{}
	var avgAccuracy sql.NullFloat64
	err := row.Scan(&stats.TotalRecords, &stats.FirstTimestamp, &stats.LastTimestamp, &avgAccuracy)
	if err != nil {
		logrus.WithError(err).Error("Failed to query location stats")
		return nil, fmt.Errorf("failed to query location stats: %w", err)
	}

	if avgAccuracy.Valid {
		accuracy := int(avgAccuracy.Float64)
		stats.AvgAccuracy = &accuracy
	}

	return stats, nil
}

// DeleteOldLocations removes location records older than the specified duration
func (r *LocationRepository) DeleteOldLocations(olderThan time.Duration) (int64, error) {
	cutoffTime := time.Now().Add(-olderThan)

	query := "DELETE FROM locations WHERE timestamp < $1"
	result, err := r.db.Exec(query, cutoffTime)
	if err != nil {
		logrus.WithError(err).Error("Failed to delete old locations")
		return 0, fmt.Errorf("failed to delete old locations: %w", err)
	}

	rowsAffected, err := result.RowsAffected()
	if err != nil {
		return 0, err
	}

	logrus.WithFields(logrus.Fields{
		"rows_deleted": rowsAffected,
		"cutoff_time":  cutoffTime,
	}).Info("Deleted old location records")

	return rowsAffected, nil
}

// LocationStats represents statistics about location data
type LocationStats struct {
	TotalRecords   int       `json:"total_records"`
	FirstTimestamp time.Time `json:"first_timestamp"`
	LastTimestamp  time.Time `json:"last_timestamp"`
	AvgAccuracy    *int      `json:"avg_accuracy,omitempty"`
}
