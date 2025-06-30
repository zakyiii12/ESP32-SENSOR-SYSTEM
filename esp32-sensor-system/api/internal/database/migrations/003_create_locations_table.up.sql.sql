-- Migration: 003_create_locations_table.sql
-- Create locations table for storing OwnTracks and other location data

CREATE TABLE IF NOT EXISTS locations (
    id SERIAL PRIMARY KEY,
    device_id VARCHAR(255) NOT NULL,
    device_name VARCHAR(255),
    timestamp TIMESTAMPTZ NOT NULL,
    latitude DOUBLE PRECISION NOT NULL,
    longitude DOUBLE PRECISION NOT NULL,
    altitude INTEGER,
    accuracy INTEGER,
    battery INTEGER,
    speed INTEGER,
    course INTEGER,
    trigger VARCHAR(50),
    tracker_id VARCHAR(100) NOT NULL,
    user_agent TEXT,
    raw_payload JSONB,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Create indexes for better query performance
CREATE INDEX IF NOT EXISTS idx_locations_device_id ON locations(device_id);
CREATE INDEX IF NOT EXISTS idx_locations_timestamp ON locations(timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_locations_device_timestamp ON locations(device_id, timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_locations_tracker_id ON locations(tracker_id);
CREATE INDEX IF NOT EXISTS idx_locations_created_at ON locations(created_at);

-- Create composite index for location queries
CREATE INDEX IF NOT EXISTS idx_locations_lat_lon ON locations(latitude, longitude);

-- Create partial index for recent locations (last 30 days)
-- CREATE INDEX IF NOT EXISTS idx_locations_recent ON locations(device_id, timestamp DESC) 
-- WHERE timestamp > NOW() - INTERVAL '30 days';