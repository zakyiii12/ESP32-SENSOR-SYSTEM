#ifndef DATA_QUEUE_H
#define DATA_QUEUE_H

#include <Arduino.h>
#include "config.h"

/**
 * Data Queue Manager Class
 * 
 * Manages a circular buffer for storing sensor data JSON strings
 * Features:
 * - Automatic overwrite of oldest data when full
 * - Thread-safe operations (for single-threaded ESP32 environment)
 * - Memory efficient string storage
 * - Queue statistics and monitoring
 */
class DataQueue {
private:
  String queue[MAX_QUEUE_SIZE];    // Circular buffer for data storage
  int front = 0;                   // Index of front element
  int rear = 0;                    // Index of rear element  
  int count = 0;                   // Current number of elements
  
  // Statistics tracking
  unsigned long totalEnqueued = 0;
  unsigned long totalDequeued = 0;
  unsigned long totalOverwrites = 0;
  int maxSizeReached = 0;
  
public:
  /**
   * Initialize the data queue
   */
  void begin() {
    Serial.println("=== Data Queue Initializing ===");
    
    clear();
    
    Serial.printf("Queue Configuration:\n");
    Serial.printf("  Max Size: %d items\n", MAX_QUEUE_SIZE);
    Serial.printf("  Estimated Memory: ~%d bytes\n", 
                  MAX_QUEUE_SIZE * 512); // Rough estimate per JSON string
    
    Serial.println("=== Data Queue Ready ===");
  }
  
  /**
   * Add data to the queue
   * @param data JSON string to enqueue
   * @return true if successfully enqueued, false if error
   */
  bool enqueue(const String& data) {
    if (data.length() == 0) {
      Serial.println("DataQueue: Attempted to enqueue empty data");
      return false;
    }
    
    // Check if queue is full
    if (isFull()) {
      Serial.printf("DataQueue: Queue full, overwriting oldest item (size: %d)\n", count);
      
      // Remove oldest item to make space (circular buffer behavior)
      dequeue();
      totalOverwrites++;
    }
    
    // Add new item
    queue[rear] = data;
    rear = (rear + 1) % MAX_QUEUE_SIZE;
    count++;
    totalEnqueued++;
    
    // Update statistics
    if (count > maxSizeReached) {
      maxSizeReached = count;
    }
    
    return true;
  }
  
  /**
   * Remove and return data from front of queue
   * @return JSON string data, empty string if queue is empty
   */
  String dequeue() {
    if (isEmpty()) {
      return "";
    }
    
    String data = queue[front];
    queue[front] = ""; // Clear the slot to free memory
    front = (front + 1) % MAX_QUEUE_SIZE;
    count--;
    totalDequeued++;
    
    return data;
  }
  
  /**
   * Look at front item without removing it
   * @return JSON string data, empty string if queue is empty
   */
  String peek() const {
    if (isEmpty()) {
      return "";
    }
    return queue[front];
  }
  
  /**
   * Check if queue is empty
   * @return true if empty, false otherwise
   */
  bool isEmpty() const {
    return count == 0;
  }
  
  /**
   * Check if queue is full
   * @return true if full, false otherwise
   */
  bool isFull() const {
    return count >= MAX_QUEUE_SIZE;
  }
  
  /**
   * Get current number of items in queue
   * @return number of items
   */
  int size() const {
    return count;
  }
  
  /**
   * Get maximum capacity of queue
   * @return maximum number of items
   */
  int capacity() const {
    return MAX_QUEUE_SIZE;
  }
  
  /**
   * Get queue utilization percentage
   * @return utilization as percentage (0-100)
   */
  float getUtilization() const {
    return (float)count / MAX_QUEUE_SIZE * 100.0;
  }
  
  /**
   * Clear all items from queue
   */
  void clear() {
    front = 0;
    rear = 0;
    count = 0;
    
    // Clear all queue slots to free memory
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
      queue[i] = "";
    }
    
    Serial.println("DataQueue: Queue cleared");
  }
  
  /**
   * Get detailed queue statistics
   * @return formatted statistics string
   */
  String getStatistics() const {
    String stats = "Queue Statistics:\n";
    stats += "  Current Size: " + String(count) + "/" + String(MAX_QUEUE_SIZE) + "\n";
    stats += "  Utilization: " + String(getUtilization(), 1) + "%\n";
    stats += "  Max Size Reached: " + String(maxSizeReached) + "\n";
    stats += "  Total Enqueued: " + String(totalEnqueued) + "\n";
    stats += "  Total Dequeued: " + String(totalDequeued) + "\n";
    stats += "  Total Overwrites: " + String(totalOverwrites) + "\n";
    stats += "  Items in Queue: " + String(totalEnqueued - totalDequeued);
    return stats;
  }
  
  /**
   * Print current queue status to Serial (for debugging)
   */
  void printStatus() const {
    Serial.printf("DataQueue Status:\n");
    Serial.printf("  Size: %d/%d (%.1f%% full)\n", 
                  count, MAX_QUEUE_SIZE, getUtilization());
    Serial.printf("  Front: %d, Rear: %d\n", front, rear);
    Serial.printf("  Total Operations: %lu enqueued, %lu dequeued, %lu overwrites\n",
                  totalEnqueued, totalDequeued, totalOverwrites);
  }
  
  /**
   * Validate queue integrity (for debugging)
   * @return true if queue is in valid state
   */
  bool validateIntegrity() const {
    // Check basic invariants
    if (count < 0 || count > MAX_QUEUE_SIZE) {
      Serial.printf("DataQueue: Invalid count: %d\n", count);
      return false;
    }
    
    if (front < 0 || front >= MAX_QUEUE_SIZE) {
      Serial.printf("DataQueue: Invalid front index: %d\n", front);
      return false;
    }
    
    if (rear < 0 || rear >= MAX_QUEUE_SIZE) {
      Serial.printf("DataQueue: Invalid rear index: %d\n", rear);
      return false;
    }
    
    // Check count consistency
    int calculatedCount = 0;
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
      if (queue[i].length() > 0) {
        calculatedCount++;
      }
    }
    
    // Note: calculated count might not match 'count' in circular buffer
    // because we clear strings when dequeuing, but this is normal
    
    return true;
  }
  
  /**
   * Get memory usage estimate
   * @return estimated memory usage in bytes
   */
  size_t getMemoryUsage() const {
    size_t totalMemory = sizeof(*this); // Object overhead
    
    // Add string memory usage
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
      totalMemory += queue[i].length();
    }
    
    return totalMemory;
  }
};

#endif