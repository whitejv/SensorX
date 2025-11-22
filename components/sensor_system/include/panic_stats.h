/*
 * panic_stats.h - Panic Statistics Management
 *
 * This module tracks system panics and resets, storing counts in NVS
 * for persistence across power cycles.
 */

#ifndef PANIC_STATS_H
#define PANIC_STATS_H

#include <esp_err.h>
#include <stdint.h>
#include <esp_system.h>

// Panic statistics structure
typedef struct {
    uint32_t total_panic_count;      // Total panics ever (persistent across all resets)
    uint32_t panics_since_clean;     // Panics since last clean boot (power-on or software reset)
    esp_reset_reason_t last_reset_reason;  // Last reset reason
    uint32_t last_panic_timestamp;   // Timestamp of last panic (ms since boot, 0 if never)
} PanicStats_t;

/**
 * Initialize panic statistics system
 * Reads current panic count from NVS and checks reset reason
 * Should be called early in app_main()
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t panic_stats_init(void);

/**
 * Get current panic statistics
 * 
 * @param stats Pointer to structure to fill with statistics
 * @return ESP_OK on success, error code on failure
 */
esp_err_t panic_stats_get(PanicStats_t* stats);

/**
 * Increment panic count (called automatically on panic-related boot)
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t panic_stats_increment(void);

/**
 * Reset panic counters (optional, for clean boot tracking)
 * Resets panics_since_clean but keeps total_panic_count
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t panic_stats_reset_clean_boot(void);

/**
 * Get reset reason as human-readable string
 * 
 * @param reason Reset reason enum value
 * @return Pointer to string describing reset reason
 */
const char* panic_stats_get_reset_reason_string(esp_reset_reason_t reason);

#endif /* PANIC_STATS_H */

