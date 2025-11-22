/*
 * panic_stats.c - Panic Statistics Management Implementation
 *
 * Tracks system panics and resets using NVS for persistence.
 */

#include "panic_stats.h"
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

static const char *TAG = "PANIC_STATS";

// NVS namespace and keys
#define PANIC_STATS_NAMESPACE    "panic_stats"
#define PANIC_STATS_KEY_COUNT    "panic_count"
#define PANIC_STATS_KEY_CLEAN    "panics_clean"
#define PANIC_STATS_KEY_TIMESTAMP "last_panic_ts"

// Global panic statistics
static PanicStats_t g_panic_stats = {0};
static bool g_panic_stats_initialized = false;

/**
 * Get reset reason as human-readable string
 */
const char* panic_stats_get_reset_reason_string(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:    return "power_on";
        case ESP_RST_EXT:        return "external_reset";
        case ESP_RST_SW:         return "software_reset";
        case ESP_RST_PANIC:      return "panic";
        case ESP_RST_INT_WDT:    return "interrupt_wdt";
        case ESP_RST_TASK_WDT:   return "task_wdt";
        case ESP_RST_WDT:        return "watchdog";
        case ESP_RST_DEEPSLEEP:  return "deep_sleep";
        case ESP_RST_BROWNOUT:   return "brownout";
        case ESP_RST_SDIO:       return "sdio";
        default:                  return "unknown";
    }
}

/**
 * Check if reset reason indicates a panic
 */
static bool is_panic_reset(esp_reset_reason_t reason) {
    return (reason == ESP_RST_PANIC ||
            reason == ESP_RST_INT_WDT ||
            reason == ESP_RST_TASK_WDT ||
            reason == ESP_RST_WDT);
}

/**
 * Check if reset reason indicates a clean boot
 */
static bool is_clean_boot(esp_reset_reason_t reason) {
    return (reason == ESP_RST_POWERON ||
            reason == ESP_RST_SW ||
            reason == ESP_RST_EXT);
}

/**
 * Read panic statistics from NVS
 */
static esp_err_t panic_stats_read_from_nvs(PanicStats_t* stats) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(PANIC_STATS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read total panic count
    ret = nvs_get_u32(nvs_handle, PANIC_STATS_KEY_COUNT, &stats->total_panic_count);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        stats->total_panic_count = 0;
        ret = ESP_OK;
    }

    // Read panics since clean boot
    uint32_t panics_clean = 0;
    esp_err_t ret2 = nvs_get_u32(nvs_handle, PANIC_STATS_KEY_CLEAN, &panics_clean);
    if (ret2 == ESP_ERR_NVS_NOT_FOUND) {
        panics_clean = 0;
    }
    stats->panics_since_clean = panics_clean;

    // Read last panic timestamp
    ret2 = nvs_get_u32(nvs_handle, PANIC_STATS_KEY_TIMESTAMP, &stats->last_panic_timestamp);
    if (ret2 == ESP_ERR_NVS_NOT_FOUND) {
        stats->last_panic_timestamp = 0;
    }

    nvs_close(nvs_handle);
    return ret;
}

/**
 * Write panic statistics to NVS
 */
static esp_err_t panic_stats_write_to_nvs(const PanicStats_t* stats) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(PANIC_STATS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }

    // Write total panic count
    ret = nvs_set_u32(nvs_handle, PANIC_STATS_KEY_COUNT, stats->total_panic_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write panic count: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Write panics since clean boot
    ret = nvs_set_u32(nvs_handle, PANIC_STATS_KEY_CLEAN, stats->panics_since_clean);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write clean boot panic count: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Write last panic timestamp
    ret = nvs_set_u32(nvs_handle, PANIC_STATS_KEY_TIMESTAMP, stats->last_panic_timestamp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write panic timestamp: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }

    nvs_close(nvs_handle);
    return ret;
}

/**
 * Initialize panic statistics system
 */
esp_err_t panic_stats_init(void) {
    if (g_panic_stats_initialized) {
        ESP_LOGW(TAG, "Panic stats already initialized");
        return ESP_OK;
    }

    // Initialize NVS if needed (should already be done, but safe to call multiple times)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read existing panic statistics
    memset(&g_panic_stats, 0, sizeof(PanicStats_t));
    ret = panic_stats_read_from_nvs(&g_panic_stats);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read panic stats from NVS, initializing to zero");
        memset(&g_panic_stats, 0, sizeof(PanicStats_t));
    }

    // Get current reset reason
    esp_reset_reason_t reset_reason = esp_reset_reason();
    g_panic_stats.last_reset_reason = reset_reason;

    ESP_LOGI(TAG, "Reset reason: %s", panic_stats_get_reset_reason_string(reset_reason));

    // Check if this was a panic-related reset
    if (is_panic_reset(reset_reason)) {
        ESP_LOGW(TAG, "Panic-related reset detected, incrementing panic count");
        g_panic_stats.total_panic_count++;
        g_panic_stats.panics_since_clean++;
        
        // Get current timestamp (milliseconds since boot)
        // Note: On panic boot, we use 0 as timestamp since we don't know when panic occurred
        // The timestamp will be set on the next panic if it occurs during runtime
        g_panic_stats.last_panic_timestamp = 0;
        
        // Save to NVS
        ret = panic_stats_write_to_nvs(&g_panic_stats);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save panic count: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGW(TAG, "Panic count incremented: total=%lu, since_clean=%lu",
                     g_panic_stats.total_panic_count, g_panic_stats.panics_since_clean);
        }
    } else if (is_clean_boot(reset_reason)) {
        // Reset panics_since_clean on clean boot
        g_panic_stats.panics_since_clean = 0;
        ret = panic_stats_write_to_nvs(&g_panic_stats);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to reset clean boot counter: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Clean boot detected, resetting panics_since_clean counter");
        }
    }

    g_panic_stats_initialized = true;
    ESP_LOGI(TAG, "Panic statistics initialized: total=%lu, since_clean=%lu",
             g_panic_stats.total_panic_count, g_panic_stats.panics_since_clean);

    return ESP_OK;
}

/**
 * Get current panic statistics
 */
esp_err_t panic_stats_get(PanicStats_t* stats) {
    if (!g_panic_stats_initialized) {
        ESP_LOGE(TAG, "Panic stats not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(stats, &g_panic_stats, sizeof(PanicStats_t));
    return ESP_OK;
}

/**
 * Increment panic count (called automatically on panic-related boot)
 */
esp_err_t panic_stats_increment(void) {
    if (!g_panic_stats_initialized) {
        ESP_LOGW(TAG, "Panic stats not initialized, initializing now");
        esp_err_t ret = panic_stats_init();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    g_panic_stats.total_panic_count++;
    g_panic_stats.panics_since_clean++;
    g_panic_stats.last_panic_timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

    return panic_stats_write_to_nvs(&g_panic_stats);
}

/**
 * Reset panic counters for clean boot tracking
 */
esp_err_t panic_stats_reset_clean_boot(void) {
    if (!g_panic_stats_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    g_panic_stats.panics_since_clean = 0;
    return panic_stats_write_to_nvs(&g_panic_stats);
}

