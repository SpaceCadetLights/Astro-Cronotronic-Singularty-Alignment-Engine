/*
  clock_engine.cpp - Clock controller implementation
  For MKS-DLC32 controller with GRBL-ESP32
  
  This controls a clock where:
  - X axis = minute hand (1mm = 1 degree)
  - Y axis = hour hand (1mm = 1 degree)
  - 12 o'clock position is at 0,0
*/

#include "Grbl.h"
#include "GCode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include "MotionControl.h"
#include <Wire.h>
#include <RTClib.h>
#include <Preferences.h>

//===================================
// CONFIGURATION AND TIME VARIABLES
//===================================


// Debug level configuration
enum DebugLevel {
    DEBUG_MINIMAL = 0,   // Only essential messages
    DEBUG_NORMAL = 1,    // Important operational messages
    DEBUG_VERBOSE = 2    // All detailed messages including movement tracking
};

// Set the active debug level here
static DebugLevel active_debug_level = DEBUG_NORMAL;

// Forward declaration for debug_msg function so it can be used in the RTC functions
static void debug_msg(const char *format, ...);

// Forward declaration for send_gcode function so it can be used in the RTC functions
static bool send_gcode(const char *line);

// Forward declaration for disable_homing_required function
static void disable_homing_required();

// Forward declaration for continuous movement function
static void move_continuous_sequence(const float positions[][2], int num_positions, float speed_multipliers[]);

// Forward declaration for continuous rotation function
static void move_continuous_rotation(const float positions[][2], int num_positions, float speed_multipliers[]);

// Forward declaration for getting current position
static void get_current_position(float &x, float &y);

// Forward declaration for position verification function
static void verify_and_correct_time_position();

// Forward declaration for rehoming function
static void rehome_clock();

// Movement control flag - set to false to disable all physical movement
static bool movement_enabled = true;

// Movement speed in degrees per second (1-180 range)
// Higher values = faster movement
static float movement_speed = 180.0f;  // Increased from 30 to 180 for snappier movement

// Change the default acceleration to 0.5 (supports lower values now)
// Movement acceleration in degrees per second^2 (0.1-1000 range)
// EXTREMELY low values = ultra-fluid movement that never reaches constant speed
static float movement_accel = 0.5f; // Default to 0.5 for ultra-fluid motion

// Movement timeout multiplier - extends wait time for movements
// Higher values give more time for movements to complete
static float movement_timeout_factor = 1.5f; // 50% extra time

// Current time (24-hour format)
static int current_hour = 12;   // 0-23
static int current_minute = 0;  // 0-59

// Clock operation modes
enum ClockMode {
    MODE_CURRENT_TIME = 0,  // Show the current time
    MODE_SPECIFIC_TIME,     // Show a specific time
    MODE_PLAY_FILE,         // Play a G-code file
    MODE_DIRECT_ANGLE,      // Move to specific angles
    MODE_PENDULUM,          // Pendulum animation
    MODE_REWIND,            // Continuous backward rotation effect
    MODE_MOVE_TO_1111,      // Move to 11:11 and back
    MODE_MOVE_TO_420        // Move to 4:20 and back
};

// Current operation mode
static ClockMode current_mode = MODE_CURRENT_TIME;

// Specific time to display (when in MODE_SPECIFIC_TIME)
static int target_hour = 0;
static int target_minute = 0;

// Angle to display (when in MODE_DIRECT_ANGLE)
static float target_minute_angle = 0.0f;
static float target_hour_angle = 0.0f;

// File to play (when in MODE_PLAY_FILE)
static char file_to_play[32] = "spin420.nc";

// Sequence configuration
typedef struct {
    ClockMode mode;        // Operation mode for this step
    int hour;              // Hour to display (for MODE_SPECIFIC_TIME)
    int minute;            // Minute to display (for MODE_SPECIFIC_TIME)
    float min_angle;       // Minute hand angle (for MODE_DIRECT_ANGLE)
    float hour_angle;      // Hour hand angle (for MODE_DIRECT_ANGLE)
    char filename[32];     // File to play (for MODE_PLAY_FILE)
    uint32_t duration_ms;  // How long to stay in this mode
} SequenceStep;

// Define the sequence steps
#define MAX_SEQUENCE_STEPS 6  // Reduced to 3 steps

// Comment out the current sequence

static SequenceStep sequence[MAX_SEQUENCE_STEPS] = {
    // Show current time for 10 seconds
    {MODE_CURRENT_TIME, 0, 0, 0, 0, "", 10000},
    
    // Play pendulum animation
    {MODE_PENDULUM, 0, 0, 0, 0, "", 100},
    
    // Play rewind animation
    {MODE_REWIND, 0, 0, 0, 0, "", 100},
    
    // Play 11:11 animation
    {MODE_MOVE_TO_1111, 0, 0, 0, 0, "", 100},
    
    // Play 4:20 animation
    {MODE_MOVE_TO_420, 0, 0, 0, 0, "", 100},
    
    // Return to current time for 10 seconds
    {MODE_CURRENT_TIME, 0, 0, 0, 0, "", 10000}
};


// // New sequence for position verification
// static SequenceStep sequence[MAX_SEQUENCE_STEPS] = {
//     // Move to 12:00 and stay for 10 seconds
//     {MODE_SPECIFIC_TIME, 12, 0, 0, 0, "", 10000},
    
//     // Show current time for 10 seconds
//     {MODE_CURRENT_TIME, 0, 0, 0, 0, "", 10000},
    
//     // Move to 6:00 and stay for 10 seconds
//     {MODE_SPECIFIC_TIME, 6, 0, 0, 0, "", 10000}
// };

// Sequence tracking variables
static int current_sequence_step = 0;
static uint32_t sequence_step_start_time = 0;
static bool sequence_active = true;

// System state tracking
static bool system_ready = false;
static bool initial_homing_done = false;
static bool file_playback_active = false;

//===================================
// RTC CONFIGURATION
//===================================

// I2C pins for the MKS DLC32 board - these are the standard ESP32 I2C pins
#define SDA_PIN 0
#define SCL_PIN 4

// RTC object
RTC_DS1307 rtc;
static bool rtc_initialized = false;

// RTC auto-update variables
static Preferences preferences;
static bool first_boot_after_upload = false;
static uint32_t compile_time_hash = 0;

// Initialize the RTC
static bool init_rtc() {
    debug_msg("Initializing DS1307 RTC...");
    
    // Calculate a hash of compile time for comparison
    const char* compile_date = __DATE__;
    const char* compile_time = __TIME__;
    compile_time_hash = 0;
    for (int i = 0; compile_date[i]; i++) {
        compile_time_hash = compile_time_hash * 31 + compile_date[i];
    }
    for (int i = 0; compile_time[i]; i++) {
        compile_time_hash = compile_time_hash * 31 + compile_time[i];
    }
    
    // Initialize preferences
    preferences.begin("clock", false);
    
    // Check if this is first boot after firmware upload
    uint32_t saved_hash = preferences.getUInt("fw_hash", 0);
    if (saved_hash != compile_time_hash) {
        debug_msg("New firmware detected! Will update RTC with compile time");
        first_boot_after_upload = true;
        
        // Save the new firmware hash
        preferences.putUInt("fw_hash", compile_time_hash);
    }
    preferences.end();
    
    // Save current stepper enable state
    bool steppers_were_enabled = (sys.state != State::Alarm);
    
    // Configure I2C pins - use lower speed to reduce EMI/power issues
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); // Use 100kHz instead of default 400kHz
    
    bool rtc_ok = false;
    
    // Try to initialize the RTC with timeout
    uint32_t start_time = millis();
    while ((millis() - start_time) < 2000) { // 2 second timeout
        if (rtc.begin()) {
            rtc_ok = true;
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (!rtc_ok) {
        // Try one more time with different speed
        Wire.setClock(50000); // Even slower speed
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        if (rtc.begin()) {
            rtc_ok = true;
            debug_msg("RTC initialized on second attempt with slower speed");
        } else {
            debug_msg("ERROR: Couldn't find RTC after multiple attempts");
        }
    }
    
    if (!rtc_ok) {
        // Re-enable steppers if they were enabled before
        if (steppers_were_enabled) {
            debug_msg("Re-enabling steppers after RTC failure");
            send_gcode("$1=255");
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        return false;
    }
    
    // Check if the RTC needs updating
    if (first_boot_after_upload || !rtc.isrunning()) {
        debug_msg("Setting RTC to compile time: %s %s", __DATE__, __TIME__);
        // Set RTC to compile time
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        
        // Get the time we just set to confirm
        DateTime now = rtc.now();
        debug_msg("RTC updated to: %04d-%02d-%02d %02d:%02d:%02d",
                 now.year(), now.month(), now.day(),
                 now.hour(), now.minute(), now.second());
    } else {
        DateTime now = rtc.now();
        debug_msg("RTC found and running. Current time: %04d-%02d-%02d %02d:%02d:%02d",
                 now.year(), now.month(), now.day(),
                 now.hour(), now.minute(), now.second());
    }
    
    // Re-enable steppers after RTC initialization
    if (steppers_were_enabled) {
        debug_msg("Re-enabling steppers after RTC initialization");
        send_gcode("$1=255");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    return true;
}

// Get time from RTC and update current_hour and current_minute
static void update_time_from_rtc() {
    if (!rtc_initialized) return;
    
    // Add try/catch equivalent for RTC failures
    bool rtc_read_ok = false;
    DateTime now;
    
    try {
        now = rtc.now();
        rtc_read_ok = true;
    } catch (...) {
        debug_msg("WARNING: RTC read failed");
        return;
    }
    
    if (rtc_read_ok && (current_hour != now.hour() || current_minute != now.minute())) {
        current_hour = now.hour();
        current_minute = now.minute();
        debug_msg("Time updated from RTC: %02d:%02d", current_hour, current_minute);
    }
}

// Set the RTC time
static void set_rtc_time(int hour, int minute) {
    if (!rtc_initialized) return;
    
    // Get current date from RTC
    DateTime now = rtc.now();
    
    // Create a new DateTime object with the updated time
    DateTime newTime(now.year(), now.month(), now.day(), hour, minute, 0);
    
    // Set the RTC
    rtc.adjust(newTime);
    debug_msg("RTC time set to %02d:%02d", hour, minute);
}

//===================================
// UTILITY FUNCTIONS
//===================================

// Function to send G-code commands
static bool send_gcode(const char *line) {
    // Add timeout for command execution
    static uint32_t last_command_time = 0;
    uint32_t now = millis();
    
    // Prevent flooding commands too quickly
    if (now - last_command_time < 10) {
        vTaskDelay(10 / portTICK_PERIOD_MS); // Minimum spacing between commands
    }
    last_command_time = millis();
    
    // Copy the line to a temporary buffer for safety
    static char buf[96];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    
    // Execute the G-code command
    return (gc_execute_line(buf, CLIENT_SERIAL) == Error::Ok);
}

// Enhanced debug message function with level control
static void debug_msg(const char *format, ...) {
    // Format and send debug messages to the serial console
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Check if this is a movement tracking message that should be filtered
    bool is_movement_tracking = 
        (strstr(buffer, "Waiting for movement") != NULL) ||
        (strstr(buffer, "Movement completed early") != NULL) ||
        (strstr(buffer, "Movement should be complete") != NULL) ||
        (strstr(buffer, "est. time") != NULL) ||
        (strstr(buffer, "Rotation progress") != NULL);
    
    // Only print movement tracking messages in VERBOSE mode
    if (is_movement_tracking && active_debug_level < DEBUG_VERBOSE) {
        return;
    }
    
    // Additional filtering for MINIMAL mode
    if (active_debug_level == DEBUG_MINIMAL) {
        // In minimal mode, only show important state changes
        // Filter out routine messages
        if (strstr(buffer, "WARNING") == NULL &&
            strstr(buffer, "ERROR") == NULL &&
            strstr(buffer, "Starting") == NULL &&
            strstr(buffer, "complete") == NULL &&
            strstr(buffer, "Time updated") == NULL) {
            return;
        }
    }
    
    // Send the message
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, buffer);
}

// Convert time to clock hand angles
static void time_to_angles(int hour, int minute, float &hour_angle, float &minute_angle) {
    // Minute hand: 360° ÷ 60 minutes = 6° per minute
    minute_angle = minute * 6.0f;
    
    // Hour hand: 360° ÷ 12 hours = 30° per hour + 0.5° per minute
    hour_angle = (hour % 12) * 30.0f + minute * 0.5f;
}

// Move clock hands to specific angles - direct linear movement with shortest path
static void move_to_angles(float minute_angle, float hour_angle) {
    char cmd[64];
    
    // Validate input angles
    if (isnan(minute_angle) || isnan(hour_angle) ||
        isinf(minute_angle) || isinf(hour_angle)) {
        debug_msg("WARNING: Invalid angle inputs to move_to_angles");
        return;
    }
    
    // Bound angles to reasonable ranges (0-360)
    while (minute_angle < 0) minute_angle += 360.0f;
    while (minute_angle >= 360) minute_angle -= 360.0f;
    while (hour_angle < 0) hour_angle += 360.0f;
    while (hour_angle >= 360) hour_angle -= 360.0f;
    
    // Extra validation for extremely large values
    if (fabs(minute_angle) > 3600.0f || fabs(hour_angle) > 3600.0f) {
        debug_msg("WARNING: Extremely large angle detected, normalizing");
        // Normalize to prevent issues
        while (minute_angle < -360.0f) minute_angle += 360.0f;
        while (minute_angle > 360.0f) minute_angle -= 360.0f;
        while (hour_angle < -360.0f) hour_angle += 360.0f;
        while (hour_angle > 360.0f) hour_angle -= 360.0f;
    }
    
    // Get current position to calculate movement distance
    float current_min_angle = 0.0f;
    float current_hour_angle = 0.0f;
    get_current_position(current_min_angle, current_hour_angle);
    
    // Normalize current angles to 0-360 range
    while (current_min_angle < 0) current_min_angle += 360.0f;
    while (current_min_angle >= 360) current_min_angle -= 360.0f;
    while (current_hour_angle < 0) current_hour_angle += 360.0f;
    while (current_hour_angle >= 360) current_hour_angle -= 360.0f;
    
    // Calculate shortest path for minute hand
    float min_dist_cw = (minute_angle >= current_min_angle) ? 
                        (minute_angle - current_min_angle) : 
                        (minute_angle + 360.0f - current_min_angle);
                        
    float min_dist_ccw = (current_min_angle >= minute_angle) ? 
                         (current_min_angle - minute_angle) : 
                         (current_min_angle + 360.0f - minute_angle);
                         
    float min_target = current_min_angle;
    
    if (min_dist_cw <= min_dist_ccw) {
        // Clockwise is shorter or equal
        if (min_dist_cw > 180.0f) {
            // Need to handle wrap-around
            // Use an intermediate point to force clockwise movement
            float intermediate_min = current_min_angle - 10.0f;
            if (intermediate_min < 0) intermediate_min += 360.0f;
            min_target = intermediate_min;
            debug_msg("Minute hand: Using intermediate point %.1f to force clockwise", intermediate_min);
        } else {
            min_target = minute_angle;
        }
    } else {
        // Counterclockwise is shorter
        if (min_dist_ccw > 180.0f) {
            // Need to handle wrap-around
            // Use an intermediate point to force counterclockwise movement
            float intermediate_min = current_min_angle + 10.0f;
            if (intermediate_min >= 360.0f) intermediate_min -= 360.0f;
            min_target = intermediate_min;
            debug_msg("Minute hand: Using intermediate point %.1f to force counterclockwise", intermediate_min);
        } else {
            min_target = minute_angle;
        }
    }
    
    // Calculate shortest path for hour hand
    float hour_dist_cw = (hour_angle >= current_hour_angle) ? 
                         (hour_angle - current_hour_angle) : 
                         (hour_angle + 360.0f - current_hour_angle);
                         
    float hour_dist_ccw = (current_hour_angle >= hour_angle) ? 
                          (current_hour_angle - hour_angle) : 
                          (current_hour_angle + 360.0f - hour_angle);
                          
    float hour_target = current_hour_angle;
    
    if (hour_dist_cw <= hour_dist_ccw) {
        // Clockwise is shorter or equal
        if (hour_dist_cw > 180.0f) {
            // Need to handle wrap-around
            // Use an intermediate point to force clockwise movement
            float intermediate_hour = current_hour_angle - 10.0f;
            if (intermediate_hour < 0) intermediate_hour += 360.0f;
            hour_target = intermediate_hour;
            debug_msg("Hour hand: Using intermediate point %.1f to force clockwise", intermediate_hour);
        } else {
            hour_target = hour_angle;
        }
    } else {
        // Counterclockwise is shorter
        if (hour_dist_ccw > 180.0f) {
            // Need to handle wrap-around
            // Use an intermediate point to force counterclockwise movement
            float intermediate_hour = current_hour_angle + 10.0f;
            if (intermediate_hour >= 360.0f) intermediate_hour -= 360.0f;
            hour_target = intermediate_hour;
            debug_msg("Hour hand: Using intermediate point %.1f to force counterclockwise", intermediate_hour);
        } else {
            hour_target = hour_angle;
        }
    }
    
    // If we're using intermediate points, make the initial movement
    bool using_intermediate = (min_target != minute_angle) || (hour_target != hour_angle);
    
    if (using_intermediate) {
        // Make an initial move to the intermediate point
        // Calculate movement time based on distance
        float max_intermediate_dist = max(
            min(min_dist_cw, min_dist_ccw), 
            min(hour_dist_cw, hour_dist_ccw)
        );
        
        uint32_t intermediate_time_ms = (uint32_t)((max_intermediate_dist / movement_speed) * 1000);
        if (intermediate_time_ms < 500) intermediate_time_ms = 500; // At least 500ms
        
        // Actually move to the intermediate point
        memset(cmd, 0, sizeof(cmd));
        snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", min_target, hour_target, movement_speed * 60.0f);
        debug_msg("Moving to intermediate point X%.1f Y%.1f", min_target, hour_target);
        send_gcode(cmd);
        
        // Wait for the intermediate movement to complete
        vTaskDelay(intermediate_time_ms / portTICK_PERIOD_MS);
        
        // Now update the targets for the final movement
        min_target = minute_angle;
        hour_target = hour_angle;
    }
    
    // Use the larger distance to calculate movement time for the main/final move
    float max_distance = max(
        min(min_dist_cw, min_dist_ccw), 
        min(hour_dist_cw, hour_dist_ccw)
    );
    
    // Calculate movement time in milliseconds based on speed
    uint32_t movement_time_ms = (uint32_t)((max_distance / movement_speed) * 1000);
    
    // Add additional time for acceleration/deceleration (simplified calculation)
    movement_time_ms += (uint32_t)(2000.0f * sqrt(max_distance / movement_accel));
    
    // Apply timeout factor for safety
    movement_time_ms = (uint32_t)(movement_time_ms * movement_timeout_factor);
    
    // Ensure minimum movement time and cap maximum to avoid WiFi problems
    if (movement_time_ms < 500) movement_time_ms = 500;
    if (movement_time_ms > 8000) movement_time_ms = 8000; // Cap at 8 seconds
    
    // Track desired position even when movement is disabled
    target_minute_angle = minute_angle;
    target_hour_angle = hour_angle;
    
    // Skip physical movement if disabled
    if (!movement_enabled) {
        debug_msg("Movement disabled - virtual position X%.1f Y%.1f", minute_angle, hour_angle);
        return;
    }
    
    // Set very low acceleration for ultra-fluid motion
    memset(cmd, 0, sizeof(cmd));
    // Convert ultra-low values to the minimum GRBL will accept
    float effective_accel = (movement_accel < 1.0f) ? 
                            (1.0f + (movement_accel * 4.0f)) : // Map 0.1-1.0 to 1.4-5.0
                            movement_accel;
    snprintf(cmd, sizeof(cmd)-1, "$120=%.2f", effective_accel);
    send_gcode(cmd);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    snprintf(cmd, sizeof(cmd)-1, "$121=%.2f", effective_accel);
    send_gcode(cmd);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Set jerk to the absolute minimum possible
    send_gcode("$J0=0.01"); // X jerk absolute minimum
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_gcode("$J1=0.01"); // Y jerk absolute minimum
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Set very low junction deviation for smooth corners
    send_gcode("$11=0.001");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Move to absolute position
    send_gcode("G90"); // Absolute positioning
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Calculate feedrate in mm/min from degrees/sec (1mm = 1deg, 60sec = 1min)
    float feedrate = movement_speed * 60.0f;
    
    // Create command with angles and speed and send it
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", min_target, hour_target, feedrate);
    debug_msg("Moving to X%.1f Y%.1f (speed:%.1f deg/s, est. time:%dms)", 
              min_target, hour_target, movement_speed, movement_time_ms);
    send_gcode(cmd);
    
    // Wait for movement to complete in small chunks
    debug_msg("Waiting for movement to complete...");
    
    // Use multiple short delays instead of one long one
    const uint32_t CHECK_INTERVAL = 100; // Check every 100ms
    uint32_t elapsed = 0;
    
    while (elapsed < movement_time_ms) {
        // Short delay that won't block WiFi
        vTaskDelay(CHECK_INTERVAL / portTICK_PERIOD_MS);
        elapsed += CHECK_INTERVAL;
        
        // Check if we're done early
        if (sys.state == State::Idle) {
            debug_msg("Movement completed early");
            break;
        }
        
        // Yield more frequently to WiFi
        if (elapsed % 500 == 0) {
            // Give extra time for WiFi processing
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
    
    // Add more robust state detection
    if (sys.state != State::Idle && elapsed >= movement_time_ms) {
        debug_msg("WARNING: Movement timeout - forcing reset");
        send_gcode("M400"); // Try to flush motion buffer
        vTaskDelay(500 / portTICK_PERIOD_MS);
        send_gcode("$X"); // Unlock if needed
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    debug_msg("Movement should be complete");
}

// Move clock to display a specific time
static void display_time(int hour, int minute) {
    float hour_angle, minute_angle;
    
    // Convert the time to angles
    time_to_angles(hour, minute, hour_angle, minute_angle);
    
    // Move to the calculated angles
    move_to_angles(minute_angle, hour_angle);
}

// Enhanced file playback function with better error handling
static void play_gcode_file(const char* filename) {
    if (!file_playback_active) {
        char cmd[64] = {0};
        
        // Check if file exists (using SD.exists would be better, but requires SD library)
        bool file_exists = true; // Assume file exists
        
        if (file_exists) {
            snprintf(cmd, sizeof(cmd)-1, "$Play=/%s", filename);
            debug_msg("Playing file: %s", filename);
            if (!send_gcode(cmd)) {
                debug_msg("WARNING: Failed to start file playback");
                return;
            }
            file_playback_active = true;
        } else {
            debug_msg("ERROR: File '%s' not found", filename);
        }
    }
}

// Pendulum animation - makes clock hands fall with gravity and swing
// Modified pendulum animation with 3-second pause at bottom
static void play_pendulum_animation() {
    debug_msg("Playing pendulum animation with separate hand movements");
    
    // Store current time for returning later
    float original_hour_angle, original_minute_angle;
    time_to_angles(current_hour, current_minute, original_hour_angle, original_minute_angle);
    
    // Save current movement parameters
    float saved_speed = movement_speed;
    float saved_accel = movement_accel;
    
    // Use very slow acceleration for all movements
    movement_accel = 0.5f; // Ultra-low for fluid motion
    
    // Create separate position arrays for minute and hour hands
    // Split into two parts - fall and return with pause in between
    
    // Part 1: Initial movement to bottom position
    float pendulum_positions_fall[][2] = {
        // Starting positions
        {original_minute_angle, original_hour_angle},         
        
        // Initial droop - minute hand droops more than hour hand
        {original_minute_angle + 15, original_hour_angle + 8}, 
        
        // Fall phase - minute hand falls faster due to less weight
        {180, 165},                                           
        
        // First swing - minute hand swings further due to less mass
        {210, 195},                                           
        
        // First back swing - minute hand overshoots more
        {150, 160},                                           
        
        // Second swing - minute hand still swings wider
        {200, 188},                                           
        
        // Second back swing - starting to synchronize
        {160, 170},                                           
        
        // Third swing - getting closer in phase
        {190, 184},                                           
        
        // Third back swing - almost synchronized
        {170, 175},                                           
        
        // Tiny final oscillation
        {182, 180},                                           
        
        // Final settling at 6:00
        {180, 180}
    };
    
    // Speed multipliers for natural physics-based motion
    float pendulum_speeds_fall[] = {
        0.5f,   // Initial movement (slow)
        3.0f,   // Fast fall 
        2.4f,   // Fast first swing
        2.1f,   // First back swing
        1.8f,   // Second swing
        1.5f,   // Second back swing
        1.2f,   // Third swing
        0.9f,   // Third back swing
        0.6f,   // Tiny oscillation
        0.3f,   // Final settling
        0.3f    // Keep speed for final position
    };
    
    // Part 2: Return movement after pause
    float pendulum_positions_return[][2] = {
        // Starting from settled position
        {180, 180},
        
        // Wake-up jolt - minute hand reacts more
        {155, 165},                                           
        
        // Return to original time
        {original_minute_angle, original_hour_angle}          
    };
    
    // Speed multipliers for return
    float pendulum_speeds_return[] = {
        0.5f,   // Start slow
        1.5f,   // Wake-up jolt
        3.0f    // Fast return to original time
    };
    
    // Short dramatic pause before starting
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    // Execute the fall animation
    movement_speed = 220.0f; // Snappy movement
    move_continuous_sequence(pendulum_positions_fall, 11, pendulum_speeds_fall);
    
    // PAUSE at the bottom position for 3 seconds
    debug_msg("Pendulum paused at bottom position for 3 seconds");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Execute the return animation
    movement_speed = 220.0f;
    move_continuous_sequence(pendulum_positions_return, 3, pendulum_speeds_return);
    
    // Restore original speed settings
    movement_speed = saved_speed;
    movement_accel = saved_accel;
    
    debug_msg("Pendulum animation complete");
}

// Ultra-reliable forward spin animation with failsafes
static void play_rewind_animation() {
    debug_msg("Playing RELIABLE forward spin animation");
    
    // Create a local copy of current time for safety
    int safe_hour = current_hour;
    int safe_minute = current_minute;
    
    // Input validation to prevent crashes
    if (safe_hour < 0 || safe_hour > 23 || safe_minute < 0 || safe_minute > 59) {
        debug_msg("WARNING: Invalid time values, using 12:00 as fallback");
        safe_hour = 12;
        safe_minute = 0;
    }
    
    // Calculate starting angles with safety checks
    float original_hour_angle = 0.0f, original_minute_angle = 0.0f;
    time_to_angles(safe_hour, safe_minute, original_hour_angle, original_minute_angle);
    
    debug_msg("Starting position: X%.1f Y%.1f (from time %02d:%02d)", 
             original_minute_angle, original_hour_angle, safe_hour, safe_minute);
    
    // Save movement speed
    float saved_speed = movement_speed;
    
    // ================ STEP 1: SAFETY MOVE ================
    // First go to the starting position to ensure we're in a known state
    debug_msg("Safety move: returning to current time position");
    movement_speed = 150.0f;
    
    // Use move_to_angles which is well-tested
    move_to_angles(original_minute_angle, original_hour_angle);
    
    // Extra time to stabilize
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // ================ STEP 2: FIRST ROTATION ================
    // Perform first complete rotation
    debug_msg("Step 1: First rotation (simple move)");
    movement_speed = 300.0f;
    
    // Move exactly 360 degrees forward
    move_to_angles(original_minute_angle, original_hour_angle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Use direct G-code for reliability
    char cmd[64];
    
    // Clear the buffer for safety
    memset(cmd, 0, sizeof(cmd));
    
    // Just send a single, simple movement command for first rotation
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", 
             original_minute_angle + 360.0f, 
             original_hour_angle + 30.0f, 
             movement_speed * 60.0f);
    
    // Send and wait
    send_gcode(cmd);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Fixed wait time for predictability
    
    // ================ STEP 3: SECOND ROTATION ================
    // Perform second complete rotation
    debug_msg("Step 2: Second rotation (simple move)");
    movement_speed = 400.0f;
    
    // Clear the buffer for safety
    memset(cmd, 0, sizeof(cmd));
    
    // Just send a single, simple movement command for second rotation
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", 
             original_minute_angle + 720.0f, 
             original_hour_angle + 60.0f, 
             movement_speed * 60.0f);
    
    // Send and wait
    send_gcode(cmd);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Fixed wait time for predictability
    
    // ================ STEP 4: SAFE RETURN ================
    // Return to original position with extra safety
    debug_msg("Step 3: Returning to original position (safe approach)");
    
    // First, complete any ongoing moves
    send_gcode("M400");  // Wait for moves to complete
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Move to original position
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", 
             original_minute_angle, 
             original_hour_angle, 
             150.0f * 60.0f);
    
    send_gcode(cmd);
    
    // Wait with timeout for final move to complete
    uint32_t timeout = 5000; // 5 seconds max
    uint32_t start = millis();
    
    debug_msg("Waiting for final position move to complete...");
    while ((millis() - start < timeout)) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        if (sys.state == State::Idle) {
            debug_msg("Final move completed successfully");
            break;
        }
    }
    
    // If we timed out, make sure we unlock
    if (sys.state != State::Idle) {
        debug_msg("WARNING: Final move timed out, forcing completion");
        send_gcode("M400"); // Try to flush buffer
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    // Ensure we're at the correct normalized position
    send_gcode("G92 X0 Y0");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Now set to the real angle
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "G92 X%.1f Y%.1f", 
             original_minute_angle, original_hour_angle);
    send_gcode(cmd);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Restore original speed
    movement_speed = saved_speed;
    
    debug_msg("Forward spin animation completed safely");
}

// Animation to move to 11:11, stay for 6 seconds, then return to current time
static void play_1111_animation() {
    debug_msg("Playing 11:11 animation");
    
    // Store current time for returning later
    float original_hour_angle, original_minute_angle;
    time_to_angles(current_hour, current_minute, original_hour_angle, original_minute_angle);
    
    // Calculate angles for 11:11
    float target_hour_angle, target_minute_angle;
    time_to_angles(11, 11, target_hour_angle, target_minute_angle);
    
    // Save current movement parameters
    float saved_speed = movement_speed;
    
    // Move to 11:11 at moderate speed
    movement_speed = 180.0f;
    debug_msg("Moving to 11:11");
    move_to_angles(target_minute_angle, target_hour_angle);
    
    // Stay at 11:11 for 6 seconds
    debug_msg("Staying at 11:11 for 6 seconds");
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    
    // Return to original time at slightly faster speed
    movement_speed = 150.0f;
    debug_msg("Returning to current time");
    move_to_angles(original_minute_angle, original_hour_angle);
    
    // Restore original speed
    movement_speed = saved_speed;
    
    debug_msg("11:11 animation complete");
}

// Animation to move to 4:20, stay for 6 seconds, then return to current time
static void play_420_animation() {
    debug_msg("Playing 4:20 animation");
    
    // Store current time for returning later
    float original_hour_angle, original_minute_angle;
    time_to_angles(current_hour, current_minute, original_hour_angle, original_minute_angle);
    
    // Calculate angles for 4:20
    float target_hour_angle, target_minute_angle;
    time_to_angles(4, 20, target_hour_angle, target_minute_angle);
    
    // Save current movement parameters
    float saved_speed = movement_speed;
    
    // Move to 4:20 at moderate speed
    movement_speed = 180.0f;
    debug_msg("Moving to 4:20");
    move_to_angles(target_minute_angle, target_hour_angle);
    
    // Stay at 4:20 for 6 seconds
    debug_msg("Staying at 4:20 for 6 seconds");
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    
    // Return to original time at slightly faster speed
    movement_speed = 150.0f;
    debug_msg("Returning to current time");
    move_to_angles(original_minute_angle, original_hour_angle);
    
    // Restore original speed
    movement_speed = saved_speed;
    
    debug_msg("4:20 animation complete");
}


// Move to the next step in the sequence
static void advance_sequence() {
    // Simply increment to the next step
    current_sequence_step = (current_sequence_step + 1) % MAX_SEQUENCE_STEPS;
    sequence_step_start_time = millis();
    
    // Skip empty slots (default MODE_CURRENT_TIME with 0 duration)
    while (current_sequence_step > 0 && 
           sequence[current_sequence_step].mode == MODE_CURRENT_TIME && 
           sequence[current_sequence_step].duration_ms == 0) {
        
        current_sequence_step = (current_sequence_step + 1) % MAX_SEQUENCE_STEPS;
    }
    
    // Apply the new mode
    current_mode = sequence[current_sequence_step].mode;
    
    // Execute the appropriate action based on the mode
    switch (current_mode) {
        case MODE_CURRENT_TIME:
            debug_msg("Sequence: Showing current time");
            display_time(current_hour, current_minute);
            verify_and_correct_time_position(); // Add this line
            break;
            
        case MODE_SPECIFIC_TIME:
            target_hour = sequence[current_sequence_step].hour;
            target_minute = sequence[current_sequence_step].minute;
            debug_msg("Sequence: Showing specific time %02d:%02d", target_hour, target_minute);
            display_time(target_hour, target_minute);
            break;
            
        case MODE_PLAY_FILE:
            strncpy(file_to_play, sequence[current_sequence_step].filename, sizeof(file_to_play)-1);
            file_to_play[sizeof(file_to_play)-1] = '\0';
            debug_msg("Sequence: Playing file %s", file_to_play);
            file_playback_active = false; // Reset so we'll trigger playback
            break;
            
        case MODE_DIRECT_ANGLE:
            target_minute_angle = sequence[current_sequence_step].min_angle;
            target_hour_angle = sequence[current_sequence_step].hour_angle;
            debug_msg("Sequence: Moving to angles X%.1f Y%.1f", 
                      target_minute_angle, target_hour_angle);
            move_to_angles(target_minute_angle, target_hour_angle);
            break;
            
        case MODE_PENDULUM:
            debug_msg("Sequence: Starting pendulum animation");
            play_pendulum_animation();
            // Immediately advance to next step after animation completes
            advance_sequence();
            break;
            
        case MODE_REWIND:
            debug_msg("Sequence: Starting rewind animation");
            play_rewind_animation();
            // Immediately advance to next step after animation completes
            advance_sequence();
            break;
            
        case MODE_MOVE_TO_1111:
            debug_msg("Sequence: Moving to 11:11 and back");
            play_1111_animation();
            // Immediately advance to next step after animation completes
            advance_sequence();
            break;
            
        case MODE_MOVE_TO_420:
            debug_msg("Sequence: Moving to 4:20 and back");
            play_420_animation();
            // Immediately advance to next step after animation completes
            advance_sequence();
            break;
    }
}

// Sequence playback mode
enum SequenceMode {
    SEQUENCE_LINEAR = 0,  // Play sequences in order
    SEQUENCE_RANDOM = 1   // Play sequences in random order
};

// Current sequence mode
static SequenceMode sequence_mode = SEQUENCE_LINEAR;

// Array to track shuffled sequence order
static int shuffled_sequence[MAX_SEQUENCE_STEPS];

// Flag to indicate if sequence needs reshuffling
static bool needs_reshuffle = true;

// Forward declaration for sequence shuffling function
//static void shuffle_sequence();

//===================================
// MAIN TASK IMPLEMENTATION
//===================================

// This function will be called from the main .ino file
void clockEngineTask(void* parameter) {
    // Wait for system to boot fully
    debug_msg("Clock engine starting, waiting 20 seconds for system to initialize...");
    vTaskDelay(20000 / portTICK_PERIOD_MS);
    
    debug_msg("Clock engine task started!");
    
    // Force initialization sequence at startup
    system_ready = false;
    initial_homing_done = false;
    
    // Initialize sequence to inactive until homing completes
    sequence_active = false;
    
    // Add at the top of your clockEngineTask function:
    static uint32_t crash_recovery_count = 0;

    // Task loop
    while(true) {
        // Safety watchdog to recover from stalled states
        static uint32_t last_watchdog_kick = 0;
        uint32_t now = millis();
        
        if (now - last_watchdog_kick > 30000) { // Every 30 seconds
            // Reset any stalled states
            if (sys.state != State::Idle && sys.state != State::Alarm) {
                debug_msg("Watchdog: Resetting potentially stalled state");
                send_gcode("$X"); // Unlock
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            last_watchdog_kick = now;
        }
        
        // Report system status more frequently during startup
        static uint32_t last_status = 0;
        if (!initial_homing_done || (now - last_status >= 5000)) {
            const char* state_str = "UNKNOWN";
            switch(sys.state) {
                case State::Idle: state_str = "IDLE"; break;
                case State::Alarm: state_str = "ALARM"; break;
                case State::Cycle: state_str = "CYCLE"; break;
                case State::Hold: state_str = "HOLD"; break;
                default: state_str = "OTHER"; break;
            }
            
            debug_msg("Machine state: %s (%d)", state_str, (int)sys.state);
            last_status = now;
        }
        //---------------------------------------------------------
        // INITIALIZATION: Always try to clear alarms at startup
        //--------------------------------------------------------
        if (!system_ready) {
            debug_msg("Initializing system...");
            
            // Always try to unlock first, regardless of state
            debug_msg("Sending unlock command ($X)");
            send_gcode("$X");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Try again to be sure
            send_gcode("$X");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Enable steppers
            debug_msg("Enabling steppers ($1=255)");
            send_gcode("$1=255");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Set units to mm
            debug_msg("Setting units to mm (G21)");
            send_gcode("G21");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Check and enable homing
            debug_msg("Checking homing configuration...");
            send_gcode("$$"); // Print settings to help diagnose
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Enable homing
            debug_msg("Enabling homing ($22=1)");
            send_gcode("$22=1");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Set homing direction, usually negative direction for each axis
            debug_msg("Setting homing directions ($23=3)"); // 3 = home X and Y in negative direction
            send_gcode("$23=3");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            system_ready = true;
            debug_msg("System initialized, initializing RTC");
            
            // Initialize RTC - but be prepared to handle failure
            rtc_initialized = init_rtc();
            if (!rtc_initialized) {
                debug_msg("Continuing without RTC - will use software time");
            }
            
            // Make absolutely sure steppers are enabled
            debug_msg("Ensuring steppers are enabled");
            send_gcode("$1=255");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            
            // Call the homing required disable function
            disable_homing_required();

            debug_msg("Preparing for homing");
            
            // Move hands to show that the system is alive, even before homing
            debug_msg("Moving hands to show system is active (at reduced speed)");
            movement_enabled = true; // Force movement on
            move_to_angles(180, 180); // Move to 3:00/9:00 position (perpendicular)
            vTaskDelay(50 / portTICK_PERIOD_MS); // 2 seconds between movements
            move_to_angles(0, 0); // Move to 6:00 position (aligned)
            vTaskDelay(50 / portTICK_PERIOD_MS); // 2 seconds between movements
            debug_msg("Initial movement test complete, continuing with homing");
        }
        
        // HOMING: Use a direct approach without state machine
        if (system_ready && !initial_homing_done) {
            debug_msg("Starting sequential homing approach");
            
            // 1. Make absolutely sure we're unlocked
            debug_msg("Unlocking system");
            send_gcode("$X");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            
            // 2. First home the HOUR hand (Y axis) independently
            debug_msg("Homing hour hand (Y axis)");
            if (sys.state == State::Idle) {
                // Home Y axis only (bit mask: Y=2)
                mc_homing_cycle(0x2);  // Home Y axis (hour hand)
                vTaskDelay(5000 / portTICK_PERIOD_MS); // Extra time to complete
                debug_msg("Hour hand (Y axis) homing complete");
            } else {
                debug_msg("ERROR: Cannot home - system not in idle state");
            }
            
            // 3. Then home the MINUTE hand (X axis) independently
            debug_msg("Homing minute hand (X axis)");
            if (sys.state == State::Idle) {
                // Home X axis only (bit mask: X=1)
                mc_homing_cycle(0x1);  // Home X axis (minute hand)
                uint32_t homing_timeout = millis() + 15000; // 15 second timeout
                while (sys.state != State::Idle) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    // Check for timeout
                    if (millis() > homing_timeout) {
                        debug_msg("WARNING: Homing timeout - forcing unlock");
                        send_gcode("$X"); // Unlock
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        break;
                    }
                }
                vTaskDelay(5000 / portTICK_PERIOD_MS); // Extra time to complete
                debug_msg("Minute hand (X axis) homing complete");
            } else {
                debug_msg("ERROR: Cannot home - system not in idle state");
            }
            
            // 4. Move to 0,0 (12 o'clock position) after both axes are homed
            debug_msg("Homing complete, moving to 12 o'clock position (0,0)");
            move_to_angles(0, 0);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            
            // 5. Mark homing as complete
            debug_msg("Marking homing as complete");
            initial_homing_done = true;
            
            // 6. Now do the test movements AFTER homing is complete
            debug_msg("Beginning test movements");
            float saved_speed = movement_speed;
            movement_speed = 45.0f; // fast speed for testing
            
            // Test HOUR hand (Y axis) - with ultra-fluid motion
            debug_msg("Testing HOUR hand (Y axis)");
            movement_speed = 150.0f;
            movement_accel = 6.0f;  // Was 30.0f (5x reduction)
            move_to_angles(0, 90);    // Hour hand at 3:00
            vTaskDelay(500 / portTICK_PERIOD_MS);
            movement_speed = 120.0f;
            movement_accel = 5.0f;  // Was 25.0f (5x reduction)
            move_to_angles(0, 180);   // Hour hand at 6:00
            vTaskDelay(500 / portTICK_PERIOD_MS);
            movement_speed = 180.0f;
            movement_accel = 4.0f;  // Was 20.0f (5x reduction)
            move_to_angles(0, 270);   // Hour hand at 9:00
            vTaskDelay(500 / portTICK_PERIOD_MS);
            movement_speed = 210.0f;
            movement_accel = 7.0f;  // Was 35.0f (5x reduction)
            move_to_angles(0, 0);     // Hour hand at 12:00
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Test MINUTE hand (X axis) - moving to 4 cardinal positions
            debug_msg("Testing MINUTE hand (X axis)");
            move_to_angles(90, 0);    // Minute hand at 3:00
            vTaskDelay(500 / portTICK_PERIOD_MS);
            move_to_angles(180, 0);   // Minute hand at 6:00
            vTaskDelay(500 / portTICK_PERIOD_MS);
            move_to_angles(270, 0);   // Minute hand at 9:00
            vTaskDelay(500 / portTICK_PERIOD_MS);
            move_to_angles(0, 0);     // Minute hand at 12:00
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Finally, move both hands to starting position
            debug_msg("Moving both hands to 12:00 position");
            move_to_angles(0, 0);     // Both hands at 12:00
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Restore normal speed
            movement_speed = saved_speed;
            debug_msg("Test movements complete");
            
            // 7. Start sequence
            sequence_active = true;
            current_sequence_step = 0;
            sequence_step_start_time = millis();
            debug_msg("Starting sequence playback");
            advance_sequence();
        }
        
        // Only run sequence logic after homing is complete
        if (system_ready && initial_homing_done) {
            // Check if we need to advance to the next sequence step
            SequenceStep *current_step = &sequence[current_sequence_step];
            
    
            
            if (sequence_active && 
                current_step->duration_ms > 0 && 
                (now - sequence_step_start_time) > current_step->duration_ms) {
                debug_msg("Sequence step duration complete");
                advance_sequence();
            }
            
            // Process current mode
            switch (current_mode) {
                case MODE_CURRENT_TIME:
                    // Update the clock position to current time every minute
                    static uint8_t last_displayed_minute = 255;
                    if (last_displayed_minute != current_minute && 
                        current_mode == MODE_CURRENT_TIME && 
                        sys.state == State::Idle) {
                        display_time(current_hour, current_minute);
                        last_displayed_minute = current_minute;
                    }
                    break;
                    
                case MODE_SPECIFIC_TIME:
                    // Already handled in advance_sequence()
                    break;
                    
                case MODE_PLAY_FILE:
                    if (!file_playback_active) {
                        play_gcode_file(file_to_play);
                    }
                    break;
                    
                case MODE_DIRECT_ANGLE:
                    // Already handled in advance_sequence()
                    break;
            }
        }
        
        // TIME UPDATE: Get time from RTC instead of software counter
        static uint32_t last_rtc_check = 0;
        if (now - last_rtc_check >= 1000) // Check RTC every second
        {
            update_time_from_rtc();
            last_rtc_check = now;
        }
        
        // FILE PLAYBACK CHECK: Check if file playback is complete
        if (file_playback_active && sys.state == State::Idle) {
            debug_msg("File playback complete");
            file_playback_active = false;
            
            // If we're in a sequence, advance to the next step
            if (sequence_active && current_mode == MODE_PLAY_FILE) {
                advance_sequence();
            }
        }
        
        // Watchdog to detect and recover from crashes
        static uint32_t last_alive_time = 0;
        if (now - last_alive_time > 30000) { // Every 30 seconds
            // Ensure system is in a valid state
            if (sys.state != State::Idle && sys.state != State::Cycle) {
                crash_recovery_count++;
                debug_msg("Potential crash detected! Recovery attempt #%d", crash_recovery_count);
                
                // Force a reset of the internal state
                send_gcode("$X"); // Unlock
                vTaskDelay(100 / portTICK_PERIOD_MS);
                send_gcode("M400"); // Wait for moves to complete
                vTaskDelay(100 / portTICK_PERIOD_MS);
                send_gcode("G92 X0 Y0"); // Force position reset
                vTaskDelay(100 / portTICK_PERIOD_MS);
                
                // Force garbage collection and memory reset
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            last_alive_time = now;
        }
        
        // Add in clockEngineTask function, before the main loop ends:
        static uint32_t heap_check_time = 0;
        if (now - heap_check_time > 60000) { // Check every minute
            size_t free_heap = xPortGetFreeHeapSize();
            debug_msg("System health: Free heap %u bytes", free_heap);
            
            if (free_heap < 10000) {
                debug_msg("WARNING: Low memory condition");
            }
            
            heap_check_time = now;
        }
        
        // Add periodic position verification
        static uint32_t last_position_check = 0;
        if (now - last_position_check > 300000) { // Every 5 minutes
            if (current_mode == MODE_CURRENT_TIME && sys.state == State::Idle) {
                debug_msg("Performing periodic position verification");
                verify_and_correct_time_position();
            }
            last_position_check = now;
        }
        
        // Add periodic rehoming
        static uint32_t last_rehome_time = 0;
        if (now - last_rehome_time > 86400000) { // 24 hours
            if (current_mode == MODE_CURRENT_TIME && sys.state == State::Idle) {
                debug_msg("Performing periodic rehoming");
                rehome_clock();
            }
            last_rehome_time = now;
        }
        
        // Short delay before next loop iteration
        vTaskDelay(50 / portTICK_PERIOD_MS);
    } // End of while(true) loop
} // END OF clockEngineTask

// Disable homing required check for more reliable operation
static void disable_homing_required() {
    // Force setting $22=0 (Homing not required)
    send_gcode("$22=0");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    debug_msg("Homing required check disabled");
    
    // Also unlock and soft-home for good measure
    send_gcode("$X");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_gcode("G92 X0 Y0");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    debug_msg("Soft homing performed");
}

// New function: Send multiple movements as a continuous sequence
static void move_continuous_sequence(const float positions[][2], int num_positions, float speed_multipliers[]) {
    // Input validation
    if (num_positions <= 0 || num_positions > 20 || positions == NULL) {
        debug_msg("ERROR: Invalid parameters to move_continuous_sequence");
        return;
    }
    
    char cmd[64];
    
    // Skip movement if disabled
    if (!movement_enabled) {
        debug_msg("Movement disabled - skipping continuous sequence");
        return;
    }
    
    // Store original settings to restore later
    float saved_accel = movement_accel;
    
    // Set minimum viable acceleration
    float effect_accel = 10.0f;
    
    // CRITICAL: Wait for any ongoing movements to complete first
    send_gcode("M400");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Set acceleration parameters with error checking
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "$120=%.2f", effect_accel);
    if (!send_gcode(cmd)) {
        debug_msg("WARNING: Failed to set acceleration, continuing anyway");
        // Don't retry - just continue with current settings
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Longer delay
    
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "$121=%.2f", effect_accel);
    if (!send_gcode(cmd)) {
        debug_msg("WARNING: Failed to set acceleration, continuing anyway");
        // Don't retry - just continue with current settings
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Longer delay
    
    // Set jerk parameters
    send_gcode("$J0=0.01");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_gcode("$J1=0.01");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Set junction deviation
    send_gcode("$11=0.001");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Move to absolute position
    send_gcode("G90");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Send all movements
    for (int i = 0; i < num_positions; i++) {
        float speed_factor = 1.0f;
        if (speed_multipliers != NULL && i < num_positions) {
            speed_factor = speed_multipliers[i];
            if (speed_factor < 0.1f) speed_factor = 0.1f;
            if (speed_factor > 5.0f) speed_factor = 5.0f;
        }
        
        float feedrate = movement_speed * speed_factor * 60.0f;
        
        memset(cmd, 0, sizeof(cmd));
        snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", 
                positions[i][0], positions[i][1], feedrate);
        send_gcode(cmd);
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    // Calculate total movement time
    float total_distance = 0;
    for (int i = 1; i < num_positions; i++) {
        float dx = positions[i][0] - positions[i-1][0];
        float dy = positions[i][1] - positions[i-1][1];
        total_distance += sqrt(dx*dx + dy*dy);
    }
    
    if (isnan(total_distance) || isinf(total_distance)) {
        total_distance = 360.0f;
    }
    
    uint32_t movement_time_ms = (uint32_t)((total_distance / movement_speed) * 1000);
    movement_time_ms = (uint32_t)(movement_time_ms * movement_timeout_factor);
    
    if (movement_time_ms < 1000) movement_time_ms = 1000;
    if (movement_time_ms > 10000) movement_time_ms = 10000;
    
    debug_msg("Continuous movement started, waiting %d ms for completion", movement_time_ms);
    
    // Wait for movement to complete (BUT DON'T CHANGE SETTINGS DURING WAIT!)
    const uint32_t CHECK_INTERVAL = 100;
    uint32_t elapsed = 0;
    uint32_t idle_count = 0;
    
    while (elapsed < movement_time_ms) {
        vTaskDelay(CHECK_INTERVAL / portTICK_PERIOD_MS);
        elapsed += CHECK_INTERVAL;
        
        // Check if done early
        if (sys.state == State::Idle) {
            idle_count++;
            if (idle_count >= 2) {
                debug_msg("Movement completed early after %d ms", elapsed);
                break;
            }
        } else {
            idle_count = 0;
        }
        
        // Extra protection against crashes during long waits
        if (elapsed % 1000 == 0) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
    
    // IMPORTANT: Wait for all movement to complete before changing settings
    send_gcode("M400");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // NOW restore settings AFTER movement is complete
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "$120=%.2f", saved_accel);
    if (!send_gcode(cmd)) {
        debug_msg("WARNING: Failed to restore acceleration, continuing anyway");
        // Don't retry - just continue
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "$121=%.2f", saved_accel);
    if (!send_gcode(cmd)) {
        debug_msg("WARNING: Failed to restore acceleration, continuing anyway");
        // Don't retry - just continue
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    debug_msg("Continuous movement complete");
}

// Move with continuous rotation, allowing angles beyond 360 degrees
static void move_continuous_rotation(const float positions[][2], int num_positions, float speed_multipliers[]) {
    // Input validation
    if (num_positions <= 0 || num_positions > 50 || positions == NULL) {
        debug_msg("ERROR: Invalid parameters to move_continuous_rotation");
        return;
    }
    
    char cmd[64];
    
    // Skip movement if disabled
    if (!movement_enabled) {
        debug_msg("Movement disabled - skipping continuous rotation");
        return;
    }
    
    // Store original settings to restore later
    float saved_accel = movement_accel;
    
    // CRITICAL: Wait for any ongoing movements to complete first
    send_gcode("M400");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Set minimum viable acceleration
    float effect_accel = 10.0f;
    
    // Set acceleration parameters - with error checking and retries
    for (int retry = 0; retry < 3; retry++) {
        memset(cmd, 0, sizeof(cmd));
        snprintf(cmd, sizeof(cmd)-1, "$120=%.2f", effect_accel);
        if (send_gcode(cmd)) break;
        debug_msg("Retry %d: Setting X acceleration", retry+1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    for (int retry = 0; retry < 3; retry++) {
        memset(cmd, 0, sizeof(cmd));
        snprintf(cmd, sizeof(cmd)-1, "$121=%.2f", effect_accel);
        if (send_gcode(cmd)) break;
        debug_msg("Retry %d: Setting Y acceleration", retry+1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Move to absolute position
    send_gcode("G90");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Break the movement into smaller batches to prevent buffer overflow
    const int BATCH_SIZE = 10;
    int batches = (num_positions + BATCH_SIZE - 1) / BATCH_SIZE;
    
    debug_msg("Processing rotation in %d batches of max %d points", batches, BATCH_SIZE);
    
    for (int batch = 0; batch < batches; batch++) {
        int start_idx = batch * BATCH_SIZE;
        int end_idx = min(start_idx + BATCH_SIZE, num_positions);
        
        debug_msg("Processing rotation batch %d/%d (points %d-%d)", 
                 batch+1, batches, start_idx, end_idx-1);
        
        // Wait for previous batch to complete before sending next batch
        if (batch > 0) {
            send_gcode("M400");
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        
        // Process this batch of movements
        for (int i = start_idx; i < end_idx; i++) {
            // Safety check for speed_multipliers
            float speed_factor = 1.0f;
            if (speed_multipliers != NULL && i < num_positions) {
                speed_factor = speed_multipliers[i];
                if (speed_factor < 0.1f) speed_factor = 0.1f;
                if (speed_factor > 5.0f) speed_factor = 5.0f;
            }
            
            float feedrate = movement_speed * speed_factor * 60.0f;
            
            // Clamp positions to safe ranges for GRBL
            // Most GRBL implementations have trouble with very large values
            float target_x = positions[i][0];
            float target_y = positions[i][1];
            
            // Send commands one at a time with short delays between
            memset(cmd, 0, sizeof(cmd));
            snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", 
                    target_x, target_y, feedrate);
            
            if (!send_gcode(cmd)) {
                debug_msg("WARNING: Failed to send movement command for point %d", i);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            
            // Debug output only for a few points to reduce serial load
            if (i % 5 == 0) {
                debug_msg("Rotation point %d: X=%.1f Y=%.1f (Speed factor: %.1f)", 
                         i, target_x, target_y, speed_factor);
            }
            
            // Allow more time between commands
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        // Allow time for this batch to process
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    // Calculate maximum rotation from the input points
    float max_revolution_count = 1.0f;  // Default minimum
    for (int i = 0; i < num_positions; i++) {
        float x_revs = fabs(positions[i][0]) / 360.0f;
        float y_revs = fabs(positions[i][1]) / 360.0f;
        if (x_revs > max_revolution_count) max_revolution_count = x_revs;
        if (y_revs > max_revolution_count) max_revolution_count = y_revs;
    }
    
    // Use calculated revolutions for timing with a more conservative estimate
    uint32_t movement_time_ms = (uint32_t)(max_revolution_count * 360.0f / movement_speed * 1000 * 2.5f);
    
    // Ensure reasonable bounds for timeout
    if (movement_time_ms < 3000) movement_time_ms = 3000;
    if (movement_time_ms > 30000) movement_time_ms = 30000;
    
    debug_msg("Waiting %d ms for continuous rotation to complete", movement_time_ms);
    
    // Wait for movement to complete
    const uint32_t CHECK_INTERVAL = 500; // Longer interval
    uint32_t elapsed = 0;
    uint32_t idle_count = 0;
    
    while (elapsed < movement_time_ms) {
        vTaskDelay(CHECK_INTERVAL / portTICK_PERIOD_MS);
        elapsed += CHECK_INTERVAL;
        
        // More frequent system state reporting
        if (elapsed % 2000 == 0) {
            debug_msg("Rotation progress: %d/%d ms, state: %d", 
                     elapsed, movement_time_ms, (int)sys.state);
        }
        
        // Check if we're done early
        if (sys.state == State::Idle) {
            idle_count++;
            if (idle_count >= 2) {
                debug_msg("Rotation completed early after %d ms", elapsed);
                break;
            }
        } else {
            idle_count = 0;
        }
    }
    
    // CRITICAL: Make absolutely sure movement is complete
    send_gcode("M400");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Restore settings
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "$120=%.2f", saved_accel);
    send_gcode(cmd);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "$121=%.2f", saved_accel);
    send_gcode(cmd);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Normalize final position
    float final_x = positions[num_positions-1][0];
    float final_y = positions[num_positions-1][1];
    
    // Convert to normalized angles (0-360°)
    while (final_x < 0) final_x += 360.0f;
    while (final_x >= 360.0f) final_x -= 360.0f;
    while (final_y < 0) final_y += 360.0f;
    while (final_y >= 360.0f) final_y -= 360.0f;
    
    // Force the final position
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "G92 X%.1f Y%.1f", final_x, final_y);
    send_gcode(cmd);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    debug_msg("Continuous rotation complete");
}

// Get the current position (angles) of the clock hands
static void get_current_position(float &x, float &y) {
    // Get current position from GRBL system
    float machine_position[N_AXIS];
    system_convert_array_steps_to_mpos(machine_position, sys_position);
    
    // X = Minute hand angle, Y = Hour hand angle
    x = machine_position[X_AXIS];
    y = machine_position[Y_AXIS];
    
    // Error checking for NaN/infinity
    if (isnan(x) || isinf(x)) {
        debug_msg("WARNING: Invalid X position detected, using fallback");
        x = 0.0f;
    }
    
    if (isnan(y) || isinf(y)) {
        debug_msg("WARNING: Invalid Y position detected, using fallback");
        y = 0.0f;
    }
    
    // Ensure angles are within 0-360 range
    while (x < 0) x += 360.0f;
    while (x >= 360) x -= 360.0f;
    while (y < 0) y += 360.0f;
    while (y >= 360) y -= 360.0f;
}

// Add this function to periodically verify position
static void verify_and_correct_time_position() {
    // 1. Get what the current time angles SHOULD be
    float expected_hour_angle, expected_minute_angle;
    time_to_angles(current_hour, current_minute, expected_hour_angle, expected_minute_angle);
    
    // 2. Get what the controller THINKS the current position is
    float current_min_angle, current_hour_angle;
    get_current_position(current_min_angle, current_hour_angle);
    
    // 3. Check if there's significant drift (more than 2 degrees)
    float minute_diff = fabs(expected_minute_angle - current_min_angle);
    float hour_diff = fabs(expected_hour_angle - current_hour_angle);
    
    // Handle wraparound at 360 degrees
    if (minute_diff > 180.0f) minute_diff = 360.0f - minute_diff;
    if (hour_diff > 180.0f) hour_diff = 360.0f - hour_diff;
    
    if (minute_diff > 2.0f || hour_diff > 2.0f) {
        debug_msg("Position drift detected: Minute: %.1f° (vs %.1f°), Hour: %.1f° (vs %.1f°)", 
                 current_min_angle, expected_minute_angle, 
                 current_hour_angle, expected_hour_angle);
        
        // 4. First move to the correct position physically
        movement_speed = 60.0f; // Slow, deliberate movement
        move_to_angles(expected_minute_angle, expected_hour_angle);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for movement to complete
        
        // 5. Then ensure the coordinate system matches
        char cmd[64];
        memset(cmd, 0, sizeof(cmd));
        snprintf(cmd, sizeof(cmd)-1, "G92 X%.1f Y%.1f", 
 
                 expected_minute_angle, expected_hour_angle);
        send_gcode(cmd);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        
        debug_msg("Position re-synchronized to match current time");
    }
}

// Move this function definition OUTSIDE of clockEngineTask
bool gcode_unknown_command_execute(char *line) {
    // Check for CLOCK commands
    if (strncmp(line, "CLOCK", 5) == 0) {
        // Extract command after CLOCK prefix
        char* cmd = line + 5;
        while (*cmd == ' ') cmd++; // Skip spaces
        
        if (strncmp(cmd, "REHOME", 6) == 0) {
            rehome_clock();
            return true;
        }
        
        // Add the DEBUG command handler here too, inside the CLOCK check
        if (strncmp(cmd, "DEBUG", 5) == 0) {
            char* level = cmd + 5;
            while (*level == ' ') level++; // Skip spaces
            
            int debugLevel = atoi(level);
            if (debugLevel >= DEBUG_MINIMAL && debugLevel <= DEBUG_VERBOSE) {
                active_debug_level = (DebugLevel)debugLevel;
                debug_msg("Debug level set to %d", active_debug_level);
            } else {
                debug_msg("Invalid debug level: %d (valid: 0-2)", debugLevel);
            }
            return true;
        }
    }
    
    return false;
}

// Add this function to do a complete rehoming sequence
void rehome_clock() {
    debug_msg("Starting complete rehoming sequence");
    
    // Ensure system is unlocked
    send_gcode("$X");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // First move to a safe position to avoid limit switch crashes
    debug_msg("Moving to safe position before homing");
    send_gcode("G90");  // Absolute positioning
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_gcode("G1 X180 Y180 F10800");  // Move to 6:00 position slowly
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Home hour hand (Y axis) first
    debug_msg("Homing hour hand (Y axis)");
    mc_homing_cycle(0x2);  // Home Y axis
    
    // Wait for homing to complete with timeout
    uint32_t timeout = millis() + 15000;
    while (sys.state != State::Idle && millis() < timeout) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (sys.state != State::Idle) {
        debug_msg("WARNING: Y homing timed out, attempting recovery");
        send_gcode("$X");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } else {
        debug_msg("Y axis homing complete");
    }
    
    // Then home minute hand (X axis)
    debug_msg("Homing minute hand (X axis)");
    mc_homing_cycle(0x1);  // Home X axis
    
    // Wait for homing to complete
    timeout = millis() + 15000;
    while (sys.state != State::Idle && millis() < timeout) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (sys.state != State::Idle) {
        debug_msg("WARNING: X homing timed out, attempting recovery");
        send_gcode("$X");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } else {
        debug_msg("X axis homing complete");
    }
    
    // Move to 12:00 position after homing
    debug_msg("Moving to 12:00 position after homing");
    send_gcode("G1 X0 Y0 F3600");  // Slow, controlled movement
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Update system status
    initial_homing_done = true;
    
    // Update current time from RTC if available
    if (rtc_initialized) {
        update_time_from_rtc();
        debug_msg("Clock rehomed and synchronized to %02d:%02d", 
                 current_hour, current_minute);
    } else {
        debug_msg("Clock rehomed (RTC not available)");
    }
    
    // Verify position matches current time
    verify_and_correct_time_position();
}
