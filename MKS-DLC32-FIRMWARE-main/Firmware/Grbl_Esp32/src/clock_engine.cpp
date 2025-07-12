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



//===================================
// CONFIGURATION AND TIME VARIABLES
//===================================

// Movement control flag - set to false to disable all physical movement
static bool movement_enabled = true;

// Movement speed in degrees per second (1-100 range)
// Lower values = slower movement
static float movement_speed = 30.0f;  // 30 degrees per second default

// Movement acceleration in degrees per second^2 (100-1000 range)
// Lower values = gentler acceleration/deceleration
static float movement_accel = 300.0f; // Default acceleration

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
    MODE_DIRECT_ANGLE       // Move to specific angles
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
#define MAX_SEQUENCE_STEPS 10
static SequenceStep sequence[MAX_SEQUENCE_STEPS] = {
    // Time sequence
    {MODE_SPECIFIC_TIME, 12, 0, 0, 0, "", 5000},             // Noon
    {MODE_SPECIFIC_TIME, 4, 20, 0, 0, "", 5000},             // 4:20
    {MODE_SPECIFIC_TIME, 7, 7, 0, 0, "", 5000},              // 7:07
    
    // Cosmic alignment (using direct angles)
    {MODE_DIRECT_ANGLE, 0, 0, 180, 180, "", 5000},           // Hands aligned at 6:00
    {MODE_DIRECT_ANGLE, 0, 0, 90, 270, "", 5000},            // Perpendicular hands (3:00/9:00)
    
    // Playful patterns
    {MODE_PLAY_FILE, 0, 0, 0, 0, "spin420.nc", 0},           // Play G-code file
    
    // Mystical times
    {MODE_SPECIFIC_TIME, 11, 11, 0, 0, "", 5000},            // 11:11
    {MODE_SPECIFIC_TIME, 12, 34, 0, 0, "", 5000},            // 12:34
    
    // Kaleidoscope effect
    {MODE_DIRECT_ANGLE, 0, 0, 45, 315, "", 5000},            // 1:30/10:30
    
    // Return to current time
    {MODE_CURRENT_TIME, 0, 0, 0, 0, "", 15000}               // Current time
};

// Sequence tracking variables
static int current_sequence_step = 0;
static uint32_t sequence_step_start_time = 0;
static bool sequence_active = true;

// System state tracking
static bool system_ready = false;
static bool initial_homing_done = false;
static bool file_playback_active = false;

//===================================
// UTILITY FUNCTIONS
//===================================

// Forward declaration of the position function
static void get_current_position(float &x, float &y);

// Function to send G-code commands
static bool send_gcode(const char *line) {
    // Copy the line to a temporary buffer for safety
    static char buf[96];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    
    // Execute the G-code command
    return (gc_execute_line(buf, CLIENT_SERIAL) == Error::Ok);
}

// Debug message function
static void debug_msg(const char *format, ...) {
    // Format and send debug messages to the serial console
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, buffer);
}

// Convert time to clock hand angles
static void time_to_angles(int hour, int minute, float &hour_angle, float &minute_angle) {
    // Minute hand: 360° ÷ 60 minutes = 6° per minute
    minute_angle = minute * 6.0f;
    
    // Hour hand: 360° ÷ 12 hours = 30° per hour + 0.5° per minute
    hour_angle = (hour % 12) * 30.0f + minute * 0.5f;
}

// Move clock hands to specific angles - direct linear movement with speed control
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
    
    // Get current position to calculate movement distance
    float current_min_angle = 0.0f;
    float current_hour_angle = 0.0f;
    get_current_position(current_min_angle, current_hour_angle);
    
    // Calculate movement distances (accounting for wraparound)
    float min_dist = fabs(minute_angle - current_min_angle);
    if (min_dist > 180) min_dist = 360 - min_dist;
    
    float hour_dist = fabs(hour_angle - current_hour_angle);
    if (hour_dist > 180) hour_dist = 360 - hour_dist;
    
    // Use the larger distance to calculate movement time
    float max_distance = max(min_dist, hour_dist);
    
    // Calculate movement time in milliseconds based on speed
    uint32_t movement_time_ms = (uint32_t)((max_distance / movement_speed) * 1000);
    
    // Add additional time for acceleration/deceleration (simplified calculation)
    movement_time_ms += (uint32_t)(2000.0f * sqrt(max_distance / movement_accel));
    
    // Apply timeout factor for safety
    movement_time_ms = (uint32_t)(movement_time_ms * movement_timeout_factor);
    
    // Ensure minimum movement time
    if (movement_time_ms < 500) movement_time_ms = 500;
    
    // Track desired position even when movement is disabled
    target_minute_angle = minute_angle;
    target_hour_angle = hour_angle;
    
    // Skip physical movement if disabled
    if (!movement_enabled) {
        debug_msg("Movement disabled - virtual position X%.1f Y%.1f", minute_angle, hour_angle);
        return;
    }
    
    // Set acceleration
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "$120=%.1f", movement_accel);
    send_gcode(cmd);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    snprintf(cmd, sizeof(cmd)-1, "$121=%.1f", movement_accel);
    send_gcode(cmd);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Move to absolute position
    send_gcode("G90"); // Absolute positioning
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Calculate feedrate in mm/min from degrees/sec (1mm = 1deg, 60sec = 1min)
    float feedrate = movement_speed * 60.0f;
    
    // Create command with angles and speed and send it
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F%.1f", minute_angle, hour_angle, feedrate);
    debug_msg("Moving to X%.1f Y%.1f (speed:%.1f deg/s, est. time:%dms)", 
              minute_angle, hour_angle, movement_speed, movement_time_ms);
    send_gcode(cmd);
    
    // Wait for movement to complete
    debug_msg("Waiting for movement to complete...");
    vTaskDelay(movement_time_ms / portTICK_PERIOD_MS);
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

// Start playback of a G-code file
static void play_gcode_file(const char* filename) {
    if (!file_playback_active) {
        char cmd[64];
        sprintf(cmd, "$Play=/%s", filename);
        debug_msg("Playing file: %s", filename);
        send_gcode(cmd);
        file_playback_active = true;
    }
}

// Move to the next step in the sequence
static void advance_sequence() {
    // Move to the next step, wrapping around if needed
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
    }
}

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
        
        // INITIALIZATION: Always try to clear alarms at startup
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
            debug_msg("System initialized, preparing for homing");
            
            // Move hands to show that the system is alive, even before homing
            debug_msg("Moving hands to show system is active (at reduced speed)");
            movement_enabled = true; // Force movement on
            move_to_angles(90, 270); // Move to 3:00/9:00 position (perpendicular)
            vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 seconds between movements
            move_to_angles(180, 180); // Move to 6:00 position (aligned)
            vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 seconds between movements
            move_to_angles(0, 0); // Return to 12:00 position
            vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 seconds between movements
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
            movement_speed = 15.0f; // Slower speed for testing
            
            move_to_angles(90, 270); // 3:00/9:00 position
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            
            move_to_angles(180, 180); // 6:00 position
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            
            move_to_angles(0, 0); // Back to 12:00 position
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            
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
                    if (last_displayed_minute != current_minute) {
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
        
        // TIME UPDATE: Simple time increment every minute
        // In a real implementation, you'd use a proper RTC
        static uint32_t last_minute_update = 0;
        if (now - last_minute_update >= 60000) { // Every 60 seconds
            current_minute++;
            if (current_minute >= 60) {
                current_minute = 0;
                current_hour = (current_hour + 1) % 24;
            }
            last_minute_update = now;
            debug_msg("Time updated: %02d:%02d", current_hour, current_minute);
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
        
        // Short delay to prevent task starvation
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//===================================
// PUBLIC API FUNCTIONS
//===================================

// Set the current time
void clock_set_time(int hour, int minute) {
    current_hour = hour % 24;
    current_minute = minute % 60;
    
    // If currently showing real time, update the display
    // Remove the system_ready check to allow movement anytime
    if (current_mode == MODE_CURRENT_TIME) {
        display_time(current_hour, current_minute);
    }
}

// Set the clock mode directly
void clock_set_mode(int mode) {
    current_mode = static_cast<ClockMode>(mode);
    
    // Apply the new mode immediately
    switch (current_mode) {
        case MODE_CURRENT_TIME:
            display_time(current_hour, current_minute);
            break;
            
        case MODE_SPECIFIC_TIME:
            display_time(target_hour, target_minute);
            break;
            
        case MODE_PLAY_FILE:
            file_playback_active = false; // Force restart
            break;
            
        case MODE_DIRECT_ANGLE:
            move_to_angles(target_minute_angle, target_hour_angle);
            break;
    }
}

// Start or stop the sequence
void clock_set_sequence_active(bool active) {
    sequence_active = active;
    if (active) {
        current_sequence_step = 0;
        sequence_step_start_time = millis();
    }
}

// Set a specific time to display
void clock_set_target_time(int hour, int minute) {
    target_hour = hour % 24;
    target_minute = minute % 60;
    
    // If currently showing specific time, update the display
    // Remove the system_ready check
    if (current_mode == MODE_SPECIFIC_TIME) {
        display_time(target_hour, target_minute);
    }
}

// Move to a specific angle
void clock_set_angles(float minute_angle, float hour_angle) {
    target_minute_angle = minute_angle;
    target_hour_angle = hour_angle;
    
    // If currently in direct angle mode, update the display
    // Remove the system_ready check
    if (current_mode == MODE_DIRECT_ANGLE) {
        move_to_angles(minute_angle, hour_angle);
    }
}

// Play a specific G-code file
void clock_play_file(const char* filename) {
    strncpy(file_to_play, filename, sizeof(file_to_play)-1);
    file_to_play[sizeof(file_to_play)-1] = '\0';
    
    if (current_mode == MODE_PLAY_FILE && system_ready && initial_homing_done) {
        file_playback_active = false; // Force restart
    }
}

// Update a specific sequence step
void clock_set_sequence_step(int step_index, int mode, int hour, int minute, 
                           float min_angle, float hour_angle, 
                           const char* filename, uint32_t duration_ms) {
    if (step_index >= 0 && step_index < MAX_SEQUENCE_STEPS) {
        sequence[step_index].mode = static_cast<ClockMode>(mode);
        sequence[step_index].hour = hour;
        sequence[step_index].minute = minute;
        sequence[step_index].min_angle = min_angle;
        sequence[step_index].hour_angle = hour_angle;
        strncpy(sequence[step_index].filename, filename, sizeof(sequence[step_index].filename)-1);
        sequence[step_index].filename[sizeof(sequence[step_index].filename)-1] = '\0';
        sequence[step_index].duration_ms = duration_ms;
    }
}

// Immediately jump to a specific sequence step
void clock_jump_to_sequence_step(int step_index) {
    if (step_index >= 0 && step_index < MAX_SEQUENCE_STEPS) {
        current_sequence_step = step_index;
        sequence_step_start_time = millis();
        advance_sequence(); // This will apply the settings from the new step
    }
}

// Set the movement enabled/disabled
void clock_set_movement_enabled(bool enabled) {
    movement_enabled = enabled;
    debug_msg("Clock movement %s", enabled ? "enabled" : "disabled");
}

// Set the movement speed in degrees per second
void clock_set_movement_speed(float speed_deg_per_sec, float accel_deg_per_sec_sq) {
    // Validate and bound input values
    if (speed_deg_per_sec < 1.0f) speed_deg_per_sec = 1.0f;
    if (speed_deg_per_sec > 100.0f) speed_deg_per_sec = 100.0f;
    
    if (accel_deg_per_sec_sq < 100.0f) accel_deg_per_sec_sq = 100.0f;
    if (accel_deg_per_sec_sq > 1000.0f) accel_deg_per_sec_sq = 1000.0f;
    
    movement_speed = speed_deg_per_sec;
    movement_accel = accel_deg_per_sec_sq;
    
    debug_msg("Movement speed set to %.1f deg/s, accel %.1f deg/s²", 
              movement_speed, movement_accel);
}

// Set the movement timeout factor
void clock_set_movement_timeout(float timeout_factor) {
    if (timeout_factor < 1.0f) timeout_factor = 1.0f;
    if (timeout_factor > 5.0f) timeout_factor = 5.0f;
    
    movement_timeout_factor = timeout_factor;
    debug_msg("Movement timeout factor set to %.1f", movement_timeout_factor);
}

// Set up a custom M-code handler for clock control
bool gcode_unknown_command_execute(char *line) {
    // Special emergency unlock/enable command
    if (strncmp(line, "M999", 4) == 0) {
        debug_msg("Emergency initialization");
        send_gcode("$X"); // Unlock
        vTaskDelay(500 / portTICK_PERIOD_MS);
        send_gcode("$1=255"); // Enable steppers
        vTaskDelay(500 / portTICK_PERIOD_MS);
        send_gcode("G21"); // Set mm units
        vTaskDelay(500 / portTICK_PERIOD_MS);
        system_ready = true; // Force system ready
        movement_enabled = true; // Force movement enabled
        return true;
    }
    
    // M900 HH:MM - Set the current time
    if (strncmp(line, "M900", 4) == 0) {
        int hour = atoi(line + 5);
        int minute = atoi(line + 8);
        clock_set_time(hour, minute);
        debug_msg("Time set to %02d:%02d", hour, minute);
        return true;
    }
    
    // M901 MODE - Set the clock mode
    if (strncmp(line, "M901", 4) == 0) {
        int mode = atoi(line + 5);
        clock_set_mode(mode);
        debug_msg("Mode set to %d", mode);
        return true;
    }
    
    // M902 HH:MM - Set a specific target time
    if (strncmp(line, "M902", 4) == 0) {
        int hour = atoi(line + 5);
        int minute = atoi(line + 8);
        clock_set_target_time(hour, minute);
        debug_msg("Target time set to %02d:%02d", hour, minute);
        return true;
    }
    
    // M903 X[angle] Y[angle] - Set specific angles
    if (strncmp(line, "M903", 4) == 0) {
        char* x_pos = strstr(line, "X");
        char* y_pos = strstr(line, "Y");
        
        float x_angle = 0.0f;
        float y_angle = 0.0f;
        
        if (x_pos) x_angle = atof(x_pos + 1);
        if (y_pos) y_angle = atof(y_pos + 1);
        
        clock_set_angles(x_angle, y_angle);
        debug_msg("Angles set to X%.1f Y%.1f", x_angle, y_angle);
        return true;
    }
    
    // M904 [filename] - Play a file
    if (strncmp(line, "M904", 4) == 0) {
        char filename[32] = {0};
        strncpy(filename, line + 5, sizeof(filename)-1);
        
        // Trim leading/trailing spaces
        char* start = filename;
        while (*start && isspace(*start)) start++;
        
        char* end = start + strlen(start) - 1;
        while (end > start && isspace(*end)) *end-- = '\0';
        
        clock_play_file(start);
        debug_msg("Playing file: %s", start);
        return true;
    }
    
    // M905 [0/1] - Start/stop sequence
    if (strncmp(line, "M905", 4) == 0) {
        int active = atoi(line + 5);
        clock_set_sequence_active(active != 0);
        debug_msg("Sequence %s", active ? "started" : "stopped");
        return true;
    }
    
    // M906 [0/1] - Disable/enable movement
    if (strncmp(line, "M906", 4) == 0) {
        int enabled = atoi(line + 5);
        clock_set_movement_enabled(enabled != 0);
        debug_msg("Movement %s", enabled ? "enabled" : "disabled");
        return true;
    }
    
    // M907 S[speed] A[accel] - Set movement speed and acceleration
    if (strncmp(line, "M907", 4) == 0) {
        float speed = movement_speed;
        float accel = movement_accel;
        
        char* s_pos = strstr(line, "S");
        char* a_pos = strstr(line, "A");
        
        if (s_pos) speed = atof(s_pos + 1);
        if (a_pos) accel = atof(a_pos + 1);
        
        clock_set_movement_speed(speed, accel);
        return true;
    }
    
    // M908 T[factor] - Set movement timeout factor
    if (strncmp(line, "M908", 4) == 0) {
        float timeout = movement_timeout_factor;
        
        char* t_pos = strstr(line, "T");
        if (t_pos) timeout = atof(t_pos + 1);
        
        clock_set_movement_timeout(timeout);
        return true;
    }
    
    return false;
}

// Helper function to get current position
static void get_current_position(float &x, float &y) {
    // Default to 0,0 if we can't get position
    x = 0.0f;
    y = 0.0f;
    
    // Check if we're in a state where we can query position
    if (sys.state == State::Idle) {
        // Get position from motor steps (converted to mm)
        x = gc_state.position[X_AXIS];
        y = gc_state.position[Y_AXIS];
    }
}
