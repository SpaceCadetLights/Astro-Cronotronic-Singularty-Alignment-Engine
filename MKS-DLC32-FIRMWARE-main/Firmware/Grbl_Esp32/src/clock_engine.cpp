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

//===================================
// CONFIGURATION AND TIME VARIABLES
//===================================

// Current time (24-hour format)
static int current_hour = 12;   // 0-23
static int current_minute = 0;  // 0-59

// Clock operation modes
enum ClockMode {
    MODE_CURRENT_TIME = 0,  // Show the current time
    MODE_SPECIFIC_TIME,     // Show a specific time
    MODE_PLAY_FILE,         // Play a G-code file
    MODE_DIRECT_ANGLE,      // Move to specific angles
    MODE_SPIRAL_MOTION,     // Spiral movement pattern
    MODE_PENDULUM_SWING,    // Pendulum swinging motion
    MODE_BOUNCE_EFFECT,     // Bouncing animation
    MODE_RANDOM_DANCE       // Random playful movements
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
    ClockMode mode;
    int hour;
    int minute;
    float min_angle;
    float hour_angle;
    char filename[32];
    uint32_t duration_ms;  // How long to stay in this mode
    int animation_style;   // 0=spiral, 1=pendulum, 2=bounce, 3=random, -1=none
} SequenceStep;

#define MAX_SEQUENCE_STEPS 10
static SequenceStep sequence[MAX_SEQUENCE_STEPS] = {
    // Time travel sequence
    {MODE_SPECIFIC_TIME, 12, 0, 0, 0, "", 5000, 0},             // Noon - Spiral animation
    {MODE_SPECIFIC_TIME, 4, 20, 0, 0, "", 5000, 2},             // 4:20 - Bounce animation
    {MODE_SPECIFIC_TIME, 7, 7, 0, 0, "", 5000, 1},              // 7:07 - Pendulum animation
    
    // Cosmic alignment (using direct angles)
    {MODE_DIRECT_ANGLE, 0, 0, 180, 180, "", 5000, 3},           // Hands aligned at 6:00 - Random dance
    {MODE_DIRECT_ANGLE, 0, 0, 90, 270, "", 5000, 1},            // Perpendicular hands (3:00/9:00) - Pendulum
    
    // Playful patterns
    {MODE_PLAY_FILE, 0, 0, 0, 0, "spin420.nc", 0, -1},          // Play G-code file
    
    // Mystical times
    {MODE_SPECIFIC_TIME, 11, 11, 0, 0, "", 5000, 2},            // 11:11 - Bounce animation
    {MODE_SPECIFIC_TIME, 12, 34, 0, 0, "", 5000, 1},            // 12:34 - Pendulum animation
    
    // Kaleidoscope effect
    {MODE_DIRECT_ANGLE, 0, 0, 45, 315, "", 5000, 3},            // 1:30/10:30 - Random dance
    
    // Return to current time
    {MODE_CURRENT_TIME, 0, 0, 0, 0, "", 15000, 0}               // Current time - Spiral animation
};

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

// Forward declarations for animation functions
static void spiral_movement(float end_x, float end_y, int revolutions);
static void pendulum_swing(float end_x, float end_y, int swings);
static void bounce_effect(float end_x, float end_y, int bounces);
static void random_dance(float end_x, float end_y, int dance_moves);
static void animated_move_to_angles(float minute_angle, float hour_angle, int animation_style);
static void get_current_position(float &x, float &y);

// Function to send G-code commands
static bool send_gcode(const char *line) {
    static char buf[96];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    return (gc_execute_line(buf, CLIENT_SERIAL) == Error::Ok);
}

// Debug message function
static void debug_msg(const char *format, ...) {
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

// Move clock hands to specific angles
static void move_to_angles(float minute_angle, float hour_angle, bool use_animation = true) {
    // If animation is disabled or we need precise positioning, use direct movement
    if (!use_animation) {
        char cmd[64];
        // Move to absolute position
        send_gcode("G90");
        // Create command with angles
        sprintf(cmd, "G0 X%.1f Y%.1f F3000", minute_angle, hour_angle);
        // Send the move command
        debug_msg("Direct move to X%.1f Y%.1f", minute_angle, hour_angle);
        send_gcode(cmd);
        return;
    }
    
    // Use animated movement with random animation style
    animated_move_to_angles(minute_angle, hour_angle, -1); // -1 means random style
}

// Move clock to display a specific time
static void display_time(int hour, int minute, int animation_style = -1) {
    float hour_angle, minute_angle;
    time_to_angles(hour, minute, hour_angle, minute_angle);
    
    if (animation_style >= 0) {
        animated_move_to_angles(minute_angle, hour_angle, animation_style);
    } else {
        move_to_angles(minute_angle, hour_angle, true); // Use random animation
    }
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
    
    switch (current_mode) {
        case MODE_CURRENT_TIME:
            debug_msg("Sequence: Showing current time");
            break;
            
        case MODE_SPECIFIC_TIME:
            target_hour = sequence[current_sequence_step].hour;
            target_minute = sequence[current_sequence_step].minute;
            debug_msg("Sequence: Showing specific time %02d:%02d (animation %d)", 
                      target_hour, target_minute, sequence[current_sequence_step].animation_style);
            display_time(target_hour, target_minute, sequence[current_sequence_step].animation_style);
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
            debug_msg("Sequence: Moving to angles X%.1f Y%.1f (animation %d)", 
                      target_minute_angle, target_hour_angle, sequence[current_sequence_step].animation_style);
            animated_move_to_angles(target_minute_angle, target_hour_angle, 
                                  sequence[current_sequence_step].animation_style);
            break;
    }
}

//===================================
// ANIMATION FUNCTIONS
//===================================

// Create a spiral movement pattern - SAFER VERSION
static void spiral_movement(float end_x, float end_y, int revolutions = 2) {
    float start_x = 0;
    float start_y = 0;
    
    // Get current position safely
    get_current_position(start_x, start_y);
    
    debug_msg("Starting spiral movement to X%.1f Y%.1f", end_x, end_y);
    
    // Use G-code arcs to create spiral effect
    send_gcode("G90"); // Absolute positioning
    vTaskDelay(50 / portTICK_PERIOD_MS); // Add delay after each command
    
    // Calculate center point for the spiral
    float center_x = (start_x + end_x) / 2;
    float center_y = (start_y + end_y) / 2;
    
    // Create multiple arc movements that gradually get closer to the target
    float dx = end_x - center_x;
    float dy = end_y - center_y;
    float radius = sqrtf(dx*dx + dy*dy);
    
    // Safety check - ensure radius is valid
    if (isnan(radius) || radius < 0.1f) {
        radius = 10.0f; // Fallback to a safe default
    }
    
    // Number of segments in our spiral - limit to reasonable range
    int segments = 12 * revolutions;
    if (segments > 50) segments = 50; // Limit max segments
    if (segments < 5) segments = 5;   // Ensure minimum segments
    
    // Move to starting point first for safety
    char cmd[64];
    memset(cmd, 0, sizeof(cmd)); // Clear buffer
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F2000", start_x, start_y);
    send_gcode(cmd);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Longer delay at start
    
    // Generate spiral points
    for (int i = 0; i < segments; i++) {
        float ratio = (float)(i+1) / segments;
        if (ratio > 1.0f) ratio = 1.0f; // Safety bounds
        
        float angle = ratio * 2 * M_PI * revolutions;
        
        // Gradually decrease the radius to create spiral
        float current_radius = radius * (1.0f - 0.8f * ratio);
        
        // Calculate position on the spiral
        float x = center_x + current_radius * cosf(angle);
        float y = center_y + current_radius * sinf(angle);
        
        // Get closer to final destination with each iteration
        x = x * (1.0f - ratio) + end_x * ratio;
        y = y * (1.0f - ratio) + end_y * ratio;
        
        // Check for invalid values
        if (isnan(x) || isnan(y) || isinf(x) || isinf(y)) {
            debug_msg("Invalid spiral point calculated, skipping");
            continue;
        }
        
        // Create movement command with safety buffer clearing
        memset(cmd, 0, sizeof(cmd));
        snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F2000", x, y);
        send_gcode(cmd);
        
        // Longer delay to prevent command buffer overflow
        vTaskDelay(70 / portTICK_PERIOD_MS);
    }
    
    // Final precise positioning with extra safety
    vTaskDelay(200 / portTICK_PERIOD_MS); // Wait longer before final position
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F1000", end_x, end_y);
    send_gcode(cmd);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait after final move
}

// Create a pendulum swinging effect
static void pendulum_swing(float end_x, float end_y, int swings = 3) {
    float start_x = 0;
    float start_y = 0;
    
    // Get current position
    get_current_position(start_x, start_y);
    
    debug_msg("Starting pendulum swing to X%.1f Y%.1f", end_x, end_y);
    
    // Use G-code arcs to create pendulum effect
    send_gcode("G90"); // Absolute positioning
    
    // Calculate distance
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float distance = sqrtf(dx*dx + dy*dy);
    
    // Calculate angle between start and end points
    float angle = atan2f(dy, dx);
    
    // Pendulum effect: oscillate with decreasing amplitude
    for (int i = 0; i < swings * 2; i++) {
        float completion = (float)(i+1) / (swings * 2);
        float amplitude = (1.0f - completion) * 0.5f; // Decreasing amplitude
        
        // Oscillation pattern
        float swing_factor = sinf(M_PI * (i+1));
        
        // Calculate position perpendicular to the direct path
        float perpendicular_angle = angle + M_PI/2;
        float offset_x = amplitude * distance * cosf(perpendicular_angle) * swing_factor;
        float offset_y = amplitude * distance * sinf(perpendicular_angle) * swing_factor;
        
        // Calculate position along the path
        float path_ratio = powf(completion, 0.7); // Non-linear progression
        float path_x = start_x + dx * path_ratio;
        float path_y = start_y + dy * path_ratio;
        
        // Apply perpendicular offset
        float x = path_x + offset_x;
        float y = path_y + offset_y;
        
        // Create movement command
        char cmd[64];
        sprintf(cmd, "G1 X%.1f Y%.1f F2000", x, y);
        send_gcode(cmd);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Final precise positioning
    char cmd[64];
    sprintf(cmd, "G1 X%.1f Y%.1f F1000", end_x, end_y);
    send_gcode(cmd);
}

// Create a bouncing effect animation - SAFER VERSION
static void bounce_effect(float end_x, float end_y, int bounces = 3) {
    float start_x = 0;
    float start_y = 0;
    
    // Get current position safely
    get_current_position(start_x, start_y);
    
    debug_msg("Starting bounce effect to X%.1f Y%.1f", end_x, end_y);
    
    send_gcode("G90"); // Absolute positioning
    
    // Calculate direct line segments
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    
    // Limit bounce count to prevent issues
    if (bounces > 5) bounces = 5;
    if (bounces < 1) bounces = 1;
    
    // Generate a series of "bounces" along the path - fewer segments to reduce computation
    int segments = 15;
    
    // Do direct movement for first 20% of the path
    char cmd[64];
    sprintf(cmd, "G1 X%.1f Y%.1f F2000", 
            start_x + dx * 0.2f, 
            start_y + dy * 0.2f);
    send_gcode(cmd);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Now do bounces for the remaining 80%
    for (int i = 3; i < segments; i++) {
        // Safe calculation of t from 0.2 to 1.0
        float t = 0.2f + (float)(i-3) / (float)(segments-3) * 0.8f;
        
        // Position along direct path
        float x = start_x + t * dx;
        float y = start_y + t * dy;
        
        // Add bouncing effect - a series of dampened bounces
        float bounce = 0;
        for (int j = 0; j < bounces; j++) {
            // Safe calculations with bounded values
            float decay = j == 0 ? 1.0f : 0.5f * decay; // Simpler decay calculation
            float frequency = 8.0f + j * 4.0f; // Linear frequency increase, safer than exponential
            float wave = sinf(t * frequency * M_PI);
            
            // Bound wave value to prevent extreme values
            if (wave > 1.0f) wave = 1.0f;
            if (wave < -1.0f) wave = -1.0f;
            
            bounce += decay * wave;
        }
        
        // Limit bounce magnitude to prevent extreme values
        if (bounce > 1.0f) bounce = 1.0f;
        if (bounce < -1.0f) bounce = -1.0f;
        
        // Apply bounce perpendicular to movement direction
        float path_angle = atan2f(dy, dx);
        float perpendicular_angle = path_angle + M_PI/2;
        
        // Scale bounce more conservatively
        float distance = sqrtf(dx*dx + dy*dy);
        float scale = distance * 0.05f; // Reduced from 0.08f to 0.05f
        
        // Apply bounce effect
        x += scale * bounce * cosf(perpendicular_angle);
        y += scale * bounce * sinf(perpendicular_angle);
        
        // Create movement command with bounds checking
        if (isnan(x) || isnan(y)) {
            // If we somehow got a NaN, skip this point
            continue;
        }
        
        // Format command with error checking
        memset(cmd, 0, sizeof(cmd)); // Clear buffer first
        snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F2000", x, y);
        send_gcode(cmd);
        vTaskDelay(70 / portTICK_PERIOD_MS);
    }
    
    // Final precise positioning with extra safety delay
    vTaskDelay(200 / portTICK_PERIOD_MS);
    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F1000", end_x, end_y);
    send_gcode(cmd);
}

// Create random playful movements before reaching the target
static void random_dance(float end_x, float end_y, int dance_moves = 5) {
    float start_x = 0;
    float start_y = 0;
    
    // Get current position
    get_current_position(start_x, start_y);
    
    debug_msg("Starting random dance to X%.1f Y%.1f", end_x, end_y);
    
    send_gcode("G90"); // Absolute positioning
    
    // Calculate the midpoint between start and end
    float mid_x = (start_x + end_x) / 2;
    float mid_y = (start_y + end_y) / 2;
    
    // Calculate distance for scaling random movements
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float distance = sqrtf(dx*dx + dy*dy);
    float random_range = distance * 0.3f; // 30% of distance
    
    // Generate random waypoints that eventually lead to the target
    for (int i = 0; i < dance_moves; i++) {
        float completion = (float)(i+1) / (dance_moves + 1);
        
        // Generate a random offset that gets smaller as we approach the target
        float random_scale = (1.0f - powf(completion, 2)) * random_range;
        
        // Random offsets
        float offset_x = ((float)rand() / RAND_MAX * 2 - 1) * random_scale;
        float offset_y = ((float)rand() / RAND_MAX * 2 - 1) * random_scale;
        
        // Position along direct path with increasing bias toward destination
        float path_x = start_x + dx * completion;
        float path_y = start_y + dy * completion;
        
        // Apply random offset
        float x = path_x + offset_x;
        float y = path_y + offset_y;
        
        // Create movement command with varying speed for playfulness
        float speed = 1000 + (float)rand() / RAND_MAX * 2000;
        char cmd[64];
        sprintf(cmd, "G1 X%.1f Y%.1f F%.0f", x, y, speed);
        send_gcode(cmd);
        
        // Random delays between moves
        int delay = 50 + rand() % 150;
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
    
    // Final precise positioning
    char cmd[64];
    sprintf(cmd, "G1 X%.1f Y%.1f F1000", end_x, end_y);
    send_gcode(cmd);
}

// Unified animated movement function that selects an animation style
static void animated_move_to_angles(float minute_angle, float hour_angle, int animation_style = -1) {
    // Validate input angles
    if (isnan(minute_angle) || isnan(hour_angle) ||
        isinf(minute_angle) || isinf(hour_angle)) {
        debug_msg("WARNING: Invalid angle inputs to animated_move_to_angles");
        return;
    }
    
    // Bound angles to reasonable ranges (0-360)
    while (minute_angle < 0) minute_angle += 360.0f;
    while (minute_angle >= 360) minute_angle -= 360.0f;
    while (hour_angle < 0) hour_angle += 360.0f;
    while (hour_angle >= 360) hour_angle -= 360.0f;
    
    // Select a random animation if not specified, or fallback on direct if needed
    if (animation_style < 0 || animation_style > 3) {
        animation_style = rand() % 4;
    }
    
    // If the angles are extreme, use direct movement for safety
    float angle_diff = fabsf(minute_angle - hour_angle);
    if (angle_diff > 180) {
        debug_msg("Large angle difference detected, using direct movement for safety");
        animation_style = -1;
    }
    
    debug_msg("Animated move to X%.1f Y%.1f (style %d)", minute_angle, hour_angle, animation_style);
    
    // Apply the selected animation with safety
    switch (animation_style) {
        case 0:
            spiral_movement(minute_angle, hour_angle, 2);
            break;
        case 1:
            pendulum_swing(minute_angle, hour_angle, 3);
            break;
        case 2:
            bounce_effect(minute_angle, hour_angle, 3);
            break;
        case 3:
            random_dance(minute_angle, hour_angle, 5);
            break;
        default:
            // Fallback to direct movement
            char cmd[64];
            memset(cmd, 0, sizeof(cmd));
            send_gcode("G90");
            vTaskDelay(50 / portTICK_PERIOD_MS);
            snprintf(cmd, sizeof(cmd)-1, "G1 X%.1f Y%.1f F1500", minute_angle, hour_angle);
            send_gcode(cmd);
            break;
    }
}

//===================================
// MAIN TASK IMPLEMENTATION
//===================================

// This function will be called from the main .ino file
void clockEngineTask(void* parameter) {
    // Wait longer for system to boot fully - increased from 5 to 20 seconds
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
        // Safety watchdog
        static uint32_t last_watchdog_kick = 0;
        uint32_t now = millis();  // DECLARE now ONLY ONCE
        
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
        }
        
        // HOMING: Use a direct approach without state machine
        if (system_ready && !initial_homing_done) {
            debug_msg("Starting simplified homing approach");
            
            // 1. Make absolutely sure we're unlocked
            debug_msg("Unlocking system");
            send_gcode("$X");
            vTaskDelay(2000 / portTICK_PERIOD_MS);  // Longer delay
            
            // 2. Try using the Serial interface directly
            debug_msg("Attempting homing via direct serial write");
            Serial.print("$H\n");  // Send directly to serial port
            vTaskDelay(5000 / portTICK_PERIOD_MS);  // Much longer delay
            
            // 3. Move to 0,0 regardless of homing result
            debug_msg("Moving to 12 o'clock position (0,0)");
            move_to_angles(0, 0);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            
            // 4. Mark homing as complete
            debug_msg("Marking homing as complete");
            initial_homing_done = true;
            
            // 5. Start sequence
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
    if (current_mode == MODE_CURRENT_TIME && system_ready && initial_homing_done) {
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
    if (current_mode == MODE_SPECIFIC_TIME && system_ready && initial_homing_done) {
        display_time(target_hour, target_minute);
    }
}

// Move to a specific angle
void clock_set_angles(float minute_angle, float hour_angle) {
    target_minute_angle = minute_angle;
    target_hour_angle = hour_angle;
    
    // If currently in direct angle mode, update the display
    if (current_mode == MODE_DIRECT_ANGLE && system_ready && initial_homing_done) {
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
                           const char* filename, uint32_t duration_ms,
                           int animation_style = -1) {
    if (step_index >= 0 && step_index < MAX_SEQUENCE_STEPS) {
        sequence[step_index].mode = static_cast<ClockMode>(mode);
        sequence[step_index].hour = hour;
        sequence[step_index].minute = minute;
        sequence[step_index].min_angle = min_angle;
        sequence[step_index].hour_angle = hour_angle;
        strncpy(sequence[step_index].filename, filename, sizeof(sequence[step_index].filename)-1);
        sequence[step_index].filename[sizeof(sequence[step_index].filename)-1] = '\0';
        sequence[step_index].duration_ms = duration_ms;
        sequence[step_index].animation_style = animation_style;
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

// Set up a custom M-code handler for clock control
bool gcode_unknown_command_execute(char *line) {
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
    
    return false;
}

// Implementation of MKS_GRBL_CMD_SEND function
void MKS_GRBL_CMD_SEND(const char* cmd) {
    // Just use our existing send_gcode function which is working
    char buffer[128];
    strncpy(buffer, cmd, sizeof(buffer)-1);
    buffer[sizeof(buffer)-1] = '\0';
    
    // Remove any newlines for gc_execute_line
    char* newline = strchr(buffer, '\n');
    if (newline) *newline = '\0';
    
    // Use our standard method
    send_gcode(buffer);
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
