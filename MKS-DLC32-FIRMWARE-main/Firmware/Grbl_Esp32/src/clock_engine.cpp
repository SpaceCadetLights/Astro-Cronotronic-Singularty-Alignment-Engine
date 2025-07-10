/*
  clock_engine.cpp - Modular implementation of clock controller
  For MKS-DLC32 controller with GRBL-ESP32
*/

#include "Grbl.h"
#include "GCode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

// Function to send G-code commands
static bool send_gcode(const char *line) {
    static char buf[96];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    return (gc_execute_line(buf, CLIENT_SERIAL) == Error::Ok);
}

// This function will be called from the main .ino file
void clockEngineTask(void* parameter) {
    // Wait for system to boot fully
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Clock engine task started!");
    
    // Initialize movement variables
    static int movement_stage = 0;
    static bool system_ready = false;
    
    // Task loop
    while(true) {
        // Current time
        uint32_t now = millis();
        
        // Report system status
        static uint32_t last_status = 0;
        if (now - last_status >= 1000) {  // Every second
            // Print machine state
            const char* state_str = "UNKNOWN";
            switch(sys.state) {
                case State::Idle: state_str = "IDLE"; break;
                case State::Alarm: state_str = "ALARM"; break;
                case State::Cycle: state_str = "CYCLE"; break;
                case State::Hold: state_str = "HOLD"; break;
                default: state_str = "OTHER"; break;
            }
            
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Machine state: %s (%d)", state_str, (int)sys.state);
            last_status = now;
            
            // Check if we're in Idle state and haven't initialized yet
            if (sys.state == State::Idle && !system_ready) {
                // Perform one-time initialization when system becomes ready
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Initializing system...");
                
                // 1. Clear alarms
                send_gcode("$X");
                vTaskDelay(100 / portTICK_PERIOD_MS);
                
                // 2. Enable steppers
                send_gcode("$1=255");
                vTaskDelay(100 / portTICK_PERIOD_MS);
                
                // 3. Set units and mode
                send_gcode("G21"); // mm
                
                system_ready = true;
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "System initialized!");
            }
        }
        
        // Try motor movement every 3 seconds
        static uint32_t last_move = 0;
        if (now - last_move >= 3000 && system_ready) {
            // Get current movement stage
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Movement stage: %d", movement_stage);
            
            // Always use relative positioning for our test
            send_gcode("G91");
            
            // Different movement based on stage
            switch(movement_stage) {
                case 0:
                    // Move X right +10mm
                    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Moving X RIGHT +10mm");
                    send_gcode("G0 X10 F1000");
                    break;
                    
                case 1:
                    // Move X left -10mm
                    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Moving X LEFT -10mm");
                    send_gcode("G0 X-10 F1000");
                    break;
                    
                case 2:
                    // Move Y forward +10mm
                    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Moving Y FORWARD +10mm");
                    send_gcode("G0 Y10 F1000");
                    break;
                    
                case 3:
                    // Move Y backward -10mm
                    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Error, "Moving Y BACKWARD -10mm");
                    send_gcode("G0 Y-10 F1000");
                    break;
            }
            
            // Return to absolute positioning
            send_gcode("G90");
            
            // Update for next movement
            movement_stage = (movement_stage + 1) % 4;
            last_move = now;
        }
        
        // Short delay to prevent task starvation
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}