/*
  Grbl_ESP32.ino - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

    2018 -	Bart Dring This file was modified for use on the ESP32
                    CPU. Do not use this with Grbl for atMega328P

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "src/Grbl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// External task function from clock_engine.cpp
extern void clockEngineTask(void* parameter);

void setup() {
    // Create a dedicated task for our clock engine
    xTaskCreatePinnedToCore(
        clockEngineTask,   // Function to implement the task
        "ClockEngine",     // Name of the task
        4096,              // Stack size in words
        NULL,              // Task input parameter
        1,                 // Priority of the task (lower number = lower priority)
        NULL,              // Task handle
        0                  // Core where the task should run (0 is good for our custom code)
    );
    
    // Initialize GRBL - will take over core 1
    grbl_init();
}

// Standard loop - GRBL takes over from here
void loop() {
    // GRBL will manage this core now
    _mc_task_init();
    
    while(1) {
        run_once();
        // This point is never reached!
    }
}
