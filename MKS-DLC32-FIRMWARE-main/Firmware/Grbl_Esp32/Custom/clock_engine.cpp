/*
  clock_engine.cpp
  Extremely defensive version – no out-of-context parser calls
*/

#include "../src/grbl.h"          // GRBL core
#include "../src/Realtime.h"      // realtime command helpers
#include <Arduino.h>
#include <cstring>

/* ---------------- debug helper ---------------- */
static void dbg(const char *s){ Serial.println(s); }

/* ---------------- flag section ---------------- */
static bool need_home       = true;   // we’ll home once in main loop
static bool need_zero_after = false;  // move to 12:00 after homing

/* ---------------- helpers --------------------- */
static void queue_home()   { enqueueRealtimeCommand(CMD_HOMING_CYCLE_START); }
static void queue_zero()   { enqueueRealtimeCommand(CMD_FEED_HOLD); /* flush */ 
                             gc_execute_line(const_cast<char*>("G90"), 0);
                             gc_execute_line(const_cast<char*>("G0 X0 Y0 F2000"), 0); }
                              // ← executed only from main loop, safe

/* ------------- GRBL hook: machine_init() ------ */
void machine_init()
{
  dbg("machine_init()");
  /* nothing else – wait for GRBL main loop */
}

/* ------------- GRBL hook: called very often ---- */
void user_routine()
{
  /* this runs in GRBL main task – it’s safe to parse G-code here */
  if (need_home && sys.state == State::Idle) {
      dbg("queuing home");
      queue_home();
      need_home = false;
      need_zero_after = true;
  }

  if (need_zero_after && sys.state == State::Idle && !sys.homing_flag) {
      dbg("queuing zero");
      queue_zero();
      need_zero_after = false;
  }
}

/* ------------- user macro buttons -------------- */
void user_defined_macro(uint8_t id)
{
  // triggered from Web UI (runs in main task)
  switch(id){
    case 0: need_home = true; break;                 // “Home” button
    case 1: gc_execute_line(const_cast<char*>("$Play=/spin420.nc"), 0); break;
    case 2: gc_execute_line(const_cast<char*>("G0 X120 Y130 F12000"), 0); break; // 4:20
    // add more cases here
  }
}