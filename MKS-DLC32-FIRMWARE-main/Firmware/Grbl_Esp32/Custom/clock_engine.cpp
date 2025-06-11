/*
  clock_engine.cpp  —  ultra-safe skeleton
  ✓ No FreeRTOS timers
  ✓ All parser calls happen in GRBL main context (user_routine, macro)
  ✓ Clear Serial prints so you can see each step
*/

#include "../src/Grbl.h"     // GRBL core types
#include "../src/GCode.h"    // gc_execute_line prototype
#include <Arduino.h>         // Serial
#include <cstring>           // strncpy

// ---------- tiny debug helper ----------
static void dbg(const char* s) { Serial.println(s); }

// ---------- flags ----------
static bool need_home        = true;  // request homing once
static bool zero_sent = false;   // tracks if G0 X0 Y0 has been issued

// ---------- safe G-code sender ----------
static void send_gcode(const char *line)
{
  static char buf[96];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1] = '\0';
  /* execute in GRBL main task (user_routine / macro) */
  gc_execute_line(buf, 0);   // client 0 = internal USB
}

// ---------- GRBL hook: runs once at boot ----------
void machine_init(void)
{
  dbg("machine_init() – waiting for GRBL idle…");
  WebUI::inputBuffer.push("$H\r");  // home machine
  /* nothing else – everything happens in user_routine() */
}

// ---------- GRBL hook: called very often in main loop ----------
void user_routine(void)
{
  // /* 1) if we still need to home and machine is idle → send $H */
  // if (need_home && sys.state == State::Idle)
  // {
  //   dbg("queue $H");
  //   send_gcode("$H");
  //   need_home = false;
  //   /* when homing starts sys.state will go to Homing */
  // }

  // /* 2) once homing completed and idle, send zero move once */
  // if (!zero_sent && !need_home && sys.state == State::Idle)
  // {
  //   dbg("queue zero move");
  //   send_gcode("G90");
  //   send_gcode("G0 X0 Y0 F2000");
  //   zero_sent = true;
  // }

    
  //   static uint32_t last = 0;
  //   if (millis() - last > 1000) {   // every second
  //       dbg("user_routine alive");
  //       last = millis();
  //   }
}
  



// ---------- Macro buttons from Web UI ----------
void user_defined_macro(uint8_t idx)
{
  char m[24];
  sprintf(m,"macro %u",idx); dbg(m);

  switch (idx)
  {
    case 1:  send_gcode("$Play=/spin420.nc");          break;          // play file
    case 2:  send_gcode("G0 X120 Y130 F12000");        break;          // snap to 4:20
    case 3:  need_home = true; zero_sent = false;      break;          // re-home
    default: break;
  }
}

/* --------------- OPTIONAL: M900 HH:MM --------------- */
bool gcode_unknown_command_execute(char *line)
{
  if (strncmp(line,"M900",4)==0)
  {
    dbg("M900 set time");
    uint8_t hh = atoi(line+5);
    uint8_t mm = atoi(line+8);
    float minDeg = mm*6.0f;                           // minutes
    float hrDeg  = (hh%12)*30.0f + mm*0.5f;           // hours
    char buf[64];
    sprintf(buf,"G0 X%.1f Y%.1f F2000",minDeg,hrDeg);
    send_gcode("G90");
    send_gcode(buf);
    return true;
  }
  return false;
}