/*
  clock_engine.cpp
  Custom control logic for the Astro‑Chronotronic Singularity Alignment Engine
  Space Cadets Designs – June 2025
*/

#include "../src/grbl.h"      // adjust path so compiler finds core header
#include <Arduino.h>   // _serprintf helpers
#include <cstring>          // for strncpy


#include "../src/GCode.h"      // exposes gc_execute_line()

#include "driver/timer.h"



static void dbg(const char *msg) { Serial.println(msg); }

static void send_gcode(const char *line) {
  static char buf[96];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1] = '\0';
  gc_execute_line(buf, 0);                // 0 = internal serial client
}

static TimerHandle_t hClockTimer = nullptr;   // global scheduler timer

/*********************************************************************
* Helpers – angles vs time
*********************************************************************/

static inline float hour_to_angle(uint8_t hh, uint8_t mm) {
  return (hh % 12) * 30.0f + (mm * 0.5f);
}

static inline float minute_to_angle(uint8_t mm, uint8_t ss) {
  return mm * 6.0f + ss * 0.1f;
}

/*********************************************************************
*  Send a single absolute move
*********************************************************************/
static void send_absolute_move(float minDeg, float hrDeg, float feed) {
  char buf[64];
  sprintf(buf, "G90\nG0 X%.3f Y%.3f F%.1f", minDeg, hrDeg, feed);
  send_gcode(buf);
}

/*********************************************************************
*  Very simple software clock (replace with DS3231 later)
*********************************************************************/
static uint8_t cur_h = 12, cur_m = 0, cur_s = 0;

/*********************************************************************
* Quick helpers for animations
*********************************************************************/
static void spin_to(uint16_t minDeg, uint16_t hrDeg) {
  char buf[48];
  sprintf(buf, "G90\nG0 X%u Y%u F4000", minDeg, hrDeg);
  dbg("spin_to");
  send_gcode(buf);
}

static void play_gcode(const char *fname) {
  if (sys.state == State::Idle) {
    dbg("play_gcode");
    char buf[64];
    sprintf(buf, "$Play=%s", fname);
    send_gcode(buf);
  }
}

static void macro_real_time() {
  dbg("macro_real_time");
  send_absolute_move(minute_to_angle(cur_m, cur_s), hour_to_angle(cur_h, cur_m), 2000);
}

/*********************************************************************
*  PLAYLIST SECTION START
*********************************************************************/
struct Cue { uint32_t ms; void (*fn)(); };

static Cue playlist[] = {
  {    0, [](){ play_gcode("/spin420.nc"); } }, // cue 0
  { 10000, macro_real_time },                   // cue 1 – show time
  { 18000, [](){ spin_to(0,330); } },           // cue 2 – 11:11
  { 28000, nullptr }                            // terminator
};

static uint32_t cue_start_ms = 0;
static size_t   cue_idx      = 0;

static void vClockTickCb(TimerHandle_t) {
  uint32_t t = millis();
  if (playlist[cue_idx].fn && t - cue_start_ms >= playlist[cue_idx].ms && sys.state == State::Idle) {
      char m[32]; sprintf(m, "cue %zu fired", cue_idx);
      dbg(m);
      playlist[cue_idx].fn();
      cue_idx++;
      if (!playlist[cue_idx].fn) {      // end reached
          cue_idx = 0;
          cue_start_ms = t;
      }
  }
}
/*********************************************************************
*  PLAYLIST SECTION END
*********************************************************************/


/*********************************************************************
* Boot one-shot timer to run auto‑homing after GRBL startup
*********************************************************************/
static TimerHandle_t hBootTimer = nullptr;

static void vBootCb(TimerHandle_t) {
  dbg("auto‑homing…");
  send_gcode("$H");
  vTaskDelay(pdMS_TO_TICKS(500));
  send_gcode("G90");
  send_gcode("G0 X0 Y0 F2000");
  dbg("zero move queued");

  // start playlist timer now that homing queued
  hClockTimer = xTimerCreate("ClockTick", pdMS_TO_TICKS(200), pdTRUE, nullptr, vClockTickCb);
  if (hClockTimer) xTimerStart(hClockTimer, 0);
}

/*********************************************************************
* Weak‑function overrides
*********************************************************************/
void machine_init() {
  dbg("machine_init start");

  /* schedule auto‑home 1 s after GRBL is ready */
  hBootTimer = xTimerCreate("Boot", pdMS_TO_TICKS(1000), pdFALSE, nullptr, vBootCb);
  if (hBootTimer) xTimerStart(hBootTimer, 0);
}


void user_defined_macro(uint8_t idx) {
  { char m[32]; sprintf(m, "macro %u", idx); dbg(m); }
  switch (idx) {
    case 0: macro_real_time();          break;
    case 1: spin_to(222,  60);          break;
    case 2: spin_to(420, 120);          break;
    case 3: spin_to(   0, 330);         break;
    case 4: play_gcode("/wild.nc");     break;
    default: break;
  }
}

/*  M900 HH:MM → set clock */
bool gcode_unknown_command_execute(char *line) {
  if (strncmp(line, "M900", 4) == 0) {
    dbg("M900 set time");
    uint8_t hh = atoi(line + 5);
    uint8_t mm = atoi(line + 8);
    cur_h = hh; cur_m = mm; cur_s = 0;
    send_absolute_move(minute_to_angle(cur_m, 0), hour_to_angle(cur_h, cur_m), 1500);
    return true;
  }
  return false;
}
