#include "isr_timer.h"
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"
#include "firmware.h"

// Depending on the board, you can select SAMD21 Hardware Timer from TC3, TC4,
// TC5, TCC, TCC1 or TCC2 SAMD51 Hardware Timer only TC3 Init SAMD timer
// TIMER_TC3
SAMDTimer ITimer(TIMER_TC3); // using TIMER_TC3

// Init SAMD_ISR_Timer
// Each SAMD_ISR_Timer can service 16 different ISR-based timers
SAMD_ISR_Timer ISR_Timer;

void timerHandler() { ISR_Timer.run(); }

void timerSetup() {
  if (ITimer.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, timerHandler)) {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  } else {
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));
  }

  // You can use up to 16 timer for each ISR_Timer
  // unit is milliseconds
  ISR_Timer.setInterval(20L, PIDControl);
}
