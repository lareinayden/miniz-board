// source https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
// also check https://forum.arduino.cc/t/changing-arduino-zero-pwm-frequency/334231/3
// about CC and WO[n] mapping https://forum.arduino.cc/t/samd21-wo-and-cc-register-mapping/852449/2

// NOTE this is inconsistent with schematic
// left
int steer_rev_pin = 2;
// right
int steer_fwd_pin = 3;

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

uint32_t pwm_period = 2400 - 1;

void pwmSetup() {
  // Number to count to with PWM (TOP value). Frequency can be calculated by
  // freq = GCLK4_freq / (TCC0_prescaler * (1 + TOP_value))
  /// TOP = (clock freq)/(desired freq)/prescaler - 1
  // TOP = 2400-1, freq = 20k
  // TOP = 48-1, freq = 1M
  // resolution = log(TOP)/log(2)

  // Because we are using TCC0, limit period to 24 bits
  pwm_period = ( pwm_period < 0x00ffffff ) ? pwm_period : 0x00ffffff;

  // Enable and configure generic clock generator 4
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Improve duty cycle
                      GCLK_GENCTRL_GENEN |        // Enable generic clock gen
                      GCLK_GENCTRL_SRC_DFLL48M |  // Select 48MHz as source
                      GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Set clock divider of 1 to generic clock generator 4
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |         // Divide 48 MHz by 1
                      GCLK_GENDIV_ID(4);           // Apply to GCLK4 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Enable GCLK4 and connect it to TCC0 and TCC1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                      GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC0/1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Divide counter by 1 giving 48 MHz (20.83 ns) on each TCC0 tick
  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

  // Use "Normal PWM" (single-slope PWM): count up to PER, match on CC[n]
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;         // Select NPWM as waveform

  //REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
  //                TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Set the period (the number to count to (TOP) before resetting timer)
  TCC0->PER.reg = pwm_period;
  while (TCC0->SYNCBUSY.bit.PER);

  // n for CC[n] is determined by n = x % 4 where x is from WO[x]
  //dutyCycle set up
  //TCC0->CC[2].reg = period / divider; 
  //TCC0->CC[2].reg = int(duty_cycle * (period-1));
  //while (TCC0->SYNCBUSY.bit.CC2);
}
void pwmSet(int pinNumber, float duty_cycle){
  switch(pinNumber) {
    // PORT: g_APinDescription[pinNumber].ulPort
    // PIN: g_APinDescription[pinNumber].ulPin
    case 3:
      // PB11 TCC0/WO[5] function F
      TCC0->CC[5 % 4].reg = int(duty_cycle * (pwm_period-1));
      PORT->Group[PORTB].DIRSET.reg = PORT_PB11;      // Set pin as output
      PORT->Group[PORTB].OUTCLR.reg = PORT_PB11;      // Set pin to low
      // Enable the port multiplexer for PB11
      PORT->Group[PORTB].PINCFG[11].bit.PMUXEN = 1;
      // identical to above
      //PORT->Group[PORTB].PINCFG[11].reg |= PORT_PINCFG_PMUXEN;

      // Connect TCC0 timer to PB11. Function F is TCC0/WO[5] for PB11.
      // Odd pin num (pin_no = 2*n + 1): use PMUXO
      // Even pin num (pin_no = 2*n): use PMUXE
      // PMUX[x]: here x = pin_no / 2
      // for more detail check link in top of file
      PORT->Group[PORTB].PMUX[5].reg |= PORT_PMUX_PMUXO_F;
      break;

    case 2:
      // PB10 TCC0/WO[4] function F
      TCC0->CC[4 % 4].reg = int(duty_cycle * (pwm_period-1));
      //PORT->Group[PORTB].DIRSET.reg = PORT_PB10;      // Set pin as output
      //PORT->Group[PORTB].OUTCLR.reg = PORT_PB10;      // Set pin to low
      PORT->Group[PORTB].PINCFG[10].bit.PMUXEN = 1;
      PORT->Group[PORTB].PMUX[5].reg |= PORT_PMUX_PMUXE_F;
      break;

    case 5:
      // PA05 TCC0/WO[1] function E
      TCC0->CC[1].reg = int(duty_cycle * (pwm_period-1));
      //PORT->Group[PORTA].DIRSET.reg = PORT_PA05;      // Set pin as output
      //PORT->Group[PORTA].OUTCLR.reg = PORT_PA05;      // Set pin to low
      PORT->Group[PORTA].PINCFG[5].bit.PMUXEN = 1;
      PORT->Group[PORTA].PMUX[2].reg |= PORT_PMUX_PMUXO_E;
      break;

    case 6:
      // PA04 TCC0/WO[0] function E
      TCC0->CC[0].reg = int(duty_cycle * (pwm_period-1));
      //PORT->Group[PORTA].DIRSET.reg = PORT_PA04;      // Set pin as output
      //PORT->Group[PORTA].OUTCLR.reg = PORT_PA04;      // Set pin to low
      PORT->Group[PORTA].PINCFG[4].bit.PMUXEN = 1;
      PORT->Group[PORTA].PMUX[2].reg |= PORT_PMUX_PMUXE_E;
      break;

    case 16:
      // D11 - this works
      PORT->Group[PORTA].DIRSET.reg = PORT_PA16;      // Set pin as output
      PORT->Group[PORTA].OUTCLR.reg = PORT_PA16;      // Set pin to low
    
      // Enable the port multiplexer for PA18
      PORT->Group[PORTA].PINCFG[16].reg |= PORT_PINCFG_PMUXEN;
    
      // Odd pin num (2*n + 1): use PMUXO
      // Even pin num (2*n): use PMUXE
      PORT->Group[PORTA].PMUX[8].reg = PORT_PMUX_PMUXE_F;

      TCC0->CC[2].reg = int(duty_cycle * (pwm_period-1));
      while (TCC0->SYNCBUSY.bit.CC2);

      break;


    default:
      Serial.println("Unsupported pin");
  }

  // Enable output (start PWM)
  TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}
void setup() {
  pwmSetup();
  // by themselves, they all work, if multiple pwmSet are enabled, only the last one works
  //pwmSet(2,0.1); -> this works
  //pwmSet(3,0.2);
  pwmSet(6,0.6); // -> this works
  pwmSet(5,0.5); // -> this works
  //pwmSet(16,0.5); -> this works
  // The CCBx register value corresponds to the pulsewidth in microseconds (us)

  //TCC0->CC[0].reg = int(0.1 * (pwm_period-1));
  //PORT->Group[PORTA].PINCFG[4].bit.PMUXEN = 1;
  //PORT->Group[PORTA].PMUX[4 >> 1].reg |= PORT_PMUX_PMUXE_E;

  //TCC0->CC[1].reg = int(0.2 * (pwm_period-1));
  //PORT->Group[PORTA].PINCFG[5].bit.PMUXEN = 1;
  //// D5 - PA05, D6 = PA04
  //PORT->Group[PORTA].PMUX[5 >> 1].reg |= PORT_PMUX_PMUXO_E;



  //PORT->Group[PORTA].PINCFG[20].bit.PMUXEN = 1;
  //PORT->Group[PORTA].PINCFG[21].bit.PMUXEN = 1;

  // D9 - PA20, D10 - PA21
  //PORT->Group[PORTA].PMUX[21 >> 1].reg |= PORT_PMUX_PMUXO_F;
  //PORT->Group[PORTA].PMUX[20 >> 1].reg |= PORT_PMUX_PMUXE_F;

  // this works
  //while(TCC0->SYNCBUSY.bit.CCB0);
  //while(TCC0->SYNCBUSY.bit.CCB1);
  //TCC0->CC[2].reg = int(0.6 * (pwm_period-1));
  //while(TCC0->SYNCBUSY.bit.CCB2);
  //TCC0->CC[3].reg = int(0.7 * (pwm_period-1));
  //while(TCC0->SYNCBUSY.bit.CCB3);

  // Enable output (start PWM)
  //TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  //while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

}

void loop() {
}
