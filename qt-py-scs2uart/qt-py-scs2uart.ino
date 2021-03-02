void setupADC();

void setup() {

  // SCSRX (from level shifter):
  pinMode(7, INPUT);

  // SCSRXUART (modified signal)
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  setupADC();

  // turn off the NeoPixel for low power usage
  digitalWrite(12, LOW);
}

// Atmel AT11480: Analog Comparator Application Examples (18 pages)
// http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42473-Analog-Comparator-Application-Examples_UserGuide_AT11480.pdf
// https://github.com/avrxml/asf/blob/master/sam0/applications/ac_examples/main.c

void setupADC() {
  PM->APBCMASK.reg |= PM_APBCMASK_AC;

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_AC_ANA;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_AC_DIG;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Analog Comparator unterstützt als analog inputs nur:
  // - PA04: A2 (linke seite)
  // - PA05: A3 (linke seite)
  // - PA06: TX_A6 (linke seite)
  // - PA07: RX_A7 (rechte seite)

  // PA07, i.e. RX_A7, analog input
  {
    // Disable digital pin circuitry
    // gpio_set_pin_direction(PA05, GPIO_DIRECTION_OFF);
    PORT->Group[PORTA].DIRCLR.reg = (1 << 7); // turn off
    PORT->Group[PORTA].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG | (1 << 7);

    // gpio_set_pin_function(PA05, PINMUX_PA05B_AC_AIN1);
    // _gpio_set_pin_function(PA05, PINMUX_PA05B_AC_AIN1);
    PORT->Group[PORTA].PINCFG[7].bit.PMUXEN = 1;
    PORT->Group[PORTA].PMUX[7 >> 1].reg |= PORT_PMUX_PMUXO_B;
  }

  // Analog Comparator
  // ///////////////////////////////////////////////////////////////////////

  AC->COMPCTRL[0].reg =
      AC_COMPCTRL_HYST |        // Enable input hysteresis
      AC_COMPCTRL_OUT_OFF |     // Enable comparator output in asynchronous mode
      AC_COMPCTRL_MUXPOS_PIN3 | // Set the positive input multiplexer to AIN[3]
      AC_COMPCTRL_MUXNEG_VSCALE | // Set the negative input multiplexer to the
                                  // voltage scaler
      AC_COMPCTRL_SPEED_HIGH |    // Place the comparator into high speed mode
      AC_COMPCTRL_INTSEL_TOGGLE;
  while (AC->STATUSB.bit.SYNCBUSY)
    ; // Wait for synchronization

  AC->SCALER[0].reg = AC_SCALER_VALUE(
      39); // Set the negative input voltage scale to 2V
           // Vscale = Vdd * (VALUE + 1) / 64 = 3.3V * (39 + 1) / 64 = 2V

  AC->CTRLA.bit.ENABLE = 1; // Enable the analog comparator peripheral
  while (AC->STATUSB.bit.SYNCBUSY)
    ; // Wait for synchronization

  AC->COMPCTRL[0].bit.ENABLE = 1; // Enable the analog comparator channel 1
  while (AC->STATUSB.bit.SYNCBUSY)
    ; // Wait for synchronization

  // Enable interrupts
  AC->INTENSET.bit.COMP0 = 1;

  NVIC_SetPriority(AC_IRQn, 0); // highest priority
  NVIC_EnableIRQ(AC_IRQn);

  //	Serial.println("running");
}

// code example for SAMD21 analog comparator:
// https://forum.arduino.cc/index.php?topic=649060.msg4377953#msg4377953

volatile int state = -1; // handled

void AC_Handler(void) {
  // Acknowledge the interrupt:
  AC->INTFLAG.bit.COMP0 = 1;

  state = !(AC->STATUSA.bit.STATE0);
}

void loop() {
  // TODO: go into power save but allow for all interrupts
  if (state == -1) {
    return; // handled
  }
  if (state == HIGH) {
    delayMicroseconds(20);
    digitalWrite(8, HIGH);
    state = -1;
    return;
  }
  if (state == LOW) {
    digitalWrite(8, LOW);
    state = -1;
    return;
  }
}
