// pin_names.h
// Names for the GPIO pins.
// Needs to be included into the main program.
// Peter J. 2026-11-03

#ifndef PIN_NAMES_H
#define PIN_NAMES_H

// A. For interaction with the PIC18F26Q71 COMMS-4-MCU.
// UART0_TX on GP0 (default)
// UART0_RX in GP1 (default)
const uint READY_PIN = 20;
const uint EVENT_PIN = 22;

// B. For interaction with all of the ADS131M04 ADC chips.
const uint MCLK_PIN = 2;
const uint SYNC_PIN = 3;

#undef SWAP03
#undef SWAP12

// C. For spi0 interaction with ADC-0 or ADC-3 chips.
#ifndef SWAP03
// Normally, we connect the ADC-0 chip.
const uint MISO_0_PIN = 4;
const uint CS_0_PIN = 5;
const uint SCK_0_PIN = 6;
const uint MOSI_0_PIN = 7;
const uint DRDY_0_PIN = 28;
#else
// However, we may connect the ADC-3 chip.
const uint MISO_0_PIN = 16;
const uint CS_0_PIN = 17;
const uint SCK_0_PIN = 18;
const uint MOSI_0_PIN = 19;
const uint DRDY_0_PIN = 21;
#endif

// D. For spi1 interaction with ADC-1 or ADC-2 chips.
#ifndef SWAP12
// Normally, we connect the ADC-1 chip.
const uint MISO_1_PIN = 8;
const uint CS_1_PIN = 9;
const uint SCK_1_PIN = 10;
const uint MOSI_1_PIN = 11;
const uint DRDY_1_PIN = 27;
#else
// However, we may connect the ADC-2 chip.
const uint MISO_1_PIN = 12;
const uint CS_1_PIN = 13;
const uint SCK_1_PIN = 14;
const uint MOSI_1_PIN = 15;
const uint DRDY_1_PIN = 26;
#endif

// E. For pio-spi2 interaction with ADC-2 or ADC-1 chips.
#ifndef SWAP12
// Normally, we connect the ADC-2 chip.
const uint MISO_2_PIN = 12;
const uint CS_2_PIN = 13;
const uint SCK_2_PIN = 14;
const uint MOSI_2_PIN = 15;
const uint DRDY_2_PIN = 26;
#else
// However, we may connect the ADC-1 chip.
const uint MISO_2_PIN = 8;
const uint CS_2_PIN = 9;
const uint SCK_2_PIN = 10;
const uint MOSI_2_PIN = 11;
const uint DRDY_2_PIN = 27;
#endif

// F. For pio-spi3 interaction with ADC-3 or ADC-0 chips.
#ifndef SWAP03
// Normally, we connect the ADC-3 chip.
const uint MISO_3_PIN = 16;
const uint CS_3_PIN = 17;
const uint SCK_3_PIN = 18;
const uint MOSI_3_PIN = 19;
const uint DRDY_3_PIN = 21;
#else
// However, we may connect the ADC-0 chip.
const uint MISO_3_PIN = 4;
const uint CS_3_PIN = 5;
const uint SCK_3_PIN = 6;
const uint MOSI_3_PIN = 7;
const uint DRDY_3_PIN = 28;
#endif

#endif
