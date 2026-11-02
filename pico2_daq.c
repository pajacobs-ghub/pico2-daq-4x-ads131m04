// pico2_daq.c
//
// RP2350 Pico2 board as the DAQ-MCU, getting data for 16 analog channels
// via 4x ADS131M04 24-bit ADC chips.
//
// PJ 2026-10-19: Adapted from the code for Jeremy's small-board prototype
//    that was built with a single ADS131M04 converter providing 4 channels,
//    plus bits of code from the 8xMPC3301 board.
//    2026-10-22: Change the blocking call to drive the SPI0 module with
//    an arrangement that uses DMA channels to send data to and
//    collect data from the SPI0 module.
//    2026-11-01: Add SPI-2 and SPI-3 implementations via PIO.

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "spi.pio.h"
#include "hardware/structs/pio.h"

#define VERSION_STR "v0.7 Pico2 as DAQ-MCU 2026-11-02"
const uint n_adc_chips = 4;

// Names for the GPIO pins.
// A. For interaction with the PIC18F26Q71 COMMS-4-MCU.
// UART0_TX on GP0 (default)
// UART0_RX in GP1 (default)
const uint READY_PIN = 20;
const uint EVENT_PIN = 22;
// B. For interaction with all of the ADS131M04 ADC chips.
const uint MCLK_PIN = 2;
const uint SYNC_PIN = 3;
// C. For SPI0 interaction with ADS1131M04-0 ADC chip.
#undef SWAP03
#ifndef SWAP03
const uint MISO_0_PIN = 4;
const uint CS_0_PIN = 5;
const uint SCK_0_PIN = 6;
const uint MOSI_0_PIN = 7;
const uint DRDY_0_PIN = 28;
#else
const uint MISO_0_PIN = 16;
const uint CS_0_PIN = 17;
const uint SCK_0_PIN = 18;
const uint MOSI_0_PIN = 19;
const uint DRDY_0_PIN = 21;
#endif
// D. For SPI1 interaction with ADS1131M04-1 ADC chip.
const uint MISO_1_PIN = 8;
const uint CS_1_PIN = 9;
const uint SCK_1_PIN = 10;
const uint MOSI_1_PIN = 11;
const uint DRDY_1_PIN = 27;
// E. For SPI1/PIOx interaction with ADS1131M04-2 ADC chip.
const uint MISO_2_PIN = 12;
const uint CS_2_PIN = 13;
const uint SCK_2_PIN = 14;
const uint MOSI_2_PIN = 15;
const uint DRDY_2_PIN = 26;
// F. For SPI0/PIOx interaction with ADS1131M04-3 ADC chip.
#ifndef SWAP03
const uint MISO_3_PIN = 16;
const uint CS_3_PIN = 17;
const uint SCK_3_PIN = 18;
const uint MOSI_3_PIN = 19;
const uint DRDY_3_PIN = 21;
#else
const uint MISO_3_PIN = 4;
const uint CS_3_PIN = 5;
const uint SCK_3_PIN = 6;
const uint MOSI_3_PIN = 7;
const uint DRDY_3_PIN = 28;
#endif

static inline void assert_ready()
{
    gpio_put(READY_PIN, 1);
}

static inline void assert_not_ready()
{
    gpio_put(READY_PIN, 0);
}

static inline void assert_event()
{
	gpio_put(EVENT_PIN, 1);
}

static inline void release_event()
{
	gpio_put(EVENT_PIN, 0);
}

static inline uint8_t read_system_event_pin()
{
	return (uint8_t) gpio_get(EVENT_PIN);
}

const uint LED_PIN = PICO_DEFAULT_LED_PIN; // GPIO25
uint8_t override_led = 0;

static inline void sampling_LED_ON()
{
    gpio_put(LED_PIN, 1);
}

static inline void sampling_LED_OFF()
{
    gpio_put(LED_PIN, 0);
}

// A place for the current sample set.
#define MAX_N_CHAN 16
static int32_t sample_data[MAX_N_CHAN];

// A place to collect the sample sets as they come in from the ADC chips.
#define N_FULLWORDS 0x00010000
#define MAX_N_SAMPLES (N_FULLWORDS/4)
int32_t data[N_FULLWORDS];
uint32_t next_fullword_index_in_data;
uint8_t fullword_index_has_wrapped_around;

// We are going to communicate with the ADS131M04 with its default word size
// of 24 bits and a full frames of 6 words but we will look at this data as
// 18 bytes when interacting with the Pico2's SPI modules.
# define N_BYTES_IN_FRAME 18
static uint8_t incoming_bytes_adc[4][N_BYTES_IN_FRAME];
// All 4 ADC chips receive the same messages.
static uint8_t outgoing_bytes_adc[N_BYTES_IN_FRAME];

static int sampling_error_flag;

// Options for over-sampling ratio in CLOCK register.
const uint8_t OSR_128 = 0;
const uint8_t OSR_256 = 1;
const uint8_t OSR_512 = 2;
const uint8_t OSR_1024 = 3;
const uint8_t OSR_2048 = 4;
const uint8_t OSR_4096 = 5;
const uint8_t OSR_8192 = 6;
const uint8_t OSR_16256 = 7;

static const uint32_t OSR_value[8] = {128, 256, 512, 1024, 2048, 4096, 8192, 16256};

// Options for Power bits in CLOCK register.
const uint8_t VERY_LOW_POWER = 0;
const uint8_t LOW_POWER = 1;
const uint8_t HIGH_RESOLUTION = 2;

const uint8_t TRIGGER_IMMEDIATE = 0;
const uint8_t TRIGGER_INTERNAL = 1;
const uint8_t TRIGGER_EXTERNAL = 2;

const uint8_t SLOPE_NEG = 0;
const uint8_t SLOPE_POS = 1;

// Parameters controlling the device are stored in virtual config registers.
#define NUMREG 9
int32_t vregister[NUMREG];
const uint MCLK_KHZ = 0;
const uint OSR_SELECT = 1;
const uint PWR_SELECT = 2;
const uint NCHANNELS = 3;
const uint NSAMPLES = 4;
const uint TRIG_MODE = 5;
const uint TRIG_CHAN = 6;
const uint TRIG_LEVEL = 7;
const uint TRIG_SLOPE = 8;

void set_registers_to_original_values()
{
	// Master clock frequency selected to give low-power operation.
    vregister[MCLK_KHZ] = 8192/2; // Master clock frequency f_CLKIN for ADS131M04, in kHz
    vregister[OSR_SELECT] = OSR_512;  // Selection bits for over-sampling ratio.
    vregister[PWR_SELECT] = LOW_POWER; // power select bits need to match MCLK setting.
    // See Table 8-2 in the ADS131M04 data sheet for acceptable combinations
    // and the resultant data output rates.
    // With LOW-POWER mode, f_CLKIN=4.096 MHz, f_MOD=2.048 MHz, OSR=512
    // f_DATA = f_MOD / OSR = 4 kSps
    vregister[NCHANNELS] = 16; // number of active analog channels
    // FIX-ME PJ 2026-10-21 Since we are porting this code over from the prototype board
    // that had only one ADS131M04 ADC chip, there are a few places that 4 channels and
    // a single ADC chip has been assumed.
    vregister[NSAMPLES] = 128; // number of samples in record after trigger event
    vregister[TRIG_MODE] = TRIGGER_IMMEDIATE;
    vregister[TRIG_CHAN] = 0; // trigger channel for internal trigger
    vregister[TRIG_LEVEL] = 10000; // trigger level as a signed integer
    vregister[TRIG_SLOPE] = SLOPE_POS;
}

static inline uint32_t byte_size_of_sample_set(void)
{
    uint8_t N_CHAN = (uint8_t)vregister[NCHANNELS];
    return 4 * N_CHAN;
}

static inline uint32_t max_n_samples(void)
{
    uint8_t N_CHAN = (uint8_t)vregister[NCHANNELS];
    return N_FULLWORDS / N_CHAN;
}

static inline uint32_t oldest_fullword_index_in_data()
{
    return (fullword_index_has_wrapped_around) ? next_fullword_index_in_data : 0;
}

static inline int assert_adc_synch(uint32_t duration_us, uint64_t timeout)
{
    // We assume that the master clock to the ADS131M04 is already running.
    // To meet the timing requirement in Figure 6-2 of the ADS131M04 data sheet,
    // wait for a trailing edge of CLKIN and then pulse the SYNCH pin low.
    // This waiting will timeout if CLKIN is not running.
    while (!gpio_get(MCLK_PIN)) {
        if (time_reached(timeout)) return 1;
    }
    while (gpio_get(MCLK_PIN)) {
        if (time_reached(timeout)) return 1;
    }
    gpio_put(SYNC_PIN, 0);
    busy_wait_us(duration_us);
    gpio_put(SYNC_PIN, 1);
    return 0;
}

static inline int wait_for_data_ready_adc0(uint64_t timeout)
// Returns 0 when the DRDY for ADC-0 pin goes low but will return 1 on time-out.
// Since we run all of the ADC chips synchronously, with the same commands,
// it should be adequate to monitor just the DRDY pin for ADC-0.
{
    while (gpio_get(DRDY_0_PIN)) {
        if (time_reached(timeout)) return 1;
    }
    return 0;
}

static inline void prepare_adc_command(uint16_t cmd)
// Set up outgoing buffer data for a simple command at next full-frame exchange.
{
    outgoing_bytes_adc[0] = (uint8_t)((cmd & 0xff00) >> 8);
    outgoing_bytes_adc[1] = (uint8_t)(cmd & 0x00ff);
    for (uint i = 2; i < N_BYTES_IN_FRAME; ++i) {
        outgoing_bytes_adc[i] = (uint8_t)0;
    }
}

static inline void prepare_to_write_adc_register(uint8_t reg_addr, uint16_t reg_value)
// Set up outgoing buffer data to write a 16-bit value to an ADS131M04 register
// at the next full-frame exchange.
// The WREG command format is: 011a aaaa annn nnnn
// where a is the register address and
//       n is the number of registers to write minus 1.
// For a single register write, n=0.
{
    uint16_t cmd = 0x6000 | ((reg_addr & 0x3f) << 7); // WREG command with address
    outgoing_bytes_adc[0] = (uint8_t)((cmd & 0xff00) >> 8);
    outgoing_bytes_adc[1] = (uint8_t)(cmd & 0x00ff);
    outgoing_bytes_adc[2] = (uint8_t)((reg_value & 0xff00) >> 8);
    outgoing_bytes_adc[3] = (uint8_t)(reg_value & 0x00ff);
    for (uint i = 4; i < N_BYTES_IN_FRAME; ++i) {
        outgoing_bytes_adc[i] = (uint8_t)0;
    }
}

static inline void prepare_to_write_adc_clock_register(uint8_t OSR_bits, uint8_t PWR_bits)
// Set the oversampling ratio and power bits by writing to the CLOCK register (0x01).
{
    // All channels enabled (bits 11:8); TBM (bit 5, turbo mode) clear.
    uint16_t clock_reg = 0x0f00 | (OSR_bits << 2) | PWR_bits;
    prepare_to_write_adc_register(0x03, clock_reg);
}

static uint dma_spi0_tx, dma_spi0_rx, dma_spi1_tx, dma_spi1_rx;
static uint dma_spi2_tx, dma_spi2_rx, dma_spi3_tx, dma_spi3_rx;
static dma_channel_config tx0_cfg, rx0_cfg, tx1_cfg, rx1_cfg;
static dma_channel_config tx2_cfg, rx2_cfg, tx3_cfg, rx3_cfg;

static inline int exchange_full_frame_adc0123(uint32_t timeout_period_us)
// Returns 0 on a successful transfer, 1 on time-out.
{
    gpio_put(CS_0_PIN, 0); // Select the ADS131M04-0 chip.
    gpio_put(CS_1_PIN, 0); // Select the ADS131M04-1 chip.
    gpio_put(CS_2_PIN, 0); // Select the ADS131M04-2 chip.
    gpio_put(CS_3_PIN, 0); // Select the ADS131M04-3 chip.
    busy_wait_us(1);
    // spi_write_read_blocking(spi0, outgoing_bytes_adc, incoming_bytes_adc, N_BYTES_IN_FRAME);
    //
    // 2026-10-22 PJ For the moment, get the DMA module to do the data transfers
    // but keep all of the code here so that it is just a simple replacement for the
    // original blocking call above.  Eventually, we will make better use of all of
    // this machinery by starting off exchanges with all 4 ADS131M04 chips letting
    // them independently complete.
    uint64_t timeout = time_us_64() + timeout_period_us;
    dma_channel_configure(dma_spi0_tx, &tx0_cfg,
                          (io_rw_8*)&spi_get_hw(spi0)->dr, // write address
                          outgoing_bytes_adc, // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    dma_channel_configure(dma_spi0_rx, &rx0_cfg,
                          incoming_bytes_adc[0], // write address
                          (io_rw_8*)&spi_get_hw(spi0)->dr, // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    dma_channel_configure(dma_spi1_tx, &tx1_cfg,
                          (io_rw_8*)&spi_get_hw(spi1)->dr, // write address
                          outgoing_bytes_adc, // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    dma_channel_configure(dma_spi1_rx, &rx1_cfg,
                          incoming_bytes_adc[1], // write address
                          (io_rw_8*)&spi_get_hw(spi1)->dr, // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    dma_channel_configure(dma_spi2_tx, &tx2_cfg,
                          (io_rw_8*)&(pio0->txf[0]), // write address
                          outgoing_bytes_adc, // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    dma_channel_configure(dma_spi2_rx, &rx2_cfg,
                          incoming_bytes_adc[2], // write address
                          (io_rw_8*)&(pio0->rxf[0]), // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    dma_channel_configure(dma_spi3_tx, &tx3_cfg,
                          (io_rw_8*)&(pio1->txf[0]), // write address
                          outgoing_bytes_adc, // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    dma_channel_configure(dma_spi3_rx, &rx3_cfg,
                          incoming_bytes_adc[3], // write address
                          (io_rw_8*)&(pio1->rxf[0]), // read address
                          dma_encode_transfer_count(N_BYTES_IN_FRAME),
                          false); // start later...
    // At this point, the outgoing data bytes are in place and ready to be sent via SPI0.
    dma_start_channel_mask((1u << dma_spi0_tx) | (1u << dma_spi0_rx) |
                           (1u << dma_spi1_tx) | (1u << dma_spi1_rx) |
                           (1u << dma_spi2_tx) | (1u << dma_spi2_rx) |
                           (1u << dma_spi3_tx) | (1u << dma_spi3_rx));
    //
    // Wait for the data to be clocked out.
    while (dma_channel_is_busy(dma_spi0_tx) || dma_channel_is_busy(dma_spi0_rx) ||
           dma_channel_is_busy(dma_spi1_tx) || dma_channel_is_busy(dma_spi1_rx) ||
           dma_channel_is_busy(dma_spi2_tx) || dma_channel_is_busy(dma_spi2_rx) ||
           dma_channel_is_busy(dma_spi3_tx) || dma_channel_is_busy(dma_spi3_rx)) {
        if (time_reached(timeout)) {
            // Deselect the ADS131M04-x chips and get out, signalling time-out.
            gpio_put(CS_0_PIN, 1);
            gpio_put(CS_1_PIN, 1);
            gpio_put(CS_2_PIN, 1);
            gpio_put(CS_3_PIN, 1);
            return 1;
        }
    }
    // Normal completion, within time-out period.
    // Deselect the ADS131M04-x chips before leaving.
    gpio_put(CS_0_PIN, 1);
    gpio_put(CS_1_PIN, 1);
    gpio_put(CS_2_PIN, 1);
    gpio_put(CS_3_PIN, 1);
    return 0;
}

static inline void unpack_adc_sample_data()
{
    for (int iadc=0; iadc < n_adc_chips; ++iadc) {
        for (int ch=0; ch < 4; ++ch) {
            int32_t word = 0L;
            uint ib = (1+ch)*3; // index of the high-byte for channel ch.
            word |= (uint32_t)incoming_bytes_adc[iadc][ib] << 16;
            word |= (uint32_t)incoming_bytes_adc[iadc][ib+1] << 8;
            word |= (uint32_t)incoming_bytes_adc[iadc][ib+2];
            // Sign-extend to the full 32-bit word.
            if (word & (1L << 23)) word |= 0xFF000000L;
            sample_data[iadc*4+ch] = word;
        }
    }
}

int __no_inline_not_in_flash_func(sample_channels)(void)
// Sample the analog channels periodically and store the data in SRAM.
//
// For mode=0, we will consider that the trigger event is immediate, at sample 0,
// and the record will stop after a specified number of samples.
// So long as the record does not wrap around, the oldest sample set will start at
// byte address 0.
//
// For mode=1 or 2, we will start sampling into the SRAM circular buffer
// for an indefinite number of samples, while waiting for the trigger event.
// Once the trigger event happens, we will continue the record for a specified
// number of samples.  Because we do not keep a record of the number of times
// that the SRAM address wraps around, we just assume that the oldest sample
// starts at the next word address to be used to store a sample.
//
// Returns 0 on success, non-zero if the ADS131M04 does not have data ready when expected.
//
{
    // Get configuration data from virtual registers.
    uint32_t f_CLKIN_kHz = vregister[MCLK_KHZ];
    uint8_t OSR_bits = (uint8_t)vregister[OSR_SELECT];
    int32_t sample_period_us = 2000L * OSR_value[OSR_bits] / f_CLKIN_kHz;
    uint8_t PWR_bits = (uint8_t)vregister[PWR_SELECT];
    //
    uint8_t N_CHAN = (uint8_t)vregister[NCHANNELS];
    uint8_t mode = (uint8_t)vregister[TRIG_MODE];
    uint8_t trigger_chan = (uint8_t)vregister[TRIG_CHAN];
    uint16_t trigger_level = (uint16_t) vregister[TRIG_LEVEL];
    uint8_t trigger_slope = (uint8_t)vregister[TRIG_SLOPE];
    //
    sampling_error_flag = 0;
    next_fullword_index_in_data = 0; // Start afresh, at index 0.
    fullword_index_has_wrapped_around = 0;
    uint8_t post_event = 0;
    uint32_t samples_remaining = vregister[NSAMPLES];
    //
    // Start driving the ADS131M04 chips with a master clock via a PWM slice.
    // According to table 677 in the RP2350 data sheet,
    // we use PWM slice 1 channel A on GPIO2 (MCLK_PIN).
    uint slice_num = pwm_gpio_to_slice_num(MCLK_PIN);
    pwm_set_wrap(slice_num, 3);
    pwm_set_enabled(slice_num, false); // In case it has been left on from previous sampling.
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 2);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    float div = (float)f_clk_sys / f_CLKIN_kHz / 4;
    pwm_set_clkdiv(slice_num, div);
    pwm_set_enabled(slice_num, true);
    busy_wait_us(500); // Let the ADS131M04 clock and internal circuits settle.
    //
    // Set the OSR and PWR in the CLOCK register before the first SYNC.
    prepare_to_write_adc_clock_register(OSR_bits, PWR_bits);
    // We actually set the register by exchanging a full frame.
    // With the SPI clock running at 8MHz, we expect 1 byte transferred per microsecond.
    // Set a reasonable time-out period to give sufficient time for 18 bytes to be exchanged.
    // FIX-ME PJ 2026-10-22 We should check the return value from this call.
    exchange_full_frame_adc0123(100);
    // Read the response.
    prepare_adc_command(0); // NULL command
    uint64_t timeout = time_us_64() + 10000; // 10ms timeout
    if (wait_for_data_ready_adc0(timeout)) return 4;
    busy_wait_us(1);
    exchange_full_frame_adc0123(100);
    // At this point, we have the response to the register-write command
    // sitting in the incoming buffer.
    // FIX-ME PJ 2026-10-21 we should check that the register was written.
    //
    // Assert the SYNCHn pin to reset the ADS131M04 and activate sampling
    // with the new OSR setting.
    timeout = time_us_64() + 10;
    uint64_t period_CLKIN_ns = 1000000 / f_CLKIN_kHz;
    if (assert_adc_synch(3*period_CLKIN_ns, timeout)) return 99;
    //
    // After SYNC, the filter needs time to settle based on the OSR value.
    // Per datasheet Table 8-3, settling times vary by OSR and
    // for OSR >= 128, 3 sample periods appears to be adequate.
    // We read 3 frames and ignore the data to clear the pipeline.
    for (int i = 0; i < 3; i++) {
        timeout = time_us_64() + sample_period_us + 1000;
        if (wait_for_data_ready_adc0(timeout)) return (1 + i);
        busy_wait_us(1);
        exchange_full_frame_adc0123(100);
    }
    //
    while (samples_remaining > 0) {
        sampling_LED_ON();
        //
        // Take the analog sample set.
        // Keep timeout tight to avoid FIFO issues (max 2 samples in FIFO)
        timeout = time_us_64() + sample_period_us + 1000;
        if (wait_for_data_ready_adc0(timeout)) return 9;
        busy_wait_us(1);
        exchange_full_frame_adc0123(100);
        unpack_adc_sample_data();
        for (uint ch=0; ch < N_CHAN; ++ch) {
            data[next_fullword_index_in_data+ch] = sample_data[ch];
        }
        //
        if (post_event) {
            // Trigger event has happened.
            samples_remaining--;
        } else {
            // We need to decide about trigger event.
            switch (mode) {
            case TRIGGER_IMMEDIATE:
                post_event = 1;
                assert_event();
                break;
            case TRIGGER_INTERNAL: {
                // Pick the particular channel value out of the recently-stored data.
                int32_t s = sample_data[trigger_chan];
                if ((trigger_slope == SLOPE_POS && s >= trigger_level) ||
                    (trigger_slope == SLOPE_NEG && s <= trigger_level)) {
                    post_event = 1;
                    assert_event();
                }
                }
                break;
            case TRIGGER_EXTERNAL:
                if (read_system_event_pin() == 0) {
                    post_event = 1;
                }
            } // end switch
        }
        // Point to the next available fullword index.
        next_fullword_index_in_data += N_CHAN;
        if (next_fullword_index_in_data >= N_FULLWORDS) {
            next_fullword_index_in_data -= N_FULLWORDS;
            fullword_index_has_wrapped_around = 1;
        }
        sampling_LED_OFF();
    } // end while (main sampling loop)
    //
    pwm_set_enabled(slice_num, false);
    // If we arrive here, there have been no observed failures.
    return 0;
} // end int sample_channels()

void sample_channels_once()
{
    // We temporarily override some of the registers to make this happen.
    uint8_t mode_save = (uint8_t)vregister[TRIG_MODE];
    uint16_t samples_remaining_save = (uint16_t)vregister[NSAMPLES];
    //
    vregister[TRIG_MODE] = 0; // Immediate mode.
    vregister[NSAMPLES] = 1; // One sample set.
    // Note that, even though we ask for one sample,
    // two sample reads will be made because there will be 1 sample
    // leading to the immediate trigger event and one after trigger.
    // It should not matter.
    sampling_error_flag = sample_channels();
    //
    // Restore register values.
    vregister[TRIG_MODE] = mode_save;
    vregister[NSAMPLES] = samples_remaining_save;
    return;
} // end sample_channels_once()

#define NSTRBUF1 128
char str_buf1[NSTRBUF1];
#define NSTRBUF2 16
char str_buf2[NSTRBUF2];

char* sample_set_to_str(uint32_t n)
{
    // Start with index of oldest sample, then move to selected sample.
    uint8_t N_CHAN = (uint8_t)vregister[NCHANNELS];
    uint32_t word_index = oldest_fullword_index_in_data() + N_CHAN*n;
    // Assume that the fullword sets in the data wrap neatly.
	if (word_index > N_FULLWORDS) { word_index -= N_FULLWORDS; }
    snprintf(str_buf1, NSTRBUF1, "%d", data[word_index+0]);
    for (uint8_t ch=1; ch < N_CHAN; ch++) {
        snprintf(str_buf2, NSTRBUF2, " %d", data[word_index+ch]);
        strncat(str_buf1, str_buf2, NSTRBUF2);
    }
    return str_buf1;
}

char* mem_dump_to_str(uint32_t addr)
// Write a 32-byte page of sample data into the serial buffer as hex digits
// representing the 16-bit converted samples. Use big-endian format.
// Finally, return a pointer to the buffer.
//
// FIX-ME PJ 2026-10-19 Adapt to the 24-bit data from the ADS131M04.
//
{
    // Assume that the byte address is correctly aligned and
    // that there are 16 2-byte values to send.
	uint32_t ihw = addr / 2; // half-word index
    uint8_t high_byte = (uint8_t) ((data[ihw] & 0xff00) >> 8);
    uint8_t low_byte = (uint8_t) (data[ihw] & 0x00ff);
    snprintf(str_buf1, NSTRBUF1, "%02x%02x", high_byte, low_byte);
    for (uint8_t ch=1; ch < 16; ch++) {
        high_byte = (uint8_t) ((data[ihw+ch] & 0xff00) >> 8);
        low_byte = (uint8_t) (data[ihw+ch] & 0x00ff);
        snprintf(str_buf2, NSTRBUF2, "%02x%02x", high_byte, low_byte);
        strncat(str_buf1, str_buf2, NSTRBUF2);
    }
	return str_buf1;
}


// For incoming serial comms
#define NBUFA 80
char bufA[NBUFA];

int getstr(char* buf, int nbuf)
// Read (without echo) a line of characters into the buffer,
// stopping when we see a new-line character.
// Returns the number of characters collected,
// excluding the terminating null character.
{
	memset(buf, '\0', nbuf);
    int i = 0;
    char c;
    uint8_t done = 0;
    while (!done) {
        c = getc(stdin);
        if (c != '\n' && c != '\r' && c != '\b' && i < (nbuf-1)) {
            // Append a normal character.
            buf[i] = c;
            i++;
        }
        if (c == '\n') {
            done = 1;
            buf[i] = '\0';
        }
        if (c == '\b' && i > 0) {
            // Backspace.
            i--;
        }
    }
    return i;
} // end getstr()

int trim_string(char *str)
// Trim space characters from the start and end of the string.
// We assume that the string is properly null terminated.
// Returns the number of characters in the trimmed string, excluding the terminating '\0'.
{
	int len = strlen(str);
	if (len == 0) return 0;
	int start = 0;
	while (isspace((unsigned char)str[start])) {
		start += 1;
	}
	if (start == len) {
	    // There are no non-space characters left.
		str[0] = '\0';
		return 0;
	}
	// At this point, we have at least one non-space character.
	if (start > 0) {
		// Move all remaining characters.
		memmove(str, str+start, len-start);
		len -= start;
	}
	str[len] = '\0';
	int last = len - 1;
	while ((last >= 0) && isspace((unsigned char)str[last])) {
		str[last] = '\0';
		last -= 1;
	}
	return last+1; // final string length
}

void interpret_command(char* cmdStr)
// The first character in the string specifies the command.
// Any following text is interpreted as delimiter-separated integers
// and used as arguments for the command.
// A successful command should echo the first character,
// followed by any results as the message.
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    uint8_t i;
	int16_t v;
    // printf("DEBUG: DAQ MCU cmdStr=\"%s\"", cmdStr);
    if (!override_led) gpio_put(LED_PIN, 1); // To indicate start of interpreter activity.
    switch (cmdStr[0]) {
	case 'v':
		// Report version string and (some) configuration details.
		uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
		printf("%s 4x ADS131M04 %d kHz ok\n", VERSION_STR, f_clk_sys);
		break;
	case 'L':
		// Turn LED on or off.
		// Turning the LED on by command overrides its use
		// as an indicator of interpreter activity.
		token_ptr = strtok(&cmdStr[1], sep_tok);
		if (token_ptr) {
			// Found some non-blank text; assume on/off value.
			// Use just the least-significant bit.
			i = (uint8_t) (atoi(token_ptr) & 1);
			gpio_put(LED_PIN, i);
			override_led = i;
			printf("%d ok\n", i);
		} else {
			// There was no text to give a value.
			printf("fail: no value\n");
		}
		break;
	case 'n':
		// Report number of virtual registers.
		printf("%d ok\n", NUMREG);
		break;
    case 'r':
        // Report a selected register value.
        token_ptr = strtok(&cmdStr[1], sep_tok);
        if (token_ptr) {
            // Found some nonblank text, assume register number.
            i = (uint8_t) atoi(token_ptr);
            if (i < NUMREG) {
                v = vregister[i];
                printf("%d ok\n", v);
            } else {
                printf("fail: invalid register.\n");
            }
        } else {
            printf("fail: no register specified.\n");
        }
        break;
    case 's':
        // Set a selected register value.
        token_ptr = strtok(&cmdStr[1], sep_tok);
        if (token_ptr) {
            // Found some nonblank text; assume register number.
            i = (uint8_t) atoi(token_ptr);
            if (i < NUMREG) {
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    // Assume text is value for register.
                    v = (int) atoi(token_ptr);
                    // Accept user-specified value.
                    vregister[i] = v;
                    printf("reg[%u] %d ok\n", i, v);
                } else {
                    printf("fail: no value given.\n");
                }
            } else {
                printf("fail: invalid register.\n");
            }
        } else {
            printf("fail: no register specified.\n");
        }
        break;
    case 'F':
        // Set the values of the registers to those values hard-coded
        // into this firmware.  A factory default, so to speak.
        set_registers_to_original_values();
        printf("vregisters reset ok\n");
        break;
    case 'b':
        // Byte size of sample set.
        printf("%d ok\n", byte_size_of_sample_set());
        break;
    case 'm':
        // Report the maximum number of sample sets that can be stored in SRAM.
        printf("%d ok\n", max_n_samples());
        break;
    case 'T':
        // Total bytes assigned to sample storage.
        printf("%d ok\n", N_FULLWORDS*4);
        break;
    case 'a':
        // Report byte index of oldest sample set in data[].
        printf("%u ok\n", oldest_fullword_index_in_data() * 4);
        break;
    case 'A':
        // Byte index (within sample data storage) of the next sample to be stored.
        printf("%d ok\n", next_fullword_index_in_data*4);
        break;
    case 'N':
        // Size of data storage in 32-byte pages.
        printf("%d ok\n", N_FULLWORDS/8);
        break;
    case 'M':
        // FIX-ME PJ 2026-10-19 Need to implement for this board.
        // Page of bytes (in big-endian format) representing 16 2-byte values.
        // Start at the specified byte-index within the stored data.
        token_ptr = strtok(&cmdStr[1], sep_tok);
        if (token_ptr) {
            // Found some nonblank text, assume byte-index.
            uint32_t addr = (uint32_t) atoi(token_ptr);
            printf("%s ok\n", mem_dump_to_str(addr));
        } else {
            printf("fail: no byte-index specified.\n");
        }
        break;
    case 'g':
        // Start the sampling process.
        // What happens next, and when it happens, depends on the
        // register settings and external signals.
        printf("start sampling ok\n");
        // The task takes an indefinite time,
        // so let the COMMS_MCU know via busy# pin.
        assert_not_ready();
        sampling_error_flag = sample_channels();
        assert_ready();
        break;
    case 'k':
        // Report the value of the sampling_error_flag flag.
        printf("k %u ok\n", sampling_error_flag);
        break;
    case 'I':
        // Immediately take a single sample set and report values.
        sample_channels_once();
        // Send back the second set of samples to allow
        // the ADC more time to settle.
        printf("%s ok\n", sample_set_to_str(1));
        // Turns out that this makes no observable difference.
        break;
    case 'P':
        // Report the selected sample set for the configured channels.
        // An index of 0 refers to the oldest sample set.
        token_ptr = strtok(&cmdStr[1], sep_tok);
        if (token_ptr) {
            // Found some nonblank text, assume sample index.
            uint32_t ii = (uint32_t) atol(token_ptr);
            printf("%s ok\n", sample_set_to_str(ii));
        } else {
            printf("fail: no index given.\n");
        }
        break;
    case 'z':
        // Release the EVENT# line.
        // Presumably this line has been help low following an internal
        // trigger event during the sampling process.
        release_event();
        printf("event line released ok\n");
        break;
	default:
		printf("fail: Unknown command %c\n", cmdStr[0]);
    }
    if (!override_led) gpio_put(LED_PIN, 0); // To indicate end of interpreter activity.
} // end interpret_command()


int main()
{
	set_registers_to_original_values();
    stdio_init_all();
	uart_set_baudrate(uart0, 230400);
	// Attempt to discard any characters sitting the UART already.
	while (uart_is_readable_within_us(uart0, 100)) {
		__unused char junkc = uart_getc(uart0);
	}
	//
    // Some information for picotool.
	//
    bi_decl(bi_program_description(VERSION_STR));
    bi_decl(bi_1pin_with_name(LED_PIN, "LED output pin"));
    bi_decl(bi_1pin_with_name(READY_PIN, "Ready/Busy# output pin"));
    bi_decl(bi_1pin_with_name(EVENT_PIN, "EVENT input/output pin"));
    bi_decl(bi_1pin_with_name(SYNC_PIN, "ADS131M04-x SYNC, ouput pin"));
    bi_decl(bi_1pin_with_name(MCLK_PIN, "ADS131M04-x CLKIN, PWM ouput pin"));
    // SPI0 <-> ADC-0
    bi_decl(bi_1pin_with_name(CS_0_PIN, "ADS131M04-0 chip select, output pin"));
    bi_decl(bi_3pins_with_func(MISO_0_PIN, MOSI_0_PIN, SCK_0_PIN, GPIO_FUNC_SPI));
    bi_decl(bi_1pin_with_name(DRDY_0_PIN, "ADS131M04-0 DRDY, input pin"));
    // SPI1 <-> ADC-1
    bi_decl(bi_1pin_with_name(CS_1_PIN, "ADS131M04-1 chip select, output pin"));
    bi_decl(bi_3pins_with_func(MISO_1_PIN, MOSI_1_PIN, SCK_1_PIN, GPIO_FUNC_SPI));
    bi_decl(bi_1pin_with_name(DRDY_1_PIN, "ADS131M04-1 DRDY, input pin"));
    // PIO-SPI-2 <-> ADC-2
    bi_decl(bi_4pins_with_names(CS_2_PIN, "ADS131M04-2 chip select, output pin",
                                MISO_2_PIN, "ADS131M04-2 MISO, input pin",
                                SCK_2_PIN, "ADS131M04-2 SCK, output pin",
                                MOSI_2_PIN, "ADS131M04-2 MOSI, input pin"));
    bi_decl(bi_1pin_with_name(DRDY_2_PIN, "ADS131M04-2 DRDY, input pin"));
    // PIO-SPI-3 <-> ADC-3
    bi_decl(bi_4pins_with_names(CS_3_PIN, "ADS131M04-3 chip select, output pin",
                                MISO_3_PIN, "ADS131M04-3 MISO, input pin",
                                SCK_3_PIN, "ADS131M04-3 SCK, output pin",
                                MOSI_3_PIN, "ADS131M04-3 MOSI, input pin"));
    bi_decl(bi_1pin_with_name(DRDY_3_PIN, "ADS131M04-3 DRDY, input pin"));
	//
	// Flash LED twice at start up.
	//
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
	sampling_LED_ON(); sleep_ms(500);
	sampling_LED_OFF(); sleep_ms(500);
	sampling_LED_ON(); sleep_ms(500);
	sampling_LED_OFF();
	//
	// Configure pins to drive the ADS131M04-0 chip with SPI0.
	//
    gpio_init(CS_0_PIN);
    gpio_set_dir(CS_0_PIN, GPIO_OUT);
    gpio_set_slew_rate(CS_0_PIN, GPIO_SLEW_RATE_FAST);
    gpio_put(CS_0_PIN, 1); // High to deselect.
    // Limit SPI0 clock to 4MHz.
    spi_init(spi0, 4000*1000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(MISO_0_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_0_PIN, GPIO_FUNC_SPI);
    gpio_set_slew_rate(MOSI_0_PIN, GPIO_SLEW_RATE_FAST);
    gpio_set_function(SCK_0_PIN, GPIO_FUNC_SPI);
    gpio_set_slew_rate(SCK_0_PIN, GPIO_SLEW_RATE_FAST);
    //
    gpio_init(DRDY_0_PIN);
    gpio_set_dir(DRDY_0_PIN, GPIO_IN);
    //
    // Transfer bytes to and from the SPI0 peripheral via DMA channels.
    dma_spi0_tx = dma_claim_unused_channel(true);
    dma_spi0_rx = dma_claim_unused_channel(true);
    tx0_cfg = dma_channel_get_default_config(dma_spi0_tx);
    rx0_cfg = dma_channel_get_default_config(dma_spi0_rx);
    channel_config_set_transfer_data_size(&tx0_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx0_cfg, true);
    channel_config_set_write_increment(&tx0_cfg, false);
    channel_config_set_transfer_data_size(&rx0_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx0_cfg, false);
    channel_config_set_write_increment(&rx0_cfg, true);
    // The dma-transfer requests are paced by the SPI peripheral.
    channel_config_set_dreq(&tx0_cfg, spi_get_dreq(spi0, true)); // tx
    channel_config_set_dreq(&rx0_cfg, spi_get_dreq(spi0, false)); // rx
	//
	// Configure pins to drive the ADS131M04-1 chip with SPI1.
	//
    gpio_init(CS_1_PIN);
    gpio_set_dir(CS_1_PIN, GPIO_OUT);
    gpio_set_slew_rate(CS_1_PIN, GPIO_SLEW_RATE_FAST);
    gpio_put(CS_1_PIN, 1); // High to deselect.
    //
    // Limit SPI1 clock to 4MHz.
    spi_init(spi1, 4000*1000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(MISO_1_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_1_PIN, GPIO_FUNC_SPI);
    gpio_set_slew_rate(MOSI_1_PIN, GPIO_SLEW_RATE_FAST);
    gpio_set_function(SCK_1_PIN, GPIO_FUNC_SPI);
    gpio_set_slew_rate(SCK_1_PIN, GPIO_SLEW_RATE_FAST);
    //
    gpio_init(DRDY_1_PIN);
    gpio_set_dir(DRDY_1_PIN, GPIO_IN);
    //
    // Transfer bytes to and from the SPI1 peripheral via DMA channels.
    dma_spi1_tx = dma_claim_unused_channel(true);
    dma_spi1_rx = dma_claim_unused_channel(true);
    tx1_cfg = dma_channel_get_default_config(dma_spi1_tx);
    rx1_cfg = dma_channel_get_default_config(dma_spi1_rx);
    channel_config_set_transfer_data_size(&tx1_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx1_cfg, true);
    channel_config_set_write_increment(&tx1_cfg, false);
    channel_config_set_transfer_data_size(&rx1_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx1_cfg, false);
    channel_config_set_write_increment(&rx1_cfg, true);
    // The dma-transfer requests are paced by the SPI peripheral.
    channel_config_set_dreq(&tx1_cfg, spi_get_dreq(spi1, true)); // tx
    channel_config_set_dreq(&rx1_cfg, spi_get_dreq(spi1, false)); // rx
    //
    // We implement spi2 and spi3 with state machines in the PIO 0 and 1 modules,
    // with both state machines using a common program.
    uint offset0 = pio_add_program(pio0, &spi_cpol0_cpha1_program);
    uint offset1 = pio_add_program(pio1, &spi_cpol0_cpha1_program);
    //
    // Configure pins to drive the ADS131M04-2 chip via pio0 sm=0 implementing spi2
    //
    gpio_init(CS_2_PIN);
    gpio_set_dir(CS_2_PIN, GPIO_OUT);
    gpio_set_slew_rate(CS_2_PIN, GPIO_SLEW_RATE_FAST);
    gpio_put(CS_2_PIN, 1); // High to deselect.
    //
    pio_spi_init(pio0, 0, offset0, SCK_2_PIN, MOSI_2_PIN, MISO_2_PIN);
    //
    gpio_init(DRDY_2_PIN);
    gpio_set_dir(DRDY_2_PIN, GPIO_IN);
    //
    // Transfer bytes to and from the spi2 peripheral via DMA channels.
    dma_spi2_tx = dma_claim_unused_channel(true);
    dma_spi2_rx = dma_claim_unused_channel(true);
    tx2_cfg = dma_channel_get_default_config(dma_spi2_tx);
    rx2_cfg = dma_channel_get_default_config(dma_spi2_rx);
    channel_config_set_transfer_data_size(&tx2_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx2_cfg, true);
    channel_config_set_write_increment(&tx2_cfg, false);
    channel_config_set_transfer_data_size(&rx2_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx2_cfg, false);
    channel_config_set_write_increment(&rx2_cfg, true);
    // The dma-transfer requests are paced by the PIO peripheral.
    channel_config_set_dreq(&tx2_cfg, pio_get_dreq(pio0, 0, true)); // tx
    channel_config_set_dreq(&rx2_cfg, pio_get_dreq(pio0, 0, false)); // rx
    //
    // Configure pins to drive the ADS131M04-3 chip via pio1 sm=0 implementing spi3
    //
    gpio_init(CS_3_PIN);
    gpio_set_dir(CS_3_PIN, GPIO_OUT);
    gpio_set_slew_rate(CS_3_PIN, GPIO_SLEW_RATE_FAST);
    gpio_put(CS_3_PIN, 1); // High to deselect.
    //
    pio_spi_init(pio1, 0, offset1, SCK_3_PIN, MOSI_3_PIN, MISO_3_PIN);
    //
    gpio_init(DRDY_3_PIN);
    gpio_set_dir(DRDY_3_PIN, GPIO_IN);
    //
    // Transfer bytes to and from the spi3 peripheral via DMA channels.
    dma_spi3_tx = dma_claim_unused_channel(true);
    dma_spi3_rx = dma_claim_unused_channel(true);
    tx3_cfg = dma_channel_get_default_config(dma_spi3_tx);
    rx3_cfg = dma_channel_get_default_config(dma_spi3_rx);
    channel_config_set_transfer_data_size(&tx3_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx3_cfg, true);
    channel_config_set_write_increment(&tx3_cfg, false);
    channel_config_set_transfer_data_size(&rx3_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx3_cfg, false);
    channel_config_set_write_increment(&rx3_cfg, true);
    // The dma-transfer requests are paced by the PIO peripheral.
    channel_config_set_dreq(&tx3_cfg, pio_get_dreq(pio1, 0, true)); // tx
    channel_config_set_dreq(&rx3_cfg, pio_get_dreq(pio1, 0, false)); // rx
    //
    // Configure SYNC pin for manual interaction.
    //
    gpio_init(SYNC_PIN);
    gpio_set_dir(SYNC_PIN, GPIO_OUT);
    gpio_set_slew_rate(SYNC_PIN, GPIO_SLEW_RATE_FAST);
    // We are driving all 4 ADC chips with this signal so increase drive strength.
    gpio_set_drive_strength(SYNC_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_put(SYNC_PIN, 1); // Idle high.
    //
    // We use PWM slice 1A on GPIO2 (MCLK_PIN).
    gpio_set_function(MCLK_PIN, GPIO_FUNC_PWM);
    gpio_set_slew_rate(MCLK_PIN, GPIO_SLEW_RATE_FAST);
    // We are driving all 4 ADC chips with this signal so increase drive strength.
    gpio_set_drive_strength(MCLK_PIN, GPIO_DRIVE_STRENGTH_12MA);
	//
	gpio_init(EVENT_PIN);
	gpio_set_dir(EVENT_PIN, GPIO_IN);
    gpio_disable_pulls(EVENT_PIN);
	release_event();
	//
	// Signal to the COMMS MCU that we are ready.
	//
    gpio_init(READY_PIN);
    gpio_set_dir(READY_PIN, GPIO_OUT);
	assert_ready();
    //
	// Enter the command loop.
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // NL (Ctrl-J) signals end of incoming string.
        int m = getstr(bufA, NBUFA);
		m = trim_string(bufA);
        // Note that the cmd string may be of zero length,
        // with the null character in the first place.
        // If that is the case, do nothing with it
		// but just reply with a newline character.
        if (m > 0) {
            interpret_command(bufA);
        } else {
			printf("fail: empty command\n");
		}
    }
    //
    spi_deinit(spi0);
    spi_deinit(spi1);
    //
    dma_channel_cleanup(dma_spi0_tx);
    dma_channel_cleanup(dma_spi0_rx);
    dma_channel_cleanup(dma_spi1_tx);
    dma_channel_cleanup(dma_spi1_rx);
    dma_channel_cleanup(dma_spi2_tx);
    dma_channel_cleanup(dma_spi2_rx);
    dma_channel_cleanup(dma_spi3_tx);
    dma_channel_cleanup(dma_spi3_rx);
    //
    dma_channel_unclaim(dma_spi0_tx);
    dma_channel_unclaim(dma_spi0_rx);
    dma_channel_unclaim(dma_spi1_tx);
    dma_channel_unclaim(dma_spi1_rx);
    dma_channel_unclaim(dma_spi2_tx);
    dma_channel_unclaim(dma_spi2_rx);
    dma_channel_unclaim(dma_spi3_tx);
    dma_channel_unclaim(dma_spi3_rx);
    return 0;
}
