#include <math.h>
#include <stdio.h>
#include <string.h>

#include <cstdlib>
#include <vector>

#include "Adafruit_NeoPixel.hpp"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "kiss_fftr.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 960

// max capture frequency
#define FSAMP 50000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 200

#define NUMLEDS 21

#define PIN 9
Adafruit_NeoPixel strip = Adafruit_NeoPixel(21, PIN, NEO_GRB + NEO_KHZ800);

// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];

int min_power[NSAMP / 2];
int max_power[NSAMP / 2];
int power[NSAMP / 2];
float ratio[NSAMP / 2];

float boundcreep = 0.9995;  // How much max_power & min_power boundaries
                            // persists between loops.

int led_b[NUMLEDS];  // LED brightness
float fade = 0.9;    // How much brightness persists between loops.

float hipass = 0.1;  // how much should we chop off the bottom of the power
                     // levels to hide noise.

void setup();
void sample(uint8_t* capture_buf);

int main() {
  uint8_t cap_buf[NSAMP];
  kiss_fft_scalar fft_in[NSAMP];  // kiss_fft_scalar is a float
  kiss_fft_cpx fft_out[NSAMP];
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);

  // setup ports and outputs
  setup();

  while (1) {
    // get NSAMP samples at FSAMP
    sample(cap_buf);

    // fill fourier transform input while subtracting DC component
    uint64_t sum = 0;
    for (int i = 0; i < NSAMP; i++) {
      sum += cap_buf[i];
    }
    float avg = (float)sum / NSAMP;
    for (int i = 0; i < NSAMP; i++) {
      fft_in[i] = (float)cap_buf[i] - avg;
    }

    // compute fast fourier transform
    kiss_fftr(cfg, fft_in, fft_out);

    // compute power and calculate max freq component
    float max_bin_power = 0;
    int max_bin_idx = 0;

    // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
    for (int i = 0; i < NSAMP / 2; i++) {
      power[i] = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;

      if (power[i] > max_power[i]) {
        max_power[i] = power[i];
      } else {
        max_power[i] = max_power[i] * boundcreep;
      }

      if (power[i] < min_power[i]) {
        min_power[i] = power[i];
      } else {
        min_power[i] = min_power[i] + power[i] * (1 - boundcreep);
      }
      if (power[i] > max_bin_power && i > 1) {
        max_bin_power = power[i];
        max_bin_idx = i;
      }

      ratio[i] = float(power[i] - min_power[i]) / (max_power[i] - min_power[i]);
    }

    // begin LED management
    // we've got (NSAMP / 2)-1 bins to fit into NUMLEDS leds. (the -1 is because
    // the first bin is always full of jank) initial i=1 -> numleds + 1 also
    // becasue of jank bin

    for (int i = 1; i < NUMLEDS + 1; i++) {
      // printf("Bin %d, Power %d, Max_Power %d, min_power %d, ratio %f \n",i,
      // power[i],max_power[i],min_power[i],ratio[i]);
      float lratio = 0;
      if (ratio[i] > hipass) {
        lratio = (ratio[i] - hipass) * (1 / 1 - hipass);
      }

      int hue = (240 * 65535 / 360);
      if (i == max_bin_idx) {
        hue = 0;
      }
      int saturation = 200;
      if (lratio * 255 > led_b[i]) {
        led_b[i] = lratio * 255;
      } else {
        led_b[i] = led_b[i] * fade;
      }
      // printf("hue %d, sat %d, value %d\n");
      uint32_t rgbcolor = strip.ColorHSV(hue, saturation, led_b[i]);
      strip.setPixelColor(i - 1, rgbcolor);
    }

    strip.show();
  }

  // should never get here
  kiss_fft_free(cfg);
}

void sample(uint8_t* capture_buf) {
  adc_fifo_drain();
  adc_run(false);

  dma_channel_configure(dma_chan, &cfg,
                        capture_buf,    // dst
                        &adc_hw->fifo,  // src
                        NSAMP,          // transfer count
                        true            // start immediately
  );

  gpio_put(LED_PIN, 1);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
}

void setup() {
  strip.begin();  // initialize LED strip
  strip.setBrightness(50);

  stdio_init_all();

  adc_gpio_init(26 + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
      true,   // Write each completed conversion to the sample FIFO
      true,   // Enable DMA data request (DREQ)
      1,      // DREQ (and IRQ) asserted when at least 1 sample present
      false,  // We won't see the ERR bit because of 8 bit reads; disable.
      true    // Shift each sample to 8 bits when pushing to FIFO
  );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);

  sleep_ms(1000);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  // calculate frequencies of each bin
  float f_max = FSAMP;
  float f_res = f_max / NSAMP;

  for (int i = 0; i < NSAMP; i++) {
    freqs[i] = f_res * i;
    printf("%0.1f Hz, ", freqs[i]);
  }
}
