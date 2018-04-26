
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

volatile int posr = 0;
volatile int pwmr = 0;

volatile uint32_t counter = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t enable = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 650


inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

uint16_t buzzerTimer        = 0;

int last_pos = 0;
int timer = 0;
const int max_time = PWM_FREQ / 10;
volatile int vel = 0;

void setPWM() {
  posr++;
  if (posr > 5) {
    posr = 0;
  }
  int ur, vr, wr;

  blockPWM(pwmr, posr, &ur, &vr, &wr);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 6, pwm_res-6);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 6, pwm_res-6);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 6, pwm_res-6);
}
