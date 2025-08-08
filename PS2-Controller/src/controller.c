// ATmega88 Microlind Bus RTC + PS/2 Interface Skeleton

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

// -------------------- Config --------------------
#define FIFO_SIZE 64
#define EEPROM_ADDR_LOWBAT 0x10
#define EEPROM_ADDR_EEPROM_ERROR_FLAG 0x11
#define EEPROM_ADDR_TX_ACK_FAIL 0x12
#define EEPROM_ADDR_RTC_BASE 0x20
#define EEPROM_RTC_SLOTS 40
#define EEPROM_RTC_SIZE 8  // 7 bytes data + 1 checksum

#define LOW_BATTERY_THRESHOLD 2200  // millivolts after the diode voltage drop, so 2.8 ish at the cell.
#define POWER_ON_VOLTAGE_THRESHOLD 3700  // Above this we assume the VCC from computer is OK.

// Pin assignments
#define BUS_EN_PIN    PB0   // Enable input
#define BUS_RD_PIN    PB3   // Read strobe
#define BUS_WR_PIN    PB4   // Write strobe
#define BUS_IRQ_KB    PB1   // Keyboard IRQ output
#define BUS_IRQ_MS    PB2   // Mouse IRQ output
#define BUS_ADDR_A0   PC0   // Address line A0
#define BUS_ADDR_A1   PC1   // Address line A1
#define BUS_ADDR_A2   PC2   // Address line A2
#define BUS_DATA_PORT PORTD
#define BUS_DATA_DDR  DDRD
#define BUS_DATA_PIN  PIND

#define PS2_KEYBOARD_CLK  PC3
#define PS2_KEYBOARD_DATA PC4
#define PS2_MOUSE_CLK     PC5
#define PS2_MOUSE_DATA    PC6

#define SPARE_PIN PB5

// -------------------- Globals --------------------
typedef struct {
  uint8_t buffer[FIFO_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
} FIFO;

FIFO kb_fifo = {{0}, 0, 0};
FIFO ms_fifo = {{0}, 0, 0};
FIFO tx_fifo = {{0}, 0, 0}; // PS/2 TX buffer for keyboard

volatile uint8_t rtc_sec = 0, rtc_min = 0, rtc_hr = 0;
volatile uint8_t rtc_day = 1, rtc_month = 1, rtc_year = 25, rtc_dow = 0;
volatile bool lowbat_flag = false;
volatile bool rtc_eeprom_error = false;
volatile bool ps2_ack_error = false;
bool is_awake = false;
uint8_t rtc_save_index = 0;

volatile uint8_t ps2_kb_bitcount = 0;
volatile uint8_t ps2_kb_byte = 0;

volatile uint8_t ps2_ms_bitcount = 0;
volatile uint8_t ps2_ms_byte = 0;

// -------------------- Prototypes --------------------
void fifo_push(FIFO* f, uint8_t val);
bool fifo_pop(FIFO* f, uint8_t* val);
uint16_t read_vcc_mv();
bool is_leap_year(uint8_t year);
uint8_t days_in_month(uint8_t month, uint8_t year);
uint8_t rtc_checksum(uint8_t* data);
void save_rtc_to_eeprom();
void load_rtc_from_eeprom();
void rtc_tick();
void rtc_timer_init();
void enter_sleep();
void clear_errors();
void bus_output(uint8_t val);
void handle_bus_read();
void handle_bus_write();
void ps2_kb_tx_service();
void ps2_kb_send_byte(uint8_t byte);

// -------------------- FIFO --------------------
void fifo_push(FIFO* f, uint8_t val) {
  uint8_t next = (f->head + 1) % FIFO_SIZE;
  if (next != f->tail) {
    f->buffer[f->head] = val;
    f->head = next;
  }
}

bool fifo_pop(FIFO* f, uint8_t* val) {
  if (f->head == f->tail) return false;
  *val = f->buffer[f->tail];
  f->tail = (f->tail + 1) % FIFO_SIZE;
  return true;
}

// -------------------- Vcc Monitoring --------------------
uint16_t read_vcc_mv() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  _delay_ms(2);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC));
  uint16_t result = ADC;
  return (1100UL * 1024UL) / result;
}

// -------------------- RTC --------------------
bool is_leap_year(uint8_t year) {
  uint16_t full_year = 2000 + year;
  return ((full_year % 4 == 0 && full_year % 100 != 0) || (full_year % 400 == 0));
}

uint8_t days_in_month(uint8_t month, uint8_t year) {
  switch (month) {
    case 2: return is_leap_year(year) ? 29 : 28;
    case 4: case 6: case 9: case 11: return 30;
    default: return 31;
  }
}

uint8_t rtc_checksum(uint8_t* data) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 7; i++) sum += data[i];
  return ~sum + 1;
}

void save_rtc_to_eeprom() {
  uint8_t data[8] = { rtc_sec, rtc_min, rtc_hr, rtc_day, rtc_month, rtc_year, rtc_dow };
  data[7] = rtc_checksum(data);

  uint16_t base = EEPROM_ADDR_RTC_BASE + (rtc_save_index * EEPROM_RTC_SIZE);
  for (uint8_t i = 0; i < 8; i++) {
    eeprom_write_byte((uint8_t*)(base + i), data[i]);
  }
  rtc_save_index = (rtc_save_index + 1) % EEPROM_RTC_SLOTS;
}

void load_rtc_from_eeprom() {
  rtc_eeprom_error = true;
  for (int8_t i = EEPROM_RTC_SLOTS - 1; i >= 0; i--) {
    uint16_t base = EEPROM_ADDR_RTC_BASE + (i * EEPROM_RTC_SIZE);
    uint8_t data[8];
    for (uint8_t j = 0; j < 8; j++) {
      data[j] = eeprom_read_byte((uint8_t*)(base + j));
    }
    if (rtc_checksum(data) == data[7]) {
      rtc_sec = data[0];
      rtc_min = data[1];
      rtc_hr = data[2];
      rtc_day = data[3];
      rtc_month = data[4];
      rtc_year = data[5];
      rtc_dow = data[6];
      rtc_save_index = (i + 1) % EEPROM_RTC_SLOTS;
      rtc_eeprom_error = false;
      return;
    }
    else {
      eeprom_write_byte((uint8_t*)EEPROM_ADDR_EEPROM_ERROR_FLAG, 0xA5);
    }
  }
  // No valid record found; use default time
  rtc_sec = rtc_min = rtc_hr = 0;
  rtc_day = 1; rtc_month = 1; rtc_year = 25; rtc_dow = 0;
  rtc_save_index = 0;
}

void rtc_tick() {
  if (++rtc_sec >= 60) {
    rtc_sec = 0;
    if (++rtc_min >= 60) {
      rtc_min = 0;
      if (++rtc_hr >= 24) {
        rtc_hr = 0;
        if (++rtc_day > days_in_month(rtc_month, rtc_year)) {
          rtc_day = 1;
          if (++rtc_month > 12) {
            rtc_month = 1;
            rtc_year++;
          }
        }
        if (++rtc_dow > 6) rtc_dow = 0;
      }
    }
    // Save every minute.
    save_rtc_to_eeprom();
  }
}

// -------------------- Unified RTC Timer Init --------------------
void rtc_timer_init() {
  TCCR2A = 0x00;
  TCCR2B = (1 << CS22) | (1 << CS20);  // prescaler 128
  TIMSK2 = (1 << TOIE2);               // enable overflow interrupt
  TCNT2 = 0;
}

// -------------------- Sleep --------------------
void enter_sleep() {
  PCMSK1 &= ~(_BV(PCINT11) | _BV(PCINT13)); // Disable keyboard + mouse interrupts
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
  PCMSK1 |= (_BV(PCINT11) | _BV(PCINT13));  // Re-enable on wake
}

// -------------------- Clear all errors after read flags --------------------
void clear_errors() {
    lowbat_flag = false;
    eeprom_write_byte((uint8_t*)EEPROM_ADDR_LOWBAT, 0x00);

    rtc_eeprom_error = false;
    eeprom_write_byte((uint8_t*)EEPROM_ADDR_EEPROM_ERROR_FLAG, 0x00);

    ps2_ack_error = false;
    eeprom_write_byte((uint8_t*)EEPROM_ADDR_TX_ACK_FAIL, 0x00);
}

// -------------------- Bus Handling --------------------
void bus_output(uint8_t val) {
  BUS_DATA_DDR = 0xFF;
  BUS_DATA_PORT = val;
}

void handle_bus_read() {
  if (!(PINB & _BV(BUS_EN_PIN))) return;

  uint8_t addr_mask = (_BV(BUS_ADDR_A0) | _BV(BUS_ADDR_A1) | _BV(BUS_ADDR_A2));
  uint8_t reg = (PINC & addr_mask) >> BUS_ADDR_A0;

  uint8_t out = 0;
  switch (reg) {
    case 0: fifo_pop(&kb_fifo, &out); break;
    case 1: fifo_pop(&ms_fifo, &out); break;
    case 2: out = (rtc_year & 0x7F); break;
    case 3: out = (rtc_month & 0x0F) | (lowbat_flag << 7) | (rtc_eeprom_error << 6) | (ps2_ack_error << 5); clear_errors(); break;
    case 4: out = (rtc_day & 0x1F) | ((rtc_dow & 0x07) << 5); break;
    case 5: out = rtc_hr; break;
    case 6: out = rtc_min; break;
    case 7: out = rtc_sec; break;
  }
  bus_output(out);
}

void handle_bus_write() {
  if (!(PINB & _BV(BUS_EN_PIN))) return;

  uint8_t addr_mask = (_BV(BUS_ADDR_A0) | _BV(BUS_ADDR_A1) | _BV(BUS_ADDR_A2));
  uint8_t reg = (PINC & addr_mask) >> BUS_ADDR_A0;
  uint8_t val = BUS_DATA_PIN;

  switch (reg) {
    case 0x00: // Keyboard TX register
      fifo_push(&tx_fifo, val);
      ps2_kb_tx_service();
      break;
    case 2: rtc_year = val & 0x7F; break;
    case 3: rtc_month = val & 0x7F; break;
    case 4:
      rtc_day = val & 0x1F;
      rtc_dow = (val >> 5) & 0x07;
      break;
    case 5: rtc_hr = val; break;
    case 6: rtc_min = val; break;
    case 7: rtc_sec = val; break;
  }
}

// -------------------- PS/2 TX --------------------
void ps2_kb_tx_service() {
  uint8_t byte;
  if (fifo_pop(&tx_fifo, &byte)) {
    ps2_kb_send_byte(byte);
  }
}

void ps2_kb_send_byte(uint8_t byte) {
  uint8_t parity = 1;
  bool ack_received = false;
  cli();
  DDRC |= _BV(PS2_KEYBOARD_CLK) | _BV(PS2_KEYBOARD_DATA);

  PORTC &= ~_BV(PS2_KEYBOARD_CLK);  // Pull Clock low
  _delay_us(100);
  PORTC &= ~_BV(PS2_KEYBOARD_DATA); // Pull Data low
  _delay_us(10);
  PORTC |= _BV(PS2_KEYBOARD_CLK);   // Release Clock
  DDRC &= ~_BV(PS2_KEYBOARD_CLK);   // Float clock

  while (PINC & _BV(PS2_KEYBOARD_CLK));

  for (uint8_t i = 0; i < 8; i++) {
    uint8_t bit = (byte >> i) & 1;
    if (bit) PORTC |= _BV(PS2_KEYBOARD_DATA); else PORTC &= ~_BV(PS2_KEYBOARD_DATA);
    parity ^= bit;
    while (!(PINC & _BV(PS2_KEYBOARD_CLK)));
    while (PINC & _BV(PS2_KEYBOARD_CLK));
  }

  if (parity) PORTC |= _BV(PS2_KEYBOARD_DATA); else PORTC &= ~_BV(PS2_KEYBOARD_DATA);
  while (!(PINC & _BV(PS2_KEYBOARD_CLK)));
  while (PINC & _BV(PS2_KEYBOARD_CLK));

  DDRC &= ~_BV(PS2_KEYBOARD_DATA);
  while (PINC & _BV(PS2_KEYBOARD_DATA));
  while (PINC & _BV(PS2_KEYBOARD_CLK));
  while (!(PINC & _BV(PS2_KEYBOARD_DATA)));
  while (!(PINC & _BV(PS2_KEYBOARD_CLK)));

  // Read ACK (0xFA expected)
  uint8_t response = 0, bitcount = 0;
  ps2_kb_bitcount = 0;
  ps2_kb_byte = 0;
  for (uint8_t i = 0; i < 11; i++) {
    while ((PINC & _BV(PS2_KEYBOARD_CLK)));
    while (!(PINC & _BV(PS2_KEYBOARD_CLK)));
    uint8_t bit = (PINC & _BV(PS2_KEYBOARD_DATA)) ? 1 : 0;
    if (i >= 1 && i <= 8) {
      ps2_kb_byte >>= 1;
      if (bit) ps2_kb_byte |= 0x80;
    }
  }
  if (ps2_kb_byte == 0xFA) {
    ack_received = true;
  }

  if (!ack_received) {
    ps2_ack_error = true; 
  }

  sei();
}

// -------------------- Interrupts --------------------
ISR(TIMER2_OVF_vect) {
  rtc_tick();
}

ISR(PCINT1_vect) {
  if (!(PINC & _BV(PS2_KEYBOARD_CLK))) {
    uint8_t bit = (PINC & _BV(PS2_KEYBOARD_DATA)) ? 1 : 0;
    if (ps2_kb_bitcount >= 1 && ps2_kb_bitcount <= 8) {
      ps2_kb_byte >>= 1;
      if (bit) ps2_kb_byte |= 0x80;
    }
    ps2_kb_bitcount++;
    if (ps2_kb_bitcount == 11) {
      fifo_push(&kb_fifo, ps2_kb_byte);
      ps2_kb_bitcount = 0;
      ps2_kb_byte = 0;
      PORTB |= _BV(BUS_IRQ_KB);
    }
  }
  if (!(PINC & _BV(PS2_MOUSE_CLK))) {
    uint8_t bit = (PINC & _BV(PS2_MOUSE_DATA)) ? 1 : 0;
    if (ps2_ms_bitcount >= 1 && ps2_ms_bitcount <= 8) {
      ps2_ms_byte >>= 1;
      if (bit) ps2_ms_byte |= 0x80;
    }
    ps2_ms_bitcount++;
    if (ps2_ms_bitcount == 11) {
      fifo_push(&ms_fifo, ps2_ms_byte);
      ps2_ms_bitcount = 0;
      ps2_ms_byte = 0;
      PORTB |= _BV(BUS_IRQ_MS);
    }
  }
}

// -------------------- Main --------------------
int main() {
  cli();
  rtc_timer_init();
  PCICR |= _BV(PCIE1);
  PCMSK1 |= _BV(PCINT11) | _BV(PCINT13); // Enable PS/2 keyboard and mouse clock pin interrupts
  sei();

  while (1) {
    uint16_t vcc = read_vcc_mv();
    if (!is_awake && vcc >= POWER_ON_VOLTAGE_THRESHOLD) {
      is_awake = true;
    } else if (is_awake && vcc < POWER_ON_VOLTAGE_THRESHOLD) {
      is_awake = false;
    }

    if (!is_awake) {
      if(vcc > LOW_BATTERY_THRESHOLD + 10 && lowbat_flag == true) {
        eeprom_write_byte((uint8_t*)EEPROM_ADDR_LOWBAT, 0x00);
        lowbat_flag = false;
      }
      if (vcc < LOW_BATTERY_THRESHOLD) {
        eeprom_write_byte((uint8_t*)EEPROM_ADDR_LOWBAT, 0xA5);
        lowbat_flag = true;
      }
      enter_sleep();
    } else {
      if (!(PINB & _BV(BUS_RD_PIN))) {
        handle_bus_read();
      }
      if (!(PINB & _BV(BUS_WR_PIN))) {
        handle_bus_write();
      }
    }
  }
}