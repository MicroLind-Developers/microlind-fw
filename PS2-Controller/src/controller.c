// ATmega88 Microlind Bus RTC + PS/2 Interface Skeleton

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

// -------------------- Config --------------------
#define FIFO_SIZE                       64      // 64 bytes fifo bufferfor keyboard and mouse data

// EEPROM addresses
#define EEPROM_ADDR_LOWBAT              0x10    // Low battery flag
#define EEPROM_ADDR_EEPROM_ERROR_FLAG   0x11    // EEPROM error flag
#define EEPROM_ADDR_TX_ACK_FAIL         0x12    // PS/2 TX ACK fail flag
#define EEPROM_ADDR_RTC_BASE            0x20    // RTC data base address

// RTC data storage 
#define EEPROM_RTC_SLOTS                40      // 40 slots to store rtc data
#define EEPROM_RTC_SIZE                 8       // 7 bytes data + 1 checksum

// Voltage thresholds for detecting low battery and power on
#define LOW_BATTERY_THRESHOLD           2200    // millivolts after the diode voltage drop, so 2.8 ish at the cell.
#define POWER_ON_VOLTAGE_THRESHOLD      3700    // Above this we assume the VCC from computer is OK.

// Pin assignments
#define BUS_EN_PIN                      PB0     // Enable input
#define BUS_RD_PIN                      PB3     // Read strobe
#define BUS_WR_PIN                      PB4     // Write strobe
#define BUS_IRQ_KB                      PB1     // Keyboard IRQ output
#define BUS_IRQ_MS                      PB2     // Mouse IRQ output
#define BUS_ADDR_A0                     PC0     // Address line A0
#define BUS_ADDR_A1                     PC1     // Address line A1
#define BUS_ADDR_A2                     PC2     // Address line A2
#define BUS_DATA_PORT                   PORTD   // Data port
#define BUS_DATA_DDR                    DDRD    // Data direction register
#define BUS_DATA_PIN                    PIND    // Data input register

#define PS2_KEYBOARD_CLK                PC3     // PS/2 keyboard clock pin  
#define PS2_KEYBOARD_DATA               PC4     // PS/2 keyboard data pin
#define PS2_MOUSE_CLK                   PC5     // PS/2 mouse clock pin
#define PS2_MOUSE_DATA                  PC6     // PS/2 mouse data pin

#ifdef SERIAL_DEBUG
// Serial debug configuration
#define DEBUG_PIN                       PB5     // Debug pin
#define DEBUG_BAUD_RATE                 115200
#define DEBUG_BIT_TIME_US               (1000000UL / DEBUG_BAUD_RATE)  // ~104.17 us per bit
#else
#define AWAKE_LED_PIN                   PB5     // LED on when controller is awake (active HIGH)
#endif  
// -------------------- Globals --------------------
typedef struct {
  uint8_t buffer[FIFO_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
} FIFO;

typedef struct {
  uint8_t year;  // 1970 = 0, 2025 = 55
  uint8_t month; // 1 = January, 12 = December
  uint8_t day;   // 1 = First day of the month, 28,29,30 or 31 = Last day of the month
  uint8_t dow;  // 0 = Monday, 6 = Sunday
  uint8_t hour; // 0 = 00, 23 = 23
  uint8_t minute; // 0 = 00, 59 = 59
  uint8_t second; // 0 = 00, 60 = 60  (if leap second is needed, it will be 60)
} RTC_DATA;

FIFO kb_fifo = {{0}, 0, 0};
FIFO ms_fifo = {{0}, 0, 0};
FIFO tx_fifo = {{0}, 0, 0}; // PS/2 TX buffer for keyboard

volatile RTC_DATA rtc_data = {55, 1, 1, 0, 0, 0, 0};
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
void fifo_push(FIFO* fifo_buffer, uint8_t val);
bool fifo_pop(FIFO* fifo_buffer, uint8_t* val);
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
void set_kb_irq(void);
void set_ms_irq(void);
void clear_kb_irq(void);
void clear_ms_irq(void);

#ifdef SERIAL_DEBUG
void debug_serial_init();
void debug_serial_send_byte(uint8_t byte);
void debug_serial_send_string(const char* str);
void debug_serial_send_hex(uint8_t byte);
#endif
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
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 7; i++) sum += data[i];
  return ~(uint8_t)(sum & 0xFF) + 1;
}

void save_rtc_to_eeprom() {
  uint8_t data[8] = { rtc_data.year, rtc_data.month, rtc_data.day, rtc_data.dow, rtc_data.hour, rtc_data.minute, rtc_data.second , 0};
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
      rtc_data.year = data[0];
      rtc_data.month = data[1];
      rtc_data.day = data[2];
      rtc_data.dow = data[3];
      rtc_data.hour = data[4];
      rtc_data.minute = data[5];
      rtc_data.second = data[6];
      rtc_save_index = (i + 1) % EEPROM_RTC_SLOTS;
      rtc_eeprom_error = false;
      return;
    }
    else {
      eeprom_write_byte((uint8_t*)EEPROM_ADDR_EEPROM_ERROR_FLAG, 0xA5);
    }
  }
  // No valid record found; use default time
  rtc_data.year = 55;
  rtc_data.month = 1;
  rtc_data.day = 1;
  rtc_data.dow = 0;
  rtc_data.hour = 0;
  rtc_data.minute = 0;
  rtc_data.second = 0;
  rtc_save_index = 0;
}

void rtc_tick() {
  if (++rtc_data.second >= 60) {
    rtc_data.second = 0;
    if (++rtc_data.minute >= 60) {
      rtc_data.minute = 0;
      if (++rtc_data.hour >= 24) {
        rtc_data.hour = 0;
        if (++rtc_data.day > days_in_month(rtc_data.month, rtc_data.year)) {
          rtc_data.day = 1;
          if (++rtc_data.month > 12) {
            rtc_data.month = 1;
            rtc_data.year++;
          }
        }
        if (++rtc_data.dow > 6) rtc_data.dow = 0;
      }
    }
    // Save every minute.
    save_rtc_to_eeprom();
  }
}

void rtc_timer_init() {
  TCCR2A = 0x00;
  TCCR2B = (1 << CS22) | (1 << CS20);  // prescaler 128
  TIMSK2 = (1 << TOIE2);               // enable overflow interrupt
  TCNT2 = 0;

  // TODO: Read the RTC data from EEPROM, find the newest one and set the rtc_save_index to the index of the newest one.
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
    case 0: if (!fifo_pop(&kb_fifo, &out)) clear_kb_irq(); break;
    case 1: if (!fifo_pop(&ms_fifo, &out)) clear_ms_irq(); break;
    case 2: out = (rtc_data.year & 0x7F); break;
    case 3: out = (rtc_data.month & 0x0F) | (lowbat_flag << 7) | (rtc_eeprom_error << 6) | (ps2_ack_error << 5); clear_errors(); break;
    case 4: out = (rtc_data.day & 0x1F) | ((rtc_data.dow & 0x07) << 5); break;
    case 5: out = rtc_data.hour; break;
    case 6: out = rtc_data.minute; break;
    case 7: out = rtc_data.second; break;
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
    case 2: rtc_data.year = val & 0x7F; break;
    case 3: rtc_data.month = val & 0x7F; break;
    case 4:
      rtc_data.day = val & 0x1F;
      rtc_data.dow = (val >> 5) & 0x07;
      break;
    case 5: rtc_data.hour = val; break;
    case 6: rtc_data.minute = val; break;
    case 7: rtc_data.second = val; break;
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

#ifdef SERIAL_DEBUG
// -------------------- Debug Serial Output --------------------
void debug_serial_init() {
  DDRB |= _BV(DEBUG_PIN);  // Set DEBUG_PIN as output
  PORTB |= _BV(DEBUG_PIN); // Start with pin high (idle state)
}

void debug_serial_send_byte(uint8_t byte) {
 cli(); // Disable interrupts for precise timing
 
 // Start bit (low)
 PORTB &= ~_BV(DEBUG_PIN);
 _delay_us(DEBUG_BIT_TIME_US);
 
 // Data bits (LSB first)
 for (uint8_t i = 0; i < 8; i++) {
   if (byte & (1 << i)) {
     PORTB |= _BV(DEBUG_PIN);
   } else {
     PORTB &= ~_BV(DEBUG_PIN);
   }
   _delay_us(DEBUG_BIT_TIME_US);
 }
 
 // Stop bit (high)
 PORTB |= _BV(DEBUG_PIN);
 _delay_us(DEBUG_BIT_TIME_US);
 
 sei(); // Re-enable interrupts
}

void debug_serial_send_string(const char* str) {
 while (*str) {
   debug_serial_send_byte(*str++);
 }
}

void debug_serial_send_hex(uint8_t byte) {
 // Convert byte to hex string and send
 char hex_str[3];
 hex_str[0] = "0123456789ABCDEF"[byte >> 4];
 hex_str[1] = "0123456789ABCDEF"[byte & 0x0F];
 hex_str[2] = '\0';
 debug_serial_send_string(hex_str);
}
#endif

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
#if SERIAL_DEBUG
      // Output keyboard data to debug serial
      debug_serial_send_string("KB:");
      debug_serial_send_hex(ps2_kb_byte);
      debug_serial_send_string("\r\n");
#endif
      ps2_kb_bitcount = 0;
      ps2_kb_byte = 0;
      set_kb_irq();
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
#if SERIAL_DEBUG
      // Output mouse data to debug serial
      debug_serial_send_string("MS:");
      debug_serial_send_hex(ps2_ms_byte);
      debug_serial_send_string("\r\n");
#endif
      ps2_ms_bitcount = 0;
      ps2_ms_byte = 0;
      set_ms_irq();
    }
  }
}

void set_kb_irq() {
  cli();
  PORTB |= _BV(BUS_IRQ_KB);
  sei();
}

void set_ms_irq() {
  cli();
  PORTB |= _BV(BUS_IRQ_MS);
  sei();
}

void clear_kb_irq() {
  cli();
  PORTB &= ~_BV(BUS_IRQ_KB);
  sei();
}

void clear_ms_irq() {
  cli();
  PORTB &= ~_BV(BUS_IRQ_MS);
  sei();
}

// -------------------- Main --------------------
int main() {
    cli();
    
    // Initialize ADC for VCC measurement
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // Enable ADC, prescaler = 128
    
    // Set up Awake LED
#if defined(SERIAL_DEBUG)
    debug_serial_init(); // Initialize debug serial output
#else
    DDRB  |= _BV(AWAKE_LED_PIN) | _BV(BUS_IRQ_KB) | _BV(BUS_IRQ_MS);   // LED pin as output
    PORTB &= ~(_BV(AWAKE_LED_PIN) | ~_BV(BUS_IRQ_KB) | ~_BV(BUS_IRQ_MS));  // start OFF

    PORTB |= _BV(AWAKE_LED_PIN) | ~_BV(BUS_IRQ_KB) | ~_BV(BUS_IRQ_MS);  // Enable LED

    _delay_ms(250);

    PORTB &= ~(_BV(AWAKE_LED_PIN) | ~_BV(BUS_IRQ_KB) | ~_BV(BUS_IRQ_MS));  // Disable LED

#endif

    rtc_timer_init();
    PCICR |= _BV(PCIE1);
    PCMSK1 |= _BV(PCINT11) | _BV(PCINT13); // Enable PS/2 keyboard and mouse clock pin interrupts
    sei();

//  // Send startup message
#ifdef SERIAL_DEBUG
    debug_serial_send_string("PS/2 Controller Ready\r\n");
#endif

  while (1) {
    uint16_t vcc = read_vcc_mv();
    if (!is_awake && vcc >= POWER_ON_VOLTAGE_THRESHOLD) {
          is_awake = true;
#ifndef SERIAL_DEBUG
          PORTB |= (_BV(AWAKE_LED_PIN) | _BV(BUS_IRQ_KB) | _BV(BUS_IRQ_MS));   // LED ON
#endif
    } else if (is_awake && vcc < POWER_ON_VOLTAGE_THRESHOLD) {
          is_awake = false;
#ifndef SERIAL_DEBUG
          PORTB &= ~(_BV(AWAKE_LED_PIN) | _BV(BUS_IRQ_KB) | _BV(BUS_IRQ_MS));  // LED OFF
#endif
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
