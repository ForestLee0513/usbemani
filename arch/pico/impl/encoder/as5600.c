
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// AS5600 I2C Address
#define AS5600_ADDR 0x36

// AS5600 Register Addresses
#define AS5600_RAW_ANGLE_H  0x0C
#define AS5600_RAW_ANGLE_L  0x0D
#define AS5600_ANGLE_H      0x0E
#define AS5600_ANGLE_L      0x0F
#define AS5600_STATUS       0x0B
#define AS5600_AGC          0x1A
#define AS5600_MAGNITUDE_H  0x1B
#define AS5600_MAGNITUDE_L  0x1C

// AS5600 native resolution (12-bit)
#define AS5600_RESOLUTION 4096

// Pin configuration - reuse ENCODER_PINS as I2C pins (SDA, SCL)
static const _pin_pair_t _encoder_pins[ENCODERS_AVAILABLE] = { ENCODER_PINS };
static const uint8_t     _encoder_pidx[ENCODERS_ACTIVE]    = { ENCODER_CHANNELS };

// I2C instance selection
#ifndef ENCODER_I2C_INST
#define ENCODER_I2C_INST i2c0
#endif

#ifndef ENCODER_I2C_SPEED
#define ENCODER_I2C_SPEED 400000
#endif

// Support for multiple I2C addresses if needed
#ifdef ENCODER_I2C_ADDRESSES
static const uint8_t _encoder_i2c_addrs[ENCODERS_ACTIVE] = { ENCODER_I2C_ADDRESSES };
#endif

static i2c_inst_t* _encoder_i2c = ENCODER_I2C_INST;

// Our stored repeating timer
repeating_timer_t _encoder_timer;

// Previous raw AS5600 readings for delta calculation
static uint16_t _as5600_prev_raw[ENCODERS_ACTIVE] = {0};

// Direction multiplier for reversed channels
#if defined(ENCODER_CHANNELS_REVERSED)
const int8_t _encoder_direction_multiplier = -1;
#else
const int8_t _encoder_direction_multiplier = 1;
#endif

// Read 16-bit value from AS5600
static inline uint16_t _as5600_read_angle(uint8_t encoder_idx) {
  uint8_t data[2];
  uint8_t reg = AS5600_ANGLE_H;
  
  // Use custom address if defined, otherwise default
  uint8_t addr = AS5600_ADDR;
  #ifdef ENCODER_I2C_ADDRESSES
  addr = _encoder_i2c_addrs[encoder_idx];
  #endif
  
  i2c_write_blocking(_encoder_i2c, addr, &reg, 1, true);
  i2c_read_blocking(_encoder_i2c, addr, data, 2, false);
  
  return ((uint16_t)data[0] << 8) | data[1];
}

// Convert AS5600 raw value (0-4095) to target PPR with quadrature stepping
// Uses ENCODER_STEPS from config (ENCODER_PPR * 4)
// Example: ENCODER_PPR=600 -> ENCODER_STEPS=2400
static inline uint16_t _as5600_to_physical(uint16_t raw_value) {
  // raw_value: 0-4095 (AS5600 12-bit)
  // target: 0-(ENCODER_STEPS-1) where ENCODER_STEPS = ENCODER_PPR << 2
  // This converts AS5600's absolute position to quadrature-equivalent steps
  uint32_t scaled = ((uint32_t)raw_value * ENCODER_STEPS) / AS5600_RESOLUTION;
  return (uint16_t)(scaled % ENCODER_STEPS);
}

// Calculate delta between two AS5600 readings with wrap-around handling
static inline int16_t _as5600_calculate_raw_delta(uint16_t current, uint16_t previous) {
  int16_t delta = current - previous;
  
  // Handle wrap-around at 12-bit boundary (0-4095)
  if (delta > 2048) {
    delta -= 4096;  // Wrapped backwards
  } else if (delta < -2048) {
    delta += 4096;  // Wrapped forwards
  }
  
  return delta;
}

// Updates all encoders
static inline void _encoder_update(void) {
  for (uint i = 0; i < ENCODERS_ACTIVE; i++) {
    //// Part 1: State Updates
    // Decrement timeouts. Clear the currently-set direction if this timeout is depleted.
    if (_encoder[i].state.timeout)      _encoder[i].state.timeout--;
    if (_encoder[i].direction.timeout)  _encoder[i].direction.timeout--;
    else _encoder[i].direction.current = 0;

    // Read current angle from AS5600 (0-4095, 12-bit)
    uint16_t raw_angle = _as5600_read_angle(i) & 0x0FFF;
    uint16_t previous_raw = _as5600_prev_raw[i];
    
    // Calculate raw delta (in AS5600 native 12-bit units)
    int16_t raw_delta = _as5600_calculate_raw_delta(raw_angle, previous_raw);

    raw_delta *= _encoder_direction_multiplier;

    // If no movement, continue to next encoder
    if (raw_delta == 0) continue;
    
    // Store current raw value for next iteration
    _as5600_prev_raw[i] = raw_angle;
    
    //// Part 2: Value Updates
    // Convert current raw angle to physical position (scaled to ENCODER_STEPS)
    uint16_t physical_position = _as5600_to_physical(raw_angle);
    _encoder[i].position.physical = physical_position;
    
    // Calculate scaled delta for logical position
    // Scale raw_delta from AS5600 units (0-4095) to ENCODER_STEPS units
    // ENCODER_STEPS = ENCODER_PPR << 2 (quadrature 4x multiplication)
    // Then apply ENCODER_LOGICAL_DELTA which handles the game-specific scaling
    int32_t scaled_delta = ((int32_t)raw_delta * ENCODER_STEPS) / AS5600_RESOLUTION;
    int32_t logical_delta = (scaled_delta * ENCODER_LOGICAL_DELTA) / 4;
    _encoder[i].position.logical_raw += logical_delta;
    
    // Update direction tracking using scaled delta
    _encoder[i].direction.delta += scaled_delta;
    
    // Check for direction threshold (from ENCODER_DIRECTION_THRESHOLD)
    if (_encoder[i].direction.delta <= (ENCODER_DIRECTION_THRESHOLD * -1)) {
      _encoder[i].direction.delta   = 0;
      _encoder[i].direction.current = ENCODER_CCW;
      _encoder[i].direction.timeout = ENCODER_TIMEOUT;
    } else if (_encoder[i].direction.delta >= ENCODER_DIRECTION_THRESHOLD) {
      _encoder[i].direction.delta   = 0;
      _encoder[i].direction.current = ENCODER_CW;
      _encoder[i].direction.timeout = ENCODER_TIMEOUT;
    }
  }
}

// A wrapper for our encoder interrupt
bool _encoder_interrupt(repeating_timer_t *rt) {
  _encoder_update();
  return true;
}

// Initialize the encoder subsystem
void _impl_encoder_init(void) {
  // Use ENCODER_FREQUENCY from config for polling rate
  static const uint poll = (1000000 / ENCODER_FREQUENCY);

  // Initialize I2C for all active encoders
  for (uint i = 0; i < ENCODERS_ACTIVE; i++) {
    const _pin_t sda_pin = _encoder_pins[_encoder_pidx[i]].a;  // Each encoder's SDA
    const _pin_t scl_pin = _encoder_pins[_encoder_pidx[i]].b;  // Each encoder's SCL
    
    // Initialize I2C bus with these pins
    // Note: If multiple encoders share the same I2C pins, this will be called
    // multiple times on the same bus, but this is safe (just redundant)
    i2c_init(_encoder_i2c, ENCODER_I2C_SPEED);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
  }

  // Initialize each encoder's position
  for (uint i = 0; i < ENCODERS_ACTIVE; i++) {
    uint16_t raw_angle = _as5600_read_angle(i) & 0x0FFF;
    
    // Store initial raw value
    _as5600_prev_raw[i] = raw_angle;
    
    // Convert to physical position (scaled to ENCODER_STEPS)
    _encoder[i].position.physical = _as5600_to_physical(raw_angle);
    
    // Set logical position to center (uses ENCODER_LOGICAL_MAX from config)
    _encoder[i].position.logical = (ENCODER_LOGICAL_MAX / 2);
  }

  alarm_pool_add_repeating_timer_us(
    _impl_arch_alarmPool(),
    poll,
    _encoder_interrupt,
    NULL,
    &_encoder_timer
  );
}