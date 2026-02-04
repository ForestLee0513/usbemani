#include "hardware/i2c.h"
#include <stdlib.h>

#ifndef AS5600_I2C_INST
#define AS5600_I2C_INST i2c0
#endif

#ifndef AS5600_I2C_SPEED
#define AS5600_I2C_SPEED 400000
#endif

#define AS5600_I2C_ADDRESS 0x36

#define AS5600_REG_ANGLE 0x0c

static const _pin_pair_t  _encoder_pins[ENCODERS_AVAILABLE] = { ENCODER_PINS };
static const uint8_t      _encoder_pidx[ENCODERS_ACTIVE]    = { ENCODER_CHANNELS };

static i2c_inst_t *const _encoder_buses[ENCODERS_ACTIVE] = { AS5600_I2C_INST };

repeating_timer_t _encoder_timer;
static int _prev_raw_angles[ENCODERS_ACTIVE] = {0};

static int as5600_read_reg16(uint8_t i, uint8_t reg) {
    uint8_t buf[2] = {reg, 0x00};
    if (i2c_write_blocking_until(_encoder_buses[i], AS5600_I2C_ADDRESS, buf, 1, true,
                             make_timeout_time_ms(1)) != 1) {
        return -1;
    }

    if (i2c_read_blocking_until(_encoder_buses[i], AS5600_I2C_ADDRESS, buf, 2, false,
                            make_timeout_time_ms(1)) != 2) {
        return -1;
    }

    return (buf[0] << 8) | buf[1];
}

bool as5600_is_present(uint8_t i) {
    uint8_t buf[1] = {0x0c};
    int ret = i2c_write_blocking_until(_encoder_buses[i], AS5600_I2C_ADDRESS, buf, 1, true,
                             make_timeout_time_ms(1));
    return ret == 1;
}

static inline void _encoder_update(void) {
    static const int DEADZONE = 4096 / (ENCODER_PPR * 4); // 루프 밖으로
    
    for (uint i = 0; i < ENCODERS_ACTIVE; i++) {
        if (_encoder[i].state.timeout)      _encoder[i].state.timeout--;
        if (_encoder[i].direction.timeout)  _encoder[i].direction.timeout--;
        else _encoder[i].direction.current = 0;

        int raw_angle = as5600_read_reg16(i, AS5600_REG_ANGLE);
        if (raw_angle == -1) continue;

        int delta = raw_angle - _prev_raw_angles[i];
        if (delta > 2048) delta -= 4096;
        else if (delta < -2048) delta += 4096;

        if (abs(delta) < DEADZONE) continue;

        uint8_t result = (delta < 0) ? ENCODER_CCW : ENCODER_CW;

        if (result & ENCODER_CCW) {
            _encoder[i].position.logical_raw -= ENCODER_LOGICAL_DELTA;
            _encoder[i].direction.delta--;
            
            if (_encoder[i].direction.delta <= (ENCODER_DIRECTION_THRESHOLD * -1)) {
                _encoder[i].direction.delta   = 0;
                _encoder[i].direction.current = ENCODER_CCW;
                _encoder[i].direction.timeout = ENCODER_TIMEOUT;
            }
            
            uint16_t physical = _encoder[i].position.physical;
            physical--;
            if (physical >= ENCODER_STEPS) physical = ENCODER_STEPS - 1;
            _encoder[i].position.physical = physical;
            
        } else {
            _encoder[i].position.logical_raw += ENCODER_LOGICAL_DELTA;
            _encoder[i].direction.delta++;
            
            if (_encoder[i].direction.delta >= ENCODER_DIRECTION_THRESHOLD) {
                _encoder[i].direction.delta   = 0;
                _encoder[i].direction.current = ENCODER_CW;
                _encoder[i].direction.timeout = ENCODER_TIMEOUT;
            }
            
            uint16_t physical = _encoder[i].position.physical;
            physical++;
            if (physical >= ENCODER_STEPS) physical = 0;
            _encoder[i].position.physical = physical;
        }

        _prev_raw_angles[i] = raw_angle;
    }
}

bool _encoder_interrupt(repeating_timer_t *rt) {
    _encoder_update();
    return true;
}

void _impl_encoder_init(void) {
    static const uint poll = ENCODER_FREQUENCY;

    for (uint i = 0; i < ENCODERS_ACTIVE; i++) {
        i2c_inst_t *i2c_port = _encoder_buses[i];

        const _pin_t sda_pin = _encoder_pins[_encoder_pidx[i]].a;
        const _pin_t scl_pin = _encoder_pins[_encoder_pidx[i]].b;

        i2c_init(i2c_port, AS5600_I2C_SPEED);

        gpio_init(sda_pin);
        gpio_init(scl_pin);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);

        int init_angle = as5600_read_reg16(i, AS5600_REG_ANGLE);
        if(init_angle != -1) _prev_raw_angles[i] = init_angle;
    }

    alarm_pool_add_repeating_timer_us(
        _impl_arch_alarmPool(),
        poll,
        _encoder_interrupt,
        NULL,
        &_encoder_timer
    );
}