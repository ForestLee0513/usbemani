#include "hardware/i2c.h"
#include <stdlib.h> // abs() 함수 사용을 위해 추가

// --- 1. 기본값 정의 ---
#ifndef ENCODER_I2C_INST
#define ENCODER_I2C_INST i2c0
#endif

#ifndef ENCODER_I2C_SPEED
#define ENCODER_I2C_SPEED 400000
#endif

#ifndef ENCODER_I2C_ADDRESSES
#define ENCODER_I2C_ADDRESSES 0x36
#endif

#define AS5600_REG_ANGLE 0x0c

static const _pin_pair_t  _encoder_pins[ENCODERS_AVAILABLE] = { ENCODER_PINS };
static const uint8_t      _encoder_pidx[ENCODERS_ACTIVE]    = { ENCODER_CHANNELS };

static i2c_inst_t *const _encoder_buses[ENCODERS_ACTIVE] = { ENCODER_I2C_INST };
static const uint8_t     _encoder_addrs[ENCODERS_ACTIVE] = { ENCODER_I2C_ADDRESSES };

repeating_timer_t _encoder_timer;
static int _prev_raw_angles[ENCODERS_ACTIVE] = {0};

// --- 유틸리티 함수 ---

static int as5600_read_reg16(uint8_t i, uint8_t reg) {
    uint8_t buf[2] = {reg, 0x00};
    if (i2c_write_blocking_until(_encoder_buses[i], _encoder_addrs[i], buf, 1, true,
                             make_timeout_time_ms(1)) != 1) {
        return -1;
    }

    if (i2c_read_blocking_until(_encoder_buses[i], _encoder_addrs[i], buf, 2, false,
                            make_timeout_time_ms(1)) != 2) {
        return -1;
    }

    return (buf[0] << 8) | buf[1];
}

bool as5600_is_present(uint8_t i) {
    uint8_t buf[1] = {0x0c};
    int ret = i2c_write_blocking_until(_encoder_buses[i], _encoder_addrs[i], buf, 1, true,
                             make_timeout_time_ms(1));
    return ret == 1;
}

// --- 핵심 업데이트 로직 ---

static inline void _encoder_update(void) {
    for (uint i = 0; i < ENCODERS_ACTIVE; i++) {
        // 타이머 및 상태 관리
        if (_encoder[i].state.timeout)      _encoder[i].state.timeout--;
        if (_encoder[i].direction.timeout)  _encoder[i].direction.timeout--;
        else _encoder[i].direction.current = 0;

        int raw_angle = as5600_read_reg16(i, AS5600_REG_ANGLE);
        if (raw_angle == -1) continue;

        // 회전 변위(Delta) 계산 및 오버플로우 처리
        int delta = raw_angle - _prev_raw_angles[i];
        if (delta > 2048) delta -= 4096;
        else if (delta < -2048) delta += 4096;

        /* * [민감도 조절의 핵심]
         * 4096 해상도를 36 PPR로 나누면 한 칸당 약 113.7 유닛입니다.
         * 노이즈로 인한 떨림을 방지하기 위해 한 칸의 1/4 정도(약 28 유닛)
         * 이하의 움직임은 무시합니다.
         */
        const int deadzone = 4096 / (ENCODER_PPR * 4); 

        if (abs(delta) < deadzone) continue; 

        // 유효한 움직임이 있을 때만 처리
        if (delta < 0) {
            _encoder[i].position.logical_raw -= ENCODER_LOGICAL_DELTA;
            
            _encoder[i].direction.delta--;
            if (_encoder[i].direction.delta <= (ENCODER_DIRECTION_THRESHOLD * -1)) {
                _encoder[i].direction.delta   = 0;
                _encoder[i].direction.current = ENCODER_CCW;
                _encoder[i].direction.timeout = ENCODER_TIMEOUT;
            }
        } else {
            _encoder[i].position.logical_raw += ENCODER_LOGICAL_DELTA;
            
            _encoder[i].direction.delta++;
            if (_encoder[i].direction.delta >= ENCODER_DIRECTION_THRESHOLD) {
                _encoder[i].direction.delta   = 0;
                _encoder[i].direction.current = ENCODER_CW;
                _encoder[i].direction.timeout = ENCODER_TIMEOUT;
            }
        }

        // 물리적 위치 업데이트 및 이전 각도 갱신
        _encoder[i].position.physical = (raw_angle * ENCODER_PPR) / 4096;
        _prev_raw_angles[i] = raw_angle; 
    }
}

bool _encoder_interrupt(repeating_timer_t *rt) {
    _encoder_update();
    return true;
}

void _impl_encoder_init(void) {
    // ENCODER_FREQUENCY가 너무 크면(16000) 반응이 느리므로, 
    // 가급적 1000~2000(1~2ms)으로 설정하는 것을 권장합니다.
    static const uint poll = ENCODER_FREQUENCY;

    for (uint i = 0; i < ENCODERS_ACTIVE; i++) {
        i2c_inst_t *i2c_port = _encoder_buses[i];

        const _pin_t sda_pin = _encoder_pins[_encoder_pidx[i]].a;
        const _pin_t scl_pin = _encoder_pins[_encoder_pidx[i]].b;

        i2c_init(i2c_port, ENCODER_I2C_SPEED);

        gpio_init(sda_pin);
        gpio_init(scl_pin);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);

        if(as5600_is_present(i)) {
            // 초기 연결 성공 시 온보드 LED 등을 켜서 확인 가능
            // gpio_put(25, 1); 
        }
        
        // 초기 각도 저장 (부팅 시 튐 방지)
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