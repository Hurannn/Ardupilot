#pragma once

#ifndef AP_EZKONTROL_H
#define AP_EZKONTROL_H

#include "AP_EZKontrol_config.h"

#if AP_EZKONTROL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_Param/AP_Param.h>

class AP_EZKontrol_Driver : public CANSensor
#if HAL_WITH_ESC_TELEM
    , public AP_ESC_Telem_Backend
#endif
{
public:
    AP_EZKontrol_Driver();

    void update();

    void set_addresses(uint8_t esc1, uint8_t esc2, uint8_t vcu);

    void set_target(uint8_t idx, int16_t current, int16_t speed);

    void set_control_mode(uint8_t m) { _control_mode = m; }

private:
    void handle_frame(AP_HAL::CANFrame &frame) override;

    void send_handshake(uint8_t idx);
    void send_command(uint8_t idx);

    uint8_t _esc_addr[2]{};
    uint8_t _vcu_addr{0};
    bool _handshake_done[2]{};
    uint8_t _life[2]{};
    uint32_t _last_tx_ms{0};

    int16_t _target_current[2]{};
    int16_t _target_speed[2]{};
    uint8_t _control_mode{0};
};

class AP_EZKontrol {
public:
    AP_EZKontrol();
    void init();
    void update();
	
	static const AP_Param::GroupInfo var_info[];

    static AP_EZKontrol *get_singleton() { return _singleton; }

private:
    static AP_EZKontrol *_singleton;
    AP_EZKontrol_Driver *_driver = nullptr;
	
	 // addresses of ESC and VCU
    AP_Int8 esc1_addr;
    AP_Int8 esc2_addr;
    AP_Int8 vcu_addr;
    // target phase current (0.1 A units)
    AP_Int16 target_phase_cur;
    // command mode (0=Torque, 1=Speed)
    AP_Int8 cmd_mode;
};

namespace AP {
    AP_EZKontrol *ezkontrol();
};

#endif // AP_EZKONTROL_ENABLED

#endif // AP_EZKONTROL_H
