#pragma once
#include "AP_EZKontrol_config.h"

#if AP_EZKONTROL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

class AP_EZKontrol_Driver : public CANSensor, public AP_ESC_Telem_Backend {
public:
    AP_EZKontrol_Driver();
    void update();

private:
    void handle_frame(AP_HAL::CANFrame &frame) override;
    void send_handshake();
    void send_command();

    bool _handshake_done;
    uint8_t _live_counter;
    uint32_t _last_ms;
};

class AP_EZKontrol {
public:
    AP_EZKontrol();
    void init();
    void update();

    static AP_EZKontrol *get_singleton() { return _singleton; }
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 esc1_addr;
    AP_Int8 esc2_addr;
    AP_Int8 vcu_addr;
    AP_Int16 target_phase_current;
    AP_Int8 control_mode;

private:
    static AP_EZKontrol *_singleton;
    AP_EZKontrol_Driver *_driver;
};

namespace AP {
    AP_EZKontrol *ezkontrol();
}

#endif // AP_EZKONTROL_ENABLED
