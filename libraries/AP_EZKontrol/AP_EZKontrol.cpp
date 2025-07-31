#include "AP_EZKontrol.h"

#if AP_EZKONTROL_ENABLED

#include <GCS_MAVLink/GCS.h>

AP_EZKontrol *AP_EZKontrol::_singleton;

const AP_Param::GroupInfo AP_EZKontrol::var_info[] = {
    // @Param: ESC1_ADDR
    // @DisplayName: Address for ESC1
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("ESC1_ADDR", 1, AP_EZKontrol, esc1_addr, 2),

    // @Param: ESC2_ADDR
    // @DisplayName: Address for ESC2
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("ESC2_ADDR", 2, AP_EZKontrol, esc2_addr, 3),

    // @Param: VCU_ADDR
    // @DisplayName: Vehicle controller address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("VCU_ADDR", 3, AP_EZKontrol, vcu_addr, 239),

    // @Param: TARGET_I
    // @DisplayName: Target phase current (0.1A)
    // @Range: -32000 32000
    // @User: Advanced
    AP_GROUPINFO("TARGET_I", 4, AP_EZKontrol, target_phase_current, 0),

    // @Param: MODE
    // @DisplayName: Control mode
    // @Values: 0:Torque,1:Speed
    // @User: Advanced
    AP_GROUPINFO("MODE", 5, AP_EZKontrol, control_mode, 0),

    AP_GROUPEND
};

AP_EZKontrol::AP_EZKontrol()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton == nullptr) {
        _singleton = this;
    }
    _driver = nullptr;
}

void AP_EZKontrol::init()
{
    if (_driver != nullptr) {
        return;
    }
    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::EZKontrol) {
            _driver = NEW_NOTHROW AP_EZKontrol_Driver();
            return;
        }
    }
}

void AP_EZKontrol::update()
{
    if (_driver != nullptr) {
        _driver->update();
    }
}

AP_EZKontrol_Driver::AP_EZKontrol_Driver() : CANSensor("EZKontrol")
{
    register_driver(AP_CAN::Protocol::EZKontrol);
    _handshake_done = false;
    _live_counter = 0;
    _last_ms = 0;
}

void AP_EZKontrol_Driver::update()
{
    const uint32_t now = AP_HAL::millis();
    if (!_handshake_done) {
        if (now - _last_ms >= 50) {
            send_handshake();
            _last_ms = now;
        }
    } else {
        if (now - _last_ms >= 50) {
            send_command();
            _last_ms = now;
        }
    }
}

void AP_EZKontrol_Driver::send_handshake()
{
    uint8_t buf[8];
    memset(buf, 0x55, sizeof(buf));
    AP_HAL::CANFrame frame(0x1801D0EF | AP_HAL::CANFrame::FlagEFF, buf, 8, false);
    write_frame(frame, 1000);
}

void AP_EZKontrol_Driver::send_command()
{
    const AP_EZKontrol *inst = AP_EZKontrol::get_singleton();
    if (inst == nullptr) {
        return;
    }
    uint8_t buf[8] = {};
    int16_t current = inst->target_phase_current;
    buf[0] = uint8_t(current);
    buf[1] = uint8_t(current >> 8);
    buf[4] = inst->control_mode;
    buf[7] = _live_counter++;
    AP_HAL::CANFrame frame(0x0C01EFD0 | AP_HAL::CANFrame::FlagEFF, buf, 8, false);
    write_frame(frame, 1000);
}

void AP_EZKontrol_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }
    if (frame.id == 0x0C01EFD0 && frame.dlc == 8 && frame.data[0] == 0xAA) {
        _handshake_done = true;
    }
    // telemetry frames could be handled here
}

namespace AP {
AP_EZKontrol *ezkontrol()
{
    return AP_EZKontrol::get_singleton();
}
}

#endif // AP_EZKONTROL_ENABLED
