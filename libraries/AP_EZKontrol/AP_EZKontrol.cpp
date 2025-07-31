#include "AP_EZKontrol.h"

#if AP_EZKONTROL_ENABLED
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_EZKontrol::var_info[] = {
    // @Param: ESC1_ADDR
    // @DisplayName: ESC1 CAN address
    // @Description: CAN address of ESC1
    // @User: Advanced
    AP_GROUPINFO("ESC1_ADDR", 1, AP_EZKontrol, esc1_addr, 0xEF),

    // @Param: ESC2_ADDR
    // @DisplayName: ESC2 CAN address
    // @Description: CAN address of ESC2
    // @User: Advanced
    AP_GROUPINFO("ESC2_ADDR", 2, AP_EZKontrol, esc2_addr, 0xF0),

    // @Param: VCU_ADDR
    // @DisplayName: VCU CAN address
    // @Description: CAN address of Vehicle Control Unit
    // @User: Advanced
    AP_GROUPINFO("VCU_ADDR", 3, AP_EZKontrol, vcu_addr, 0xD0),

    // @Param: TARGET_PHASE_CUR
    // @DisplayName: Target phase current
    // @Description: Target phase current in 0.1A units
    // @Units: 0.1A
    // @User: Advanced
    AP_GROUPINFO("TARGET_PHASE_CUR", 4, AP_EZKontrol, target_phase_cur, 0),

    // @Param: CMD_MODE
    // @DisplayName: Command mode
    // @Description: 0=Torque, 1=Speed
    // @Values: 0:Torque,1:Speed
    // @User: Advanced
    AP_GROUPINFO("CMD_MODE", 5, AP_EZKontrol, cmd_mode, 0),

    AP_GROUPEND
};

static inline uint32_t make_vcu_tx_id(uint8_t esc_addr, uint8_t vcu_addr)
{
    return (0x0CU << 24) | (0x01U << 16) | (uint32_t(esc_addr) << 8) | vcu_addr;
}

static inline uint32_t make_mcu_tx1_id(uint8_t esc_addr, uint8_t vcu_addr)
{
    return (0x18U << 24) | (0x01U << 16) | (uint32_t(vcu_addr) << 8) | esc_addr;
}

static inline uint32_t make_mcu_tx2_id(uint8_t esc_addr, uint8_t vcu_addr)
{
    return (0x18U << 24) | (0x02U << 16) | (uint32_t(vcu_addr) << 8) | esc_addr;
}

AP_EZKontrol_Driver::AP_EZKontrol_Driver() : CANSensor("EZKontrol")
{
    register_driver(AP_CAN::Protocol::EZKontrol);
}

void AP_EZKontrol_Driver::set_addresses(uint8_t esc1, uint8_t esc2, uint8_t vcu)
{
    _esc_addr[0] = esc1;
    _esc_addr[1] = esc2;
    _vcu_addr = vcu;
}

void AP_EZKontrol_Driver::set_target(uint8_t idx, int16_t current, int16_t speed)
{
    if (idx < 2) {
        _target_current[idx] = current;
        _target_speed[idx] = speed;
    }
}

void AP_EZKontrol_Driver::send_handshake(uint8_t idx)
{
    if (idx >= 2) {
        return;
    }
    uint8_t buf[8];
    memset(buf, 0xAA, sizeof(buf));
    const uint32_t id = make_vcu_tx_id(_esc_addr[idx], _vcu_addr);
    AP_HAL::CANFrame frame(id | AP_HAL::CANFrame::FlagEFF, buf, sizeof(buf), false);
    write_frame(frame, 1000);
}

void AP_EZKontrol_Driver::send_command(uint8_t idx)
{
    if (idx >= 2) {
        return;
    }
    uint8_t buf[8];
    buf[0] = _target_current[idx] & 0xFF;
    buf[1] = (_target_current[idx] >> 8) & 0xFF;
    buf[2] = _target_speed[idx] & 0xFF;
    buf[3] = (_target_speed[idx] >> 8) & 0xFF;
    buf[4] = _control_mode;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = _life[idx]++;

    const uint32_t id = make_vcu_tx_id(_esc_addr[idx], _vcu_addr);
    AP_HAL::CANFrame frame(id | AP_HAL::CANFrame::FlagEFF, buf, sizeof(buf), false);
    write_frame(frame, 1000);
}

void AP_EZKontrol_Driver::update()
{
    const uint32_t now = AP_HAL::millis();
    for (uint8_t i = 0; i < 2; i++) {
        if (!_handshake_done[i]) {
            if (now - _last_tx_ms >= 50) {
                send_handshake(i);
            }
        } else if (now - _last_tx_ms >= 50) {
            send_command(i);
        }
    }
    if (now - _last_tx_ms >= 50) {
        _last_tx_ms = now;
    }
}

void AP_EZKontrol_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    for (uint8_t idx = 0; idx < 2; idx++) {
        if (id == make_mcu_tx1_id(_esc_addr[idx], _vcu_addr)) {
            if (frame.dlc == 8) {
                bool all55 = true;
                for (uint8_t i=0;i<8;i++) {
                    if (frame.data[i] != 0x55) { all55 = false; break; }
                }
                if (all55 && !_handshake_done[idx]) {
                    _handshake_done[idx] = true;
                    return;
                }

#if HAL_WITH_ESC_TELEM
                int16_t bus_v = le16toh_ptr(&frame.data[0]);
                int16_t bus_c = le16toh_ptr(&frame.data[2]);
                (void)le16toh_ptr(&frame.data[4]);
                int16_t speed = le16toh_ptr(&frame.data[6]);

                TelemetryData t{};
                t.voltage = float(bus_v) * 0.1f;
                t.current = float(bus_c) * 0.1f;
                update_rpm(idx, speed, 0);
                update_telem_data(idx, t,
                                  TelemetryType::CURRENT |
                                  TelemetryType::VOLTAGE);
#endif
            }
            return;
        } else if (id == make_mcu_tx2_id(_esc_addr[idx], _vcu_addr) && frame.dlc == 8) {
#if HAL_WITH_ESC_TELEM
            int8_t ctrl_temp = int8_t(frame.data[0]);
            int8_t motor_temp = int8_t(frame.data[1]);
            TelemetryData t{};
            t.temperature_cdeg = motor_temp * 100;
            t.motor_temp_cdeg = ctrl_temp * 100;
            update_telem_data(idx, t,
                              TelemetryType::TEMPERATURE |
                              TelemetryType::MOTOR_TEMPERATURE);
#endif
            return;
        }
    }
}

AP_EZKontrol::AP_EZKontrol()
{
	AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_EZKontrol must be singleton");
    }
#endif
    _singleton = this;
}

void AP_EZKontrol::init()
{
    if (_driver != nullptr) {
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::EZKontrol) {
            _driver = NEW_NOTHROW AP_EZKontrol_Driver();
            if (_driver != nullptr) {
                _driver->set_addresses(esc1_addr.get(), esc2_addr.get(), vcu_addr.get());
            }
            return;
        }
    }
}

void AP_EZKontrol::update()
{
    if (_driver == nullptr) {
        return;
    }

    float left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft);
    float right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight);

    if (cmd_mode.get() == 0) {
        int16_t max_cur = target_phase_cur.get();
        _driver->set_target(0, int16_t(constrain_float(left, -1.0f, 1.0f) * max_cur), 0);
        _driver->set_target(1, int16_t(constrain_float(right, -1.0f, 1.0f) * max_cur), 0);
    } else {
        int16_t cur = target_phase_cur.get();
        _driver->set_target(0, cur, int16_t(constrain_float(left, -1.0f, 1.0f) * 32000.0f));
        _driver->set_target(1, cur, int16_t(constrain_float(right, -1.0f, 1.0f) * 32000.0f));
    }
    _driver->set_control_mode(cmd_mode.get());
    _driver->update();
}

AP_EZKontrol *AP_EZKontrol::_singleton;

namespace AP {
AP_EZKontrol *ezkontrol()
{
    return AP_EZKontrol::get_singleton();
}
};

#endif // AP_EZKONTROL_ENABLED
