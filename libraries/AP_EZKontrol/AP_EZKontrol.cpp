#include "AP_EZKontrol.h"

#if AP_EZKONTROL_ENABLED
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// CAN IDs
static const uint32_t ID_VCU_TX = 0x0C01EFD0;
static const uint32_t ID_MCU_TX1 = 0x1801D0EF;
static const uint32_t ID_MCU_TX2 = 0x1802D0EF;

AP_EZKontrol_Driver::AP_EZKontrol_Driver() : CANSensor("EZKontrol")
{
    register_driver(AP_CAN::Protocol::EZKontrol);
}

void AP_EZKontrol_Driver::send_handshake()
{
    uint8_t buf[8];
    memset(buf, 0xAA, sizeof(buf));
    AP_HAL::CANFrame frame(ID_VCU_TX | AP_HAL::CANFrame::FlagEFF, buf, sizeof(buf), false);
    write_frame(frame, 1000);
}

void AP_EZKontrol_Driver::send_command()
{
    uint8_t buf[8];
    buf[0] = _target_current & 0xFF;
    buf[1] = (_target_current >> 8) & 0xFF;
    buf[2] = _target_speed & 0xFF;
    buf[3] = (_target_speed >> 8) & 0xFF;
    buf[4] = _control_mode;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = _life++;

    AP_HAL::CANFrame frame(ID_VCU_TX | AP_HAL::CANFrame::FlagEFF, buf, sizeof(buf), false);
    write_frame(frame, 1000);
}

void AP_EZKontrol_Driver::update()
{
    const uint32_t now = AP_HAL::millis();
    if (!_handshake_done) {
        if (now - _last_tx_ms >= 50) {
            send_handshake();
            _last_tx_ms = now;
        }
        return;
    }

    if (now - _last_tx_ms >= 50) {
        send_command();
        _last_tx_ms = now;
    }
}

void AP_EZKontrol_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    if (id == ID_MCU_TX1) {
        if (frame.dlc == 8) {
            bool all55 = true;
            for (uint8_t i=0;i<8;i++) {
                if (frame.data[i] != 0x55) { all55 = false; break; }
            }
            if (all55 && !_handshake_done) {
                _handshake_done = true;
                return;
            }

#if HAL_WITH_ESC_TELEM
            int16_t bus_v = le16toh_ptr(&frame.data[0]);
            int16_t bus_c = le16toh_ptr(&frame.data[2]);
            int16_t phase_c = le16toh_ptr(&frame.data[4]);
            int16_t speed = le16toh_ptr(&frame.data[6]);

            TelemetryData t{};
            t.voltage = float(bus_v) * 0.1f;
            t.current = float(bus_c) * 0.1f;
            update_rpm(0, speed, 0);
            update_telem_data(0, t,
                              TelemetryType::CURRENT |
                              TelemetryType::VOLTAGE);
#endif
        }
    } else if (id == ID_MCU_TX2 && frame.dlc == 8) {
#if HAL_WITH_ESC_TELEM
        int8_t ctrl_temp = int8_t(frame.data[0]);
        int8_t motor_temp = int8_t(frame.data[1]);
        TelemetryData t{};
        t.temperature_cdeg = motor_temp * 100;
        t.motor_temp_cdeg = ctrl_temp * 100;
        update_telem_data(0, t,
                          TelemetryType::TEMPERATURE |
                          TelemetryType::MOTOR_TEMPERATURE);
#endif
    }
}

AP_EZKontrol::AP_EZKontrol()
{
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

AP_EZKontrol *AP_EZKontrol::_singleton;

namespace AP {
AP_EZKontrol *ezkontrol()
{
    return AP_EZKontrol::get_singleton();
}
};

#endif // AP_EZKONTROL_ENABLED
