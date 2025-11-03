// ade7880_ext.cpp
//
// Extended ADE7880 component for ESPHome:
// - keeps original Shelly 3EM reverse-engineered behaviour
// - adds per-phase frequency (from APERIOD/BPERIOD/CPERIOD)
// - adds per-phase voltage/current THD (from VTHDN/ITHDN)
// - cycles ADE7880 harmonic engine through phases A -> B -> C
//
// Place alongside:
//   ade7880_ext.h
//   ade7880_ext_i2c.cpp
//   ade7880_ext_registers.h
//
// in components/ade7880_ext/

#include "ade7880_ext.h"
#include "ade7880_ext_registers.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>

namespace esphome {
namespace ade7880_ext {

static const char *const TAG = "ade7880_ext";

void IRAM_ATTR ADE7880ExtStore::gpio_intr(ADE7880ExtStore *arg) { arg->reset_done = true; }

void ADE7880Ext::setup() {
  if (this->irq0_pin_ != nullptr) {
    this->irq0_pin_->setup();
  }
  this->irq1_pin_->setup();
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
  }
  this->store_.irq1_pin = this->irq1_pin_->to_isr();
  this->irq1_pin_->attach_interrupt(ADE7880ExtStore::gpio_intr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);

  // if IRQ1 is already asserted, the cause must be determined
  if (this->irq1_pin_->digital_read() == 0) {
    ESP_LOGD(TAG, "IRQ1 found asserted during setup()");
    auto status1 = this->read_u32_register16_(STATUS1);
    if ((status1 & ~STATUS1_RSTDONE) != 0) {
      // not safe to proceed, must initiate reset
      ESP_LOGD(TAG, "IRQ1 asserted for !RSTDONE, resetting device");
      this->reset_device_();
      return;
    }
    if ((status1 & STATUS1_RSTDONE) == STATUS1_RSTDONE) {
      // safe to proceed, device has just completed reset cycle
      ESP_LOGD(TAG, "Acknowledging RSTDONE");
      this->write_u32_register16_(STATUS0, 0xFFFF);
      this->write_u32_register16_(STATUS1, 0xFFFF);
      this->init_device_();
      return;
    }
  }

  this->reset_device_();
}

void ADE7880Ext::loop() {
  // check for completion of a reset cycle
  if (!this->store_.reset_done) {
    return;
  }

  ESP_LOGD(TAG, "Acknowledging RSTDONE");
  this->write_u32_register16_(STATUS0, 0xFFFF);
  this->write_u32_register16_(STATUS1, 0xFFFF);
  this->init_device_();
  this->store_.reset_done = false;
  this->store_.reset_pending = false;
}

template<typename F>
void ADE7880Ext::update_sensor_from_s24zp_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_s24zp_register16_(a_register);
  sensor->publish_state(f(val));
}

template<typename F>
void ADE7880Ext::update_sensor_from_s16_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_s16_register16_(a_register);
  sensor->publish_state(f(val));
}

template<typename F>
void ADE7880Ext::update_sensor_from_s32_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_s32_register16_(a_register);
  sensor->publish_state(f(val));
}

void ADE7880Ext::update() {
  if (this->store_.reset_pending) {
    return;
  }

  auto start = millis();

  // -------- Neutral --------
  if (this->channel_n_ != nullptr) {
    auto *chan = this->channel_n_;
    this->update_sensor_from_s24zp_register16_(chan->current, NIRMS, [](float val) { return val / 100000.0f; });
  }

  // -------- Phase A --------
  if (this->channel_a_ != nullptr) {
    auto *chan = this->channel_a_;
    this->update_sensor_from_s24zp_register16_(chan->current, AIRMS, [](float val) { return val / 100000.0f; });
    this->update_sensor_from_s24zp_register16_(chan->voltage, AVRMS, [](float val) { return val / 10000.0f; });
    this->update_sensor_from_s24zp_register16_(chan->active_power, AWATT, [](float val) { return val / 100.0f; });
    this->update_sensor_from_s24zp_register16_(chan->apparent_power, AVA, [](float val) { return val / 100.0f; });
    this->update_sensor_from_s16_register16_(chan->power_factor, APF,
                                             [](float val) { return std::abs(val / -327.68f); });
    this->update_sensor_from_s32_register16_(chan->forward_active_energy, AFWATTHR, [&chan](float val) {
      return chan->forward_active_energy_total += val / 14400.0f;
    });
    this->update_sensor_from_s32_register16_(chan->reverse_active_energy, AFWATTHR, [&chan](float val) {
      return chan->reverse_active_energy_total += val / 14400.0f;
    });

    // per-phase frequency from APERIOD
    if (chan->frequency != nullptr) {
      uint16_t period = this->read_u16_register16_(APERIOD);
      if (period != 0) {
        // according to ADE7880 docs: frequency = 256000 / period
        float freq = 256000.0f / static_cast<float>(period);
        chan->frequency->publish_state(freq);
      }
    }
  }

  // -------- Phase B --------
  if (this->channel_b_ != nullptr) {
    auto *chan = this->channel_b_;
    this->update_sensor_from_s24zp_register16_(chan->current, BIRMS, [](float val) { return val / 100000.0f; });
    this->update_sensor_from_s24zp_register16_(chan->voltage, BVRMS, [](float val) { return val / 10000.0f; });
    this->update_sensor_from_s24zp_register16_(chan->active_power, BWATT, [](float val) { return val / 100.0f; });
    this->update_sensor_from_s24zp_register16_(chan->apparent_power, BVA, [](float val) { return val / 100.0f; });
    this->update_sensor_from_s16_register16_(chan->power_factor, BPF,
                                             [](float val) { return std::abs(val / -327.68f); });
    this->update_sensor_from_s32_register16_(chan->forward_active_energy, BFWATTHR, [&chan](float val) {
      return chan->forward_active_energy_total += val / 14400.0f;
    });
    this->update_sensor_from_s32_register16_(chan->reverse_active_energy, BFWATTHR, [&chan](float val) {
      return chan->reverse_active_energy_total += val / 14400.0f;
    });

    // per-phase frequency from BPERIOD
    if (chan->frequency != nullptr) {
      uint16_t period = this->read_u16_register16_(BPERIOD);
      if (period != 0) {
        float freq = 256000.0f / static_cast<float>(period);
        chan->frequency->publish_state(freq);
      }
    }
  }

  // -------- Phase C --------
  if (this->channel_c_ != nullptr) {
    auto *chan = this->channel_c_;
    this->update_sensor_from_s24zp_register16_(chan->current, CIRMS, [](float val) { return val / 100000.0f; });
    this->update_sensor_from_s24zp_register16_(chan->voltage, CVRMS, [](float val) { return val / 10000.0f; });
    this->update_sensor_from_s24zp_register16_(chan->active_power, CWATT, [](float val) { return val / 100.0f; });
    this->update_sensor_from_s24zp_register16_(chan->apparent_power, CVA, [](float val) { return val / 100.0f; });
    this->update_sensor_from_s16_register16_(chan->power_factor, CPF,
                                             [](float val) { return std::abs(val / -327.68f); });
    this->update_sensor_from_s32_register16_(chan->forward_active_energy, CFWATTHR, [&chan](float val) {
      return chan->forward_active_energy_total += val / 14400.0f;
    });
    this->update_sensor_from_s32_register16_(chan->reverse_active_energy, CFWATTHR, [&chan](float val) {
      return chan->reverse_active_energy_total += val / 14400.0f;
    });

    // per-phase frequency from CPERIOD
    if (chan->frequency != nullptr) {
      uint16_t period = this->read_u16_register16_(CPERIOD);
      if (period != 0) {
        float freq = 256000.0f / static_cast<float>(period);
        chan->frequency->publish_state(freq);
      }
    }
  }

  // ---------- THD cycling ----------
  // we only bother if user actually defined any THD sensor
  bool any_thd =
      (this->channel_a_ && (this->channel_a_->voltage_thd || this->channel_a_->current_thd)) ||
      (this->channel_b_ && (this->channel_b_->voltage_thd || this->channel_b_->current_thd)) ||
      (this->channel_c_ && (this->channel_c_->voltage_thd || this->channel_c_->current_thd));

  if (any_thd) {
    uint8_t phase = this->thd_phase_index_ % 3;

    PowerChannel *target = nullptr;
    switch (phase) {
      case 0:
        target = this->channel_a_;
        break;
      case 1:
        target = this->channel_b_;
        break;
      case 2:
        target = this->channel_c_;
        break;
    }

    if (target != nullptr) {
      // read THD for currently selected phase
      int32_t vthd_raw = this->read_s24zp_register16_(VTHDN);
      int32_t ithd_raw = this->read_s24zp_register16_(ITHDN);

      // VTHDN/ITHDN are 3.21 signed -> divide by 2^21 to get ratio, *100 for percent
      constexpr float denom = 2097152.0f;  // 2^21
      if (target->voltage_thd != nullptr) {
        float pct = (static_cast<float>(vthd_raw) / denom) * 100.0f;
        target->voltage_thd->publish_state(pct);
      }
      if (target->current_thd != nullptr) {
        float pct = (static_cast<float>(ithd_raw) / denom) * 100.0f;
        target->current_thd->publish_state(pct);
      }
    }

    // advance to next phase and program ADE7880 harmonic engine for that phase
    this->thd_phase_index_ = (this->thd_phase_index_ + 1) % 3;
    this->select_harmonic_phase_(this->thd_phase_index_);
  }

  ESP_LOGD(TAG, "update took %" PRIu32 " ms", millis() - start);
}

void ADE7880Ext::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7880Ext (extended ADE7880):");
  LOG_PIN("  IRQ0  Pin: ", this->irq0_pin_);
  LOG_PIN("  IRQ1  Pin: ", this->irq1_pin_);
  LOG_PIN("  RESET Pin: ", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency (nominal): %.0f Hz", this->frequency_);

  if (this->channel_a_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Phase A:");
    LOG_SENSOR("    ", "Current", this->channel_a_->current);
    LOG_SENSOR("    ", "Voltage", this->channel_a_->voltage);
    LOG_SENSOR("    ", "Active Power", this->channel_a_->active_power);
    LOG_SENSOR("    ", "Apparent Power", this->channel_a_->apparent_power);
    LOG_SENSOR("    ", "Power Factor", this->channel_a_->power_factor);
    LOG_SENSOR("    ", "Forward Active Energy", this->channel_a_->forward_active_energy);
    LOG_SENSOR("    ", "Reverse Active Energy", this->channel_a_->reverse_active_energy);
    LOG_SENSOR("    ", "Frequency", this->channel_a_->frequency);
    LOG_SENSOR("    ", "Voltage THD", this->channel_a_->voltage_thd);
    LOG_SENSOR("    ", "Current THD", this->channel_a_->current_thd);
    ESP_LOGCONFIG(TAG,
                  "    Calibration:\n"
                  "      Current: %" PRId32 "\n"
                  "      Voltage: %" PRId32 "\n"
                  "      Power: %" PRId32 "\n"
                  "      Phase Angle: %u",
                  this->channel_a_->current_gain_calibration,
                  this->channel_a_->voltage_gain_calibration,
                  this->channel_a_->power_gain_calibration,
                  this->channel_a_->phase_angle_calibration);
  }

  if (this->channel_b_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Phase B:");
    LOG_SENSOR("    ", "Current", this->channel_b_->current);
    LOG_SENSOR("    ", "Voltage", this->channel_b_->voltage);
    LOG_SENSOR("    ", "Active Power", this->channel_b_->active_power);
    LOG_SENSOR("    ", "Apparent Power", this->channel_b_->apparent_power);
    LOG_SENSOR("    ", "Power Factor", this->channel_b_->power_factor);
    LOG_SENSOR("    ", "Forward Active Energy", this->channel_b_->forward_active_energy);
    LOG_SENSOR("    ", "Reverse Active Energy", this->channel_b_->reverse_active_energy);
    LOG_SENSOR("    ", "Frequency", this->channel_b_->frequency);
    LOG_SENSOR("    ", "Voltage THD", this->channel_b_->voltage_thd);
    LOG_SENSOR("    ", "Current THD", this->channel_b_->current_thd);
    ESP_LOGCONFIG(TAG,
                  "    Calibration:\n"
                  "      Current: %" PRId32 "\n"
                  "      Voltage: %" PRId32 "\n"
                  "      Power: %" PRId32 "\n"
                  "      Phase Angle: %u",
                  this->channel_b_->current_gain_calibration,
                  this->channel_b_->voltage_gain_calibration,
                  this->channel_b_->power_gain_calibration,
                  this->channel_b_->phase_angle_calibration);
  }

  if (this->channel_c_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Phase C:");
    LOG_SENSOR("    ", "Current", this->channel_c_->current);
    LOG_SENSOR("    ", "Voltage", this->channel_c_->voltage);
    LOG_SENSOR("    ", "Active Power", this->channel_c_->active_power);
    LOG_SENSOR("    ", "Apparent Power", this->channel_c_->apparent_power);
    LOG_SENSOR("    ", "Power Factor", this->channel_c_->power_factor);
    LOG_SENSOR("    ", "Forward Active Energy", this->channel_c_->forward_active_energy);
    LOG_SENSOR("    ", "Reverse Active Energy", this->channel_c_->reverse_active_energy);
    LOG_SENSOR("    ", "Frequency", this->channel_c_->frequency);
    LOG_SENSOR("    ", "Voltage THD", this->channel_c_->voltage_thd);
    LOG_SENSOR("    ", "Current THD", this->channel_c_->current_thd);
    ESP_LOGCONFIG(TAG,
                  "    Calibration:\n"
                  "      Current: %" PRId32 "\n"
                  "      Voltage: %" PRId32 "\n"
                  "      Power: %" PRId32 "\n"
                  "      Phase Angle: %u",
                  this->channel_c_->current_gain_calibration,
                  this->channel_c_->voltage_gain_calibration,
                  this->channel_c_->power_gain_calibration,
                  this->channel_c_->phase_angle_calibration);
  }

  if (this->channel_n_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Neutral:");
    LOG_SENSOR("    ", "Current", this->channel_n_->current);
    ESP_LOGCONFIG(TAG,
                  "    Calibration:\n"
                  "      Current: %" PRId32,
                  this->channel_n_->current_gain_calibration);
  }

  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
}

void ADE7880Ext::calibrate_s10zp_reading_(uint16_t a_register, int16_t calibration) {
  if (calibration == 0) {
    return;
  }
  this->write_s10zp_register16_(a_register, calibration);
}

void ADE7880Ext::calibrate_s24zpse_reading_(uint16_t a_register, int32_t calibration) {
  if (calibration == 0) {
    return;
  }
  this->write_s24zpse_register16_(a_register, calibration);
}

void ADE7880Ext::init_device_() {
  // lock I2C
  this->write_u8_register16_(CONFIG2, CONFIG2_I2C_LOCK);

  // gain defaults
  this->write_u16_register16_(GAIN, 0);

  // configure line frequency compensation
  if (this->frequency_ > 55.0f) {
    this->write_u16_register16_(COMPMODE, COMPMODE_DEFAULT | COMPMODE_SELFREQ);
  }

  // apply channel calibrations
  if (this->channel_n_ != nullptr) {
    this->calibrate_s24zpse_reading_(NIGAIN, this->channel_n_->current_gain_calibration);
  }

  if (this->channel_a_ != nullptr) {
    this->calibrate_s24zpse_reading_(AIGAIN, this->channel_a_->current_gain_calibration);
    this->calibrate_s24zpse_reading_(AVGAIN, this->channel_a_->voltage_gain_calibration);
    this->calibrate_s24zpse_reading_(APGAIN, this->channel_a_->power_gain_calibration);
    this->calibrate_s10zp_reading_(APHCAL, this->channel_a_->phase_angle_calibration);
  }

  if (this->channel_b_ != nullptr) {
    this->calibrate_s24zpse_reading_(BIGAIN, this->channel_b_->current_gain_calibration);
    this->calibrate_s24zpse_reading_(BVGAIN, this->channel_b_->voltage_gain_calibration);
    this->calibrate_s24zpse_reading_(BPGAIN, this->channel_b_->power_gain_calibration);
    this->calibrate_s10zp_reading_(BPHCAL, this->channel_b_->phase_angle_calibration);
  }

  if (this->channel_c_ != nullptr) {
    this->calibrate_s24zpse_reading_(CIGAIN, this->channel_c_->current_gain_calibration);
    this->calibrate_s24zpse_reading_(CVGAIN, this->channel_c_->voltage_gain_calibration);
    this->calibrate_s24zpse_reading_(CPGAIN, this->channel_c_->power_gain_calibration);
    this->calibrate_s10zp_reading_(CPHCAL, this->channel_c_->phase_angle_calibration);
  }

  // write three default values to flush I2C write queue
  this->write_s32_register16_(VLEVEL, 0);
  this->write_s32_register16_(VLEVEL, 0);
  this->write_s32_register16_(VLEVEL, 0);

  // enable DSP write protection (we'll unlock only when changing HCONFIG)
  this->write_u8_register16_(DSPWP_SEL, DSPWP_SEL_SET);
  this->write_u8_register16_(DSPWP_SET, DSPWP_SET_RO);

  // start DSP
  this->write_u16_register16_(RUN, RUN_ENABLE);

  // set initial harmonic phase to A
  this->thd_phase_index_ = 0;
  this->select_harmonic_phase_(0);
}

void ADE7880Ext::select_harmonic_phase_(uint8_t phase) {
  // unlock RAM writes
  this->write_u8_register16_(DSPWP_SEL, DSPWP_SEL_SET);
  this->write_u8_register16_(DSPWP_SET, 0x00);

  uint16_t hcfg = this->read_u16_register16_(HCONFIG);
  hcfg &= ~(HCONFIG_ACTPHSEL_MASK | HCONFIG_HPHASE_MASK);

  uint16_t ph = phase % 3;
  hcfg |= (ph << HCONFIG_ACTPHSEL_SHIFT);
  hcfg |= (ph << HCONFIG_HPHASE_SHIFT);

  this->write_u16_register16_(HCONFIG, hcfg);

  // relock
  this->write_u8_register16_(DSPWP_SEL, DSPWP_SEL_SET);
  this->write_u8_register16_(DSPWP_SET, DSPWP_SET_RO);
}

void ADE7880Ext::reset_device_() {
  if (this->reset_pin_ != nullptr) {
    ESP_LOGD(TAG, "Reset device using RESET pin");
    this->reset_pin_->digital_write(false);
    delay(1);
    this->reset_pin_->digital_write(true);
  } else {
    ESP_LOGD(TAG, "Reset device using SWRST command");
    this->write_u16_register16_(CONFIG, CONFIG_SWRST);
  }
  this->store_.reset_pending = true;
}

}  // namespace ade7880_ext
}  // namespace esphome
