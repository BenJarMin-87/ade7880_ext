#include "ade7880_ext.h"

namespace esphome {
namespace ade7880_ext {

// adapted from https://stackoverflow.com/a/55912127/1886371
template<size_t Bits, typename T> inline T sign_extend(const T &v) noexcept {
  using S = struct { signed Val : Bits; };
  return reinterpret_cast<const S *>(&v)->Val;
}

uint8_t ADE7880Ext::read_u8_register16_(uint16_t a_register) {
  uint8_t in;
  this->read_register16(a_register, &in, sizeof(in));
  return in;
}

int16_t ADE7880Ext::read_s16_register16_(uint16_t a_register) {
  int16_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

uint16_t ADE7880Ext::read_u16_register16_(uint16_t a_register) {
  uint16_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

int32_t ADE7880Ext::read_s24zp_register16_(uint16_t a_register) {
  int32_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return sign_extend<24>(convert_big_endian(in));
}

int32_t ADE7880Ext::read_s32_register16_(uint16_t a_register) {
  int32_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

uint32_t ADE7880Ext::read_u32_register16_(uint16_t a_register) {
  uint32_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

void ADE7880Ext::write_u8_register16_(uint16_t a_register, uint8_t value) {
  this->write_register16(a_register, &value, sizeof(value));
}

void ADE7880Ext::write_s10zp_register16_(uint16_t a_register, int16_t value) {
  int16_t out = convert_big_endian(value & 0x03FF);
  this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

void ADE7880Ext::write_u16_register16_(uint16_t a_register, uint16_t value) {
  uint16_t out = convert_big_endian(value);
  this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

void ADE7880Ext::write_s24zpse_register16_(uint16_t a_register, int32_t value) {
  int32_t out = convert_big_endian(value & 0x0FFFFFFF);
  this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

void ADE7880Ext::write_s32_register16_(uint16_t a_register, int32_t value) {
  int32_t out = convert_big_endian(value);
  this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

void ADE7880Ext::write_u32_register16_(uint16_t a_register, uint32_t value) {
  uint32_t out = convert_big_endian(value);
  this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

}  // namespace ade7880_ext
}  // namespace esphome
