// Display Library for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: the e-paper panels require 3.3V supply AND data lines!
//
// Display Library based on Demo Example from Good Display: https://www.good-display.com/companyfile/32/
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

#include "GxEPD2_EPD.h"

#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif

GxEPD2_EPD::GxEPD2_EPD(UniversalPin *cs, UniversalPin *dc, UniversalPin *rst, UniversalPin *busy, int16_t busy_level, uint32_t busy_timeout,
                       uint16_t w, uint16_t h, GxEPD2::Panel p, bool c, bool pu, bool fpu) :
  WIDTH(w), HEIGHT(h), panel(p), hasColor(c), hasPartialUpdate(pu), hasFastPartialUpdate(fpu),
  _cs(cs), _dc(dc), _rst(rst), _busy(busy), _busy_level(busy_level), _busy_timeout(busy_timeout), _diag_enabled(false),
  _pSPIx(&SPI), _spi_settings(4000000, MSBFIRST, SPI_MODE0)
{
  _initial_write = true;
  _initial_refresh = true;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _init_display_done = false;
  _reset_duration = 10;
  _busy_callback = 0;
  _busy_callback_parameter = 0;
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate)
{
  init(serial_diag_bitrate, true, 10, false);
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate, bool initial, uint16_t reset_duration, bool pulldown_rst_mode)
{
  _initial_write = initial;
  _initial_refresh = initial;
  _pulldown_rst_mode = pulldown_rst_mode;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _init_display_done = false;
  _reset_duration = reset_duration;
  if (serial_diag_bitrate > 0)
  {
    Serial.begin(serial_diag_bitrate);
    _diag_enabled = true;
  }
  if (_cs->toInt() >= 0)
  {
    _cs->digitalWrite(HIGH);
    _cs->pinMode(OUTPUT);
  }
  if (_dc->toInt() >= 0)
  _reset();
  _pSPIx->begin(); // may steal _rst pin (Waveshare Pico-ePaper-2.9)
  if (_rst >= 0)
  {
    _rst->digitalWrite( HIGH); // preset (less glitch for any analyzer)
    _rst->pinMode(OUTPUT);
    _rst->digitalWrite( HIGH); // set (needed e.g. for RP2040)
  }
  if (_cs->toInt() >= 0)
  {
    _cs->digitalWrite( HIGH); // preset (less glitch for any analyzer)
    _cs->pinMode(OUTPUT);
    _cs->digitalWrite( HIGH); // set (needed e.g. for RP2040)
  }
  if (_dc >= 0)
  {
    _dc->digitalWrite(HIGH);
    _dc->pinMode(OUTPUT);
  }
  _reset();
  if (_busy->toInt() >= 0)
  {
    _busy->pinMode(INPUT);
  }
  _pSPIx->begin();
  if (_busy->toInt() == MISO) // may be overridden
  {
    _busy->pinMode(INPUT);
  }
  if (_dc->toInt() == MISO) // may be overridden, TTGO T5 V2.66
  {
    _dc->pinMode(OUTPUT);
  }
  if (_cs->toInt() == MISO) // may be overridden
  {
    _cs->pinMode(INPUT);
  }
}

void GxEPD2_EPD::end()
{
  _pSPIx->end();
  if (_cs->toInt() >= 0) _cs->pinMode(INPUT);
  if (_dc->toInt() >= 0) _dc->pinMode(INPUT);
  if (_rst->toInt() >= 0) _rst->pinMode(INPUT);
}

void GxEPD2_EPD::setBusyCallback(void (*busyCallback)(const void*), const void* busy_callback_parameter)
{
  _busy_callback = busyCallback;
  _busy_callback_parameter = busy_callback_parameter;
}

void GxEPD2_EPD::selectSPI(SPIClass& spi, SPISettings spi_settings)
{
  _pSPIx = &spi;
  _spi_settings = spi_settings;
}

void GxEPD2_EPD::_reset()
{
  if (_rst->toInt() >= 0)
  {
    if (_pulldown_rst_mode)
    {
      _rst->digitalWrite( LOW);
      _rst->pinMode( OUTPUT);
      delay(_reset_duration);
      _rst->pinMode( INPUT_PULLUP);
      delay(_reset_duration > 10 ? _reset_duration : 10);
    }
    else
    {
      _rst->digitalWrite(HIGH); // NEEDED for Waveshare "clever" reset circuit->digitalWrite(power controller before reset pulse
      _rst->pinMode( OUTPUT);
      delay(10); // NEEDED for Waveshare "clever" reset circuit, at least delay(2);
      _rst->digitalWrite( LOW);
      delay(_reset_duration);
      _rst->digitalWrite( HIGH);
      delay(_reset_duration > 10 ? _reset_duration : 10);
    }
    _hibernating = false;
  }
}

void GxEPD2_EPD::_waitWhileBusy(const char* comment, uint16_t busy_time)
{
  if (_busy->toInt() >= 0)
  {
    delay(1); // add some margin to become active
    unsigned long start = micros();
    while (1)
    {
      if (_busy->digitalRead() != _busy_level) break;
      if (_busy_callback) _busy_callback(_busy_callback_parameter);
      else delay(1);
      if (_busy->digitalRead() != _busy_level) break;
      if (micros() - start > _busy_timeout)
      {
//        Serial.println("Busy Timeout!");
        throw std::runtime_error("Busy Timeout");
        break;
      }
#if defined(ESP8266) || defined(ESP32)
      yield(); // avoid wdt
#endif
    }
    if (comment)
    {
#if !defined(DISABLE_DIAGNOSTIC_OUTPUT)
      if (_diag_enabled)
      {
        unsigned long elapsed = micros() - start;
        Serial.print(comment);
        Serial.print(" : ");
        Serial.println(elapsed);
      }
#endif
    }
    (void) start;
  }
  else delay(busy_time);
}

void GxEPD2_EPD::_writeCommand(uint8_t c)
{
  _pSPIx->beginTransaction(_spi_settings);
  if (_dc->toInt() >= 0) _dc->digitalWrite( LOW);
  if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
  _pSPIx->transfer(c);
  if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  if (_dc->toInt() >= 0) _dc->digitalWrite( HIGH);
  _pSPIx->endTransaction();
}

void GxEPD2_EPD::_writeData(uint8_t d)
{
  _pSPIx->beginTransaction(_spi_settings);
  if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
  _pSPIx->transfer(d);
  if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  _pSPIx->endTransaction();
}

void GxEPD2_EPD::_writeData(const uint8_t* data, uint16_t n)
{
  _pSPIx->beginTransaction(_spi_settings);
  if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
  for (uint16_t i = 0; i < n; i++)
  {
    _pSPIx->transfer(*data++);
  }
  if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  _pSPIx->endTransaction();
}

void GxEPD2_EPD::_writeDataPGM(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  _pSPIx->beginTransaction(_spi_settings);
  if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
  for (uint16_t i = 0; i < n; i++)
  {
    _pSPIx->transfer(pgm_read_byte(&*data++));
  }
  while (fill_with_zeroes > 0)
  {
    _pSPIx->transfer(0x00);
    fill_with_zeroes--;
  }
  if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  _pSPIx->endTransaction();
}

void GxEPD2_EPD::_writeDataPGM_sCS(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  _pSPIx->beginTransaction(_spi_settings);
  for (uint8_t i = 0; i < n; i++)
  {
    if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
    _pSPIx->transfer(pgm_read_byte(&*data++));
    if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  }
  while (fill_with_zeroes > 0)
  {
    if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
    _pSPIx->transfer(0x00);
    fill_with_zeroes--;
    if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  }
  _pSPIx->endTransaction();
}

void GxEPD2_EPD::_writeCommandData(const uint8_t* pCommandData, uint8_t datalen)
{
  _pSPIx->beginTransaction(_spi_settings);
  if (_dc->toInt() >= 0) _dc->digitalWrite( LOW);
  if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
  _pSPIx->transfer(*pCommandData++);
  if (_dc->toInt() >= 0) _dc->digitalWrite( HIGH);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    _pSPIx->transfer(*pCommandData++);
  }
  if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  _pSPIx->endTransaction();
}

void GxEPD2_EPD::_writeCommandDataPGM(const uint8_t* pCommandData, uint8_t datalen)
{
  _pSPIx->beginTransaction(_spi_settings);
  if (_dc->toInt() >= 0) _dc->digitalWrite( LOW);
  if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
  _pSPIx->transfer(pgm_read_byte(&*pCommandData++));
  if (_dc->toInt() >= 0) _dc->digitalWrite( HIGH);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    _pSPIx->transfer(pgm_read_byte(&*pCommandData++));
  }
  if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  _pSPIx->endTransaction();
}

void GxEPD2_EPD::_startTransfer()
{
  _pSPIx->beginTransaction(_spi_settings);
  if (_cs->toInt() >= 0) _cs->digitalWrite( LOW);
}

void GxEPD2_EPD::_transfer(uint8_t value)
{
  _pSPIx->transfer(value);
}

void GxEPD2_EPD::_endTransfer()
{
  if (_cs->toInt() >= 0) _cs->digitalWrite( HIGH);
  _pSPIx->endTransaction();
}
