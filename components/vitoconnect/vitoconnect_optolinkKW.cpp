/* VitoWiFi

Copyright 2019 Bert Melis

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "vitoconnect_optolinkKW.h"

namespace esphome {
namespace vitoconnect {

static const char *TAG = "vitoconnect";

OptolinkKW::OptolinkKW(uart::UARTDevice* uart) :
  Optolink(uart),
  _state(UNDEF),
  _lastMillis(0),
  _lastInitResetMillis(0),
  _initResetAttempts(0),
  _initBackoffUntil(0),
  _write(false),
  _rcvBuffer{0},
  _rcvBufferLen(0),
  _rcvLen(0) {}

void OptolinkKW::begin() {
  _state = INIT;
  _lastMillis = millis();
  _lastInitResetMillis = 0;
  _initResetAttempts = 0;
  _initBackoffUntil = 0;
}

void OptolinkKW::loop() {
  switch (_state) {
  case INIT:
    _init();
    break;
  case IDLE:
    _idle();
    break;
  case SYNC:
    _sync();
    break;
  case SEND:
    _send();
    break;
  case RECEIVE:
    _receive();
    break;
  default:
    // begin() not called
    break;
  }
  // Do not apply the generic request timeout while we are already in INIT.
  // INIT has its own sync/backoff recovery. Resetting it here would cancel the
  // quiet backoff window and make us hammer 0x04 forever.
  if (_state != INIT && _queue.size() > 0 && millis() - _lastMillis > 5000UL) {
    ESP_LOGW(TAG, "KW timeout: no protocol progress for 5s, dropping current request and resyncing");
    _tryOnError(TIMEOUT);
    _state = INIT;
    _uart->flush();
    _lastMillis = millis();
    _lastInitResetMillis = 0;
    _initResetAttempts = 0;
    _initBackoffUntil = 0;
  }
  // TODO(@bertmelis): move timeouts here, clear queue on timeout
}

void OptolinkKW::_init() {
  const uint32_t now = millis();

  if (_uart->available()) {
    if (_uart->peek() == 0x05) {
      if (_initResetAttempts > 0) {
        ESP_LOGI(TAG, "KW sync recovered after %u reset attempt(s)", _initResetAttempts);
      }
      _initResetAttempts = 0;
      _initBackoffUntil = 0;
      _lastMillis = now;
      _state = IDLE;
      _idle();
    } else {
      _uart->read();
    }
    return;
  }

  // After five unsuccessful reset bytes, stop transmitting for 30 seconds.
  // This gives the Vitotronic a quiet window to return to its normal KW sync state.
  if (_initBackoffUntil != 0) {
    if ((int32_t) (now - _initBackoffUntil) < 0) {
      return;
    }
    ESP_LOGD(TAG, "KW sync backoff finished, retrying");
    _initBackoffUntil = 0;
    _initResetAttempts = 0;
    _lastInitResetMillis = 0;
  }

  // Keep the 0x04 reset cadence independent from _lastMillis.
  // _lastMillis tracks real protocol progress and must not be refreshed merely
  // because we transmitted another reset byte.
  if (now - _lastInitResetMillis > 1000UL) {
    _lastInitResetMillis = now;
    const uint8_t buff[] = {0x04};
    _uart->write_array(buff, sizeof(buff));
    ++_initResetAttempts;

    if (_initResetAttempts >= 5) {
      ESP_LOGW(TAG, "KW sync not acquired after %u reset attempts, backing off for 30s", _initResetAttempts);
      _initBackoffUntil = now + 30000UL;
    }
  }
}

void OptolinkKW::_idle() {
  if (_uart->available()) {
    if (_uart->read() == 0x05) {
      _lastMillis = millis();
      if (_queue.size() > 0) {
        _state = SYNC;
      }
    } else {
      ESP_LOGD(TAG, "Received unexpected data");
      // received something unexpected
    }
  } else if ((_queue.size() > 0) && (millis() - _lastMillis < 10UL)) {  // don't wait for 0x05 sync signal, send directly after last request
    _state = SEND;
    _send();
  } else if (millis() - _lastMillis > 5 * 1000UL) {
    _state = INIT;
  }
}

void OptolinkKW::_sync() {
  const uint8_t buff[1] = {0x01};
  _uart->write_array(buff, sizeof(buff));
  _state = SEND;
  _send();
}

void OptolinkKW::_send() {
  uint8_t buff[MAX_DP_LENGTH + 4];
  OptolinkDP* dp = _queue.front();
  uint8_t length = dp->length;
  uint16_t address = dp->address;
  if (dp->write) {
    // type is WRITE, has length of 4 chars + length of value
    buff[0] = 0xF4;
    buff[1] = (address >> 8) & 0xFF;
    buff[2] = address & 0xFF;
    buff[3] = length;
    // add value to message
    memcpy(&buff[4], dp->data, length);
    _rcvLen = 1;  // expected answer length is only ACK (0x00)
    _uart->write_array(buff, 4 + length);
  } else {
    // type is READ
    // has fixed length of 4 chars
    buff[0] = 0xF7;
    buff[1] = (address >> 8) & 0xFF;
    buff[2] = address & 0xFF;
    buff[3] = length;
    _rcvLen = length;  // expected answer length is requested length
    _uart->write_array(buff, 4);
  }
  _rcvBufferLen = 0;
  _lastMillis = millis();
  _state = RECEIVE;
}

void OptolinkKW::_receive() {
  while (_uart->available() != 0) {  // read complete RX buffer
    _rcvBuffer[_rcvBufferLen] = _uart->read();
    ++_rcvBufferLen;
    _lastMillis = millis();
  }
  if (_rcvBufferLen == _rcvLen) {  // message complete, TODO: check message (eg 0x00 for READ messages)   
    OptolinkDP* dp = _queue.front();
    ESP_LOGD(TAG, "Adding data to datapoint with address %x and received length %d", dp->address, _rcvBufferLen);
    _tryOnData(_rcvBuffer, _rcvBufferLen);
    _state = IDLE;
    _lastMillis = millis();
    return;
  } else if (millis() - _lastMillis > 1 * 1000UL) {  // Vitotronic isn't answering, try again
    ESP_LOGD(TAG, "Received length %d doesn't match expected length %d", _rcvBufferLen, _rcvLen);
    _rcvBufferLen = 0;
    memset(_rcvBuffer, 0, 4);
    _state = INIT;
  }
}

}  // namespace vitoconnect
}  // namespace esphome
