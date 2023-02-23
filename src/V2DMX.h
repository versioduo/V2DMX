// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <SPI.h>

class V2DMX {
public:
  constexpr V2DMX(SPIClass *spi) : _spi{spi} {}

  // Build SPI bus from SERCOM.
  constexpr V2DMX(uint8_t pin, SERCOM *sercom, SercomSpiTXPad padTx, EPioType pinFunc) :
    _sercom{.pin{pin}, .sercom{sercom}, .padTx{padTx}, .pinFunc{pinFunc}} {}

  void begin();
  void reset();

  // Encodes the DMA bit stream and fires a DMA transaction. If there
  // is a pending update and no current DMA transfer active, a new
  // transaction is started immediately.
  void loop();

  // Set channel values and request an update.
  void setChannels(uint16_t i, const uint8_t *data, uint16_t size);
  void setChannel(uint16_t i, const uint8_t value);
  uint8_t getChannel(uint16_t i) {
    return _channels.values[i];
  }

private:
  union DMABuffer {
    uint8_t bytes[5 + (11 * 64)];
    struct {
      uint8_t header[5];
      uint8_t blocks[64][11];
    };
  };

  union Channels {
    uint8_t values[512];
    uint8_t blocks[64][8];
  };

  struct {
    const uint8_t pin;
    SERCOM *sercom;
    const SercomSpiTXPad padTx;
    const EPioType pinFunc;
  } _sercom{};

  SPIClass *_spi{};

  DMABuffer *_dmaBuffer{};
  Channels _channels{};
  uint16_t _nChannels{};
  bool _updateDMA{};
  bool _transfer{};
  unsigned long _transferUsec{};
};
