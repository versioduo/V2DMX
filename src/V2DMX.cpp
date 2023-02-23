// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2DMX.h"
#include <wiring_private.h>

// Break:            26 bits,         104 usec (minimum 92 usec)
// Mark after Break:  3 bits,          12 usec (minimum 12 usec)
// Start Code:        1 + 8 + 2 bits,  44 usec (value '0' @ 250kHz)
// Slots:            Up to 512 slots 8 bit channel data
//
// The duration between breaks should be at least 1200 microseconds, the
// equivalent of approximately 24 slots large message.
static constexpr uint8_t dmaHeader[5]{
  0b00000000,
  0b00000000,
  0b00000000,
  0b00011100,
  0b11000000,
};

// 8 slots with value '0', stop bits, LSB first.
static constexpr uint8_t dmaBlockInit[11]{
  0b00000000,
  0b00000110,
  0b00110000,
  0b10000000,
  0b00000001,
  0b00001100,
  0b01100000,
  0b00000000,
  0b00000011,
  0b00011000,
  0b11000000,
};

void V2DMX::begin() {
  _dmaBuffer = (DMABuffer *)malloc(sizeof(DMABuffer));

  // Build SPI bus from SERCOM.
  //
  // SPIClass.begin() applies the board config to all given pins, which might not
  // match our configuration. Just pass the same pin to all of them, to make sure
  // we do not touch anything else. Our pin will be switched to the SERCOM after
  // begin().
  if (!_spi)
    _spi = new SPIClass(_sercom.sercom, _sercom.pin, _sercom.pin, _sercom.pin, _sercom.padTx, SERCOM_RX_PAD_3);

  // Configure SPI, the transaction will never stop.
  _spi->begin();
  _spi->beginTransaction(SPISettings(250000, LSBFIRST, SPI_MODE0));

  // Switch our pin to SERCOM, begin() set all given pins to the board config.
  if (_sercom.sercom)
    pinPeripheral(_sercom.pin, _sercom.pinFunc);

  reset();
}

void V2DMX::reset() {
  while (_spi->isBusy())
    yield();

  // Break + Mark + Start Code '0', LSB first.
  memcpy(_dmaBuffer, dmaHeader, sizeof(dmaHeader));

  // 64 blocks of 11 bytes, each block containing 8 channel values:
  // one start bit, value '0', two stop bits.
  for (uint8_t i = 0; i < 64; i++)
    memcpy(_dmaBuffer->blocks[i], dmaBlockInit, sizeof(dmaBlockInit));

  memset(_channels.values, 0, sizeof(_channels.values));
  _nChannels = 0;

  _transferUsec = 0;
  _updateDMA    = true;
}

static void updateDMABlock(uint8_t block[11], uint8_t channel[8]) {
  block[0]  = dmaBlockInit[0] | channel[0] << 1;
  block[1]  = dmaBlockInit[1] | channel[0] >> 7 | channel[1] << 4;
  block[2]  = dmaBlockInit[2] | channel[1] >> 4 | channel[2] << 7;
  block[3]  = dmaBlockInit[3] | channel[2] >> 1;
  block[4]  = dmaBlockInit[4] | channel[3] << 2;
  block[5]  = dmaBlockInit[5] | channel[3] >> 6 | channel[4] << 5;
  block[6]  = dmaBlockInit[6] | channel[4] >> 3;
  block[7]  = dmaBlockInit[7] | channel[5];
  block[8]  = dmaBlockInit[8] | channel[6] << 3;
  block[9]  = dmaBlockInit[9] | channel[6] >> 5 | channel[7] << 6;
  block[10] = dmaBlockInit[10] | channel[7] >> 2;
}

void V2DMX::loop() {
  if (_updateDMA) {
    // Refresh the DMA data.
    for (uint8_t i = 0; i < 64; i++) {
      // Do not needlessly update the untouched higher channels.
      if (i * 8 > _nChannels)
        break;

      updateDMABlock(_dmaBuffer->blocks[i], _channels.blocks[i]);
    }

    _updateDMA = false;
    _transfer  = true;
  }

  // Regularly send the DMX data regardless if something has changed; some
  // devices switch themselves off after a timeout.
  //
  // Do not transfer unchanged data in a loop though; we want to be able to
  // send new incoming updates as fast as possible (sync an incoming update with
  // the start of a new DMX data frame), and not needlessly wait for an unchanged
  // frame to finish transmitting.
  if (!_transfer && (unsigned long)(micros() - _transferUsec) < 400 * 1000)
    return;

  if (_spi->isBusy())
    return;

  _spi->transfer(_dmaBuffer, NULL, sizeof(DMABuffer), false);
  _transfer     = false;
  _transferUsec = micros();
}

void V2DMX::setChannels(uint16_t i, const uint8_t *data, uint16_t size) {
  if (i + size >= 512)
    return;

  // Remember the largest channel number in use to limit the DMA data update.
  if (_nChannels < i + size)
    _nChannels = i + size;

  memcpy(_channels.values + i, data, size);
  _updateDMA = true;
}

void V2DMX::setChannel(uint16_t i, uint8_t value) {
  setChannels(i, &value, 1);
}
