// © Kay Sievers <kay@vrfy.org>, 2020-2021
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
static constexpr uint8_t dma_header[5]{0b00000000, 0b00000000, 0b00000000, 0b00011100, 0b11000000};

// 8 slots with value '0', stop bits, LSB first.
static constexpr uint8_t dma_block_init[11]{0b00000000,
                                            0b00000110,
                                            0b00110000,
                                            0b10000000,
                                            0b00000001,
                                            0b00001100,
                                            0b01100000,
                                            0b00000000,
                                            0b00000011,
                                            0b00011000,
                                            0b11000000};

void V2DMX::begin() {
  _dma_buffer = (DMABuffer *)malloc(sizeof(DMABuffer));

  // Build SPI bus from SERCOM.
  //
  // SPIClass.begin() applies the board config to all given pins, which might not
  // match our configuration. Just pass the same pin to all of them, to make sure
  // we do not touch anything else. Our pin will be switched to the SERCOM after
  // begin().
  if (!_spi)
    _spi = new SPIClass(_sercom.sercom, _sercom.pin, _sercom.pin, _sercom.pin, _sercom.pad_tx, SERCOM_RX_PAD_3);

  // Configure SPI, the transaction will never stop.
  _spi->begin();
  _spi->beginTransaction(SPISettings(250000, LSBFIRST, SPI_MODE0));

  // Switch our pin to SERCOM, begin() set all given pins to the board config.
  if (_sercom.sercom)
    pinPeripheral(_sercom.pin, _sercom.pin_func);

  reset();
}

void V2DMX::reset() {
  while (_spi->isBusy())
    yield();

  // Break + Mark + Start Code '0', LSB first.
  memcpy(_dma_buffer, dma_header, sizeof(dma_header));

  // 64 blocks of 11 bytes, each block containing 8 channel values:
  // one start bit, value '0', two stop bits.
  for (uint8_t i = 0; i < 64; i++)
    memcpy(_dma_buffer->blocks[i], dma_block_init, sizeof(dma_block_init));

  memset(_channels.values, 0, sizeof(_channels.values));
  _n_channels = 0;

  _update_usec = 0;
  _update      = true;
}

static void updateDMABlock(uint8_t block[11], uint8_t channel[8]) {
  block[0]  = dma_block_init[0] | channel[0] << 1;
  block[1]  = dma_block_init[1] | channel[0] >> 7 | channel[1] << 4;
  block[2]  = dma_block_init[2] | channel[1] >> 4 | channel[2] << 7;
  block[3]  = dma_block_init[3] | channel[2] >> 1;
  block[4]  = dma_block_init[4] | channel[3] << 2;
  block[5]  = dma_block_init[5] | channel[3] >> 6 | channel[4] << 5;
  block[6]  = dma_block_init[6] | channel[4] >> 3;
  block[7]  = dma_block_init[7] | channel[5];
  block[8]  = dma_block_init[8] | channel[6] << 3;
  block[9]  = dma_block_init[9] | channel[6] >> 5 | channel[7] << 6;
  block[10] = dma_block_init[10] | channel[7] >> 2;
}

void V2DMX::loop() {
  if (_update) {
    // Refresh the DMA data
    for (uint8_t i = 0; i < 64; i++) {
      // Do not needlessly update the untouched higher channels.
      if (i * 8 > _n_channels)
        break;

      updateDMABlock(_dma_buffer->blocks[i], _channels.blocks[i]);
    }
    _update = false;

  } else {
    // Regularly send our message, regardless if something has changed. Devices
    // switch themselves off when the signal is lost.
    if ((unsigned long)(micros() - _update_usec) < 200 * 1000)
      return;
  }

  if (_spi->isBusy())
    return;

  _spi->transfer(_dma_buffer, NULL, sizeof(DMABuffer), false);
  _update_usec = micros();
}

void V2DMX::setChannels(uint16_t i, const uint8_t *data, uint16_t size) {
  if (i + size >= 512)
    return;

  // Remember the largest channel number in use to limit the DMA data update.
  if (_n_channels < i + size)
    _n_channels = i + size;

  memcpy(_channels.values + i, data, size);
  _update = true;
}

void V2DMX::setChannel(uint16_t i, uint8_t value) {
  setChannels(i, &value, 1);
}
