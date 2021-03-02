#ifndef SCSFILTER_H_
#define SCSFILTER_H_

#include <stdbool.h>
#include <stdint.h>

constexpr int telegramLen = 7;

typedef struct {
  // circular buffer for incoming bytes, indexed using cur
  uint8_t buf[telegramLen];
  int cur;

  uint8_t tbuf[telegramLen];
} scsfilter;

void sf_WriteByte(scsfilter *sf, uint8_t b) {
  sf->buf[sf->cur] = b;
  sf->cur = (sf->cur + 1) % telegramLen;
}

bool sf_completeAndValid(scsfilter *sf) {
  const uint8_t prev = sf->buf[(sf->cur + (telegramLen - 1)) % telegramLen];
  if (prev != 0xa3) {
    return false; // incomplete: previous byte not a telegram termination
  }

  // Copy the whole telegram into tbuf; this makes the following checksum code
  // simpler and is required when Telegram() returns anyway:
  for (int i = 0; i < telegramLen; i++) {
    sf->tbuf[i] = sf->buf[(sf->cur + i) % telegramLen];
  }

  const uint8_t stored = sf->tbuf[5];
  const uint8_t computed =
      sf->tbuf[1] ^ sf->tbuf[2] ^ sf->tbuf[3] ^ sf->tbuf[4];
  if (stored != computed) {
    return false; // incomplete: checksum mismatch
  }

  return true;
}

// sf_ringForApartment must only be called after calling sf_completeAndValid.
int sf_ringForApartment(scsfilter *sf) {
  if (sf->tbuf[3] != 0x60) {
    return -1; // not a ring command
  }

  if (sf->tbuf[1] != 0x91) {
    return -1; // not sent by the intercom house station
  }

  return (int)(sf->tbuf[2]); // apartment id
}

#endif
