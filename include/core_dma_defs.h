
#pragma once

namespace sioc::dma {

  typedef void (*transferCB_t)(const int);
  typedef void (*errorCB_t)(const int, const channel_error);

  enum class crc_error {

  };

  enum class channel_error {

  };

  enum class channel_peripheral {

  };

}

