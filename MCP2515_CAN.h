/*
  Wrapper for mcp2515 driver
*/

#ifndef MCP2515_CAN_H_
#define MCP2515_CAN_H_

#include <can.h>
#include <mcp2515.h>
#include <Arduino.h>

class MCP2515_CAN{
  public:

    typedef enum RXQUEUE_TABLE {
      RX_SIZE_2 = (uint8_t)2,
      RX_SIZE_4 = (uint8_t)4,
      RX_SIZE_8 = (uint8_t)8,
      RX_SIZE_16 = (uint8_t)16,
      RX_SIZE_32 = (uint8_t)32,
      RX_SIZE_64 = (uint8_t)64,
      RX_SIZE_128 = (uint8_t)128,
      RX_SIZE_256 = (uint8_t)256,
    } RXQUEUE_TABLE;

    typedef enum CAN_ERROR{
      ERROR_OK,
      ERROR_CONNECTED,
      ERROR_NOT_CONNECTED,
      ERROR_UNKNOWN_COMMAND,
      ERROR_INVALID_COMMAND,
      ERROR_ERROR_FRAME_NOT_SUPPORTED,
      ERROR_BUFFER_OVERFLOW,
      ERROR_SERIAL_TX_OVERRUN,
      ERROR_LISTEN_ONLY,
      ERROR_MCP2515_INIT,
      ERROR_MCP2515_CONFIG,
      ERROR_MCP2515_BITRATE,
      ERROR_MCP2515_SET_MODE,
      ERROR_MCP2515_SEND,
      ERROR_MCP2515_READ,
      ERROR_MCP2515_FILTER,
      ERROR_MCP2515_ERRIF,
      ERROR_MCP2515_MERRF,
      ERROR_MCP2515_WAKIF
    }CAN_ERROR;

    typedef struct can_frame can_frame;
    typedef enum CAN_CLOCK CAN_CLOCK;

    // Default buffer size is set to 16. But this can be changed by using constructor in main code.
    MCP2515_CAN(uint8_t cs, const uint32_t spi_clock = 0, RXQUEUE_TABLE rxSize = RX_SIZE_8);
    MCP2515_CAN(uint8_t cs, RXQUEUE_TABLE rxSize = RX_SIZE_8);
    void begin();
    CAN_ERROR getError();
    void setClock(CAN_CLOCK clock);
    void setBaudRate(CAN_SPEED speed);
    bool read(can_frame &CAN_rx_msg);
    bool write(const can_frame &CAN_tx_msg, bool sendMB = false);
    // Manually set filter and mask parameters
    bool setFilter(const uint32_t filter);
    bool setFilterMask(const uint32_t mask);
    void setListenOnly(bool listenOnly);
    void enableLoopBack(bool yes = 1);
    void processInterrupt();
    CAN_ERROR connectCan();
    CAN_ERROR disconnectCan();
    bool isConnected();
  
    // These are public because these are also used from interrupts.
    typedef struct RingbufferTypeDef {
      volatile uint8_t head = 0;
      volatile uint8_t tail = 0;
      volatile uint8_t size = 0;
      volatile can_frame *buffer = nullptr;
    } RingbufferTypeDef;

    RingbufferTypeDef rxRing{};

    bool addToRingBuffer(RingbufferTypeDef &ring, const can_frame &msg);
    bool removeFromRingBuffer(RingbufferTypeDef &ring, can_frame &msg);

    volatile can_frame *rx_buffer = nullptr;

  protected:
    uint8_t sizeRxBuffer = 0;
  
  private:
    MCP2515 *_mcp2515 = nullptr;
    uint8_t _cs = 0;
    CAN_STATE _state = BUS_OFF;
    CAN_ERROR _err = ERROR_OK;
    CAN_CLOCK _canClock = MCP_16MHZ;
    CAN_SPEED _bitrate = CAN_1000KBPS;
    bool _timestampEnabled = false;
    bool _listenOnly = false;
    bool _loopback = false;
    bool _isConnected = false;
    uint8_t _rxErrorCount = 0;
    static const char CR  = '\r';
    static const char BEL = 7;
    static const uint16_t TIMESTAMP_LIMIT = 0xEA60;

    void setError(CAN_ERROR error);
    bool isInitialized() { return rx_buffer != nullptr; }
    void initRingBuffer(RingbufferTypeDef &ring, volatile can_frame *buffer, uint8_t size);
    void initializeBuffers(void);
    bool isRingBufferEmpty(RingbufferTypeDef &ring);
    uint32_t ringBufferCount(RingbufferTypeDef &ring);

    CAN_ERROR createBusState(const char state, char *buffer, const int length);
    CAN_ERROR createErrorStatus(const char error, char *buffer, const int length);
    CAN_ERROR checkErrorCounter();
    void pollReceiveCan();
    void processError();
    CAN_ERROR receiveCan(const MCP2515::RXBn rxBuffer);
};




#endif //MCP2515_CAN_H_