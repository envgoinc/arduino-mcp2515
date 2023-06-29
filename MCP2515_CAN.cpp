
#include <stdio.h>
#include <ctype.h>

#include "MCP2515_CAN.h"

using namespace std;

MCP2515_CAN::MCP2515_CAN(uint8_t cs, const uint32_t spi_clock, RXQUEUE_TABLE rxSize){
  _cs = cs;
  _state = BUS_ACTIVE;
  _mcp2515 = new MCP2515(_cs, spi_clock);

  sizeRxBuffer=rxSize;
}

MCP2515_CAN::MCP2515_CAN(uint8_t cs, RXQUEUE_TABLE rxSize){
  _cs = cs;
  _state = BUS_ACTIVE;
  _mcp2515 = new MCP2515(_cs);

  sizeRxBuffer=rxSize;
}

void MCP2515_CAN::begin(){
  MCP2515::ERROR error;

  initializeBuffers();

  error = _mcp2515->reset();
  if(error != MCP2515::ERROR_OK){
      setError(ERROR_MCP2515_INIT);
      return;
  }
  error = _mcp2515->setConfigMode();
  if( error != MCP2515::ERROR_OK){
      setError(ERROR_MCP2515_CONFIG);
      return;
  }
  error = _mcp2515->setClkOut(CLKOUT_DIV1);
  if( error != MCP2515::ERROR_OK){
      setError(ERROR_MCP2515_INIT);
      return;
  }

}

void MCP2515_CAN::setError(CAN_ERROR error){
  _err = error;
}

MCP2515_CAN::CAN_ERROR MCP2515_CAN::getError(){
  return _err;
}

void MCP2515_CAN::setClock(CAN_CLOCK clock){
  _canClock = clock;
}
void MCP2515_CAN::setBaudRate(CAN_SPEED speed){
  _bitrate = speed;
}

bool MCP2515_CAN::read(can_frame &CAN_rx_msg){
  bool ret;
  ret = removeFromRingBuffer(rxRing, CAN_rx_msg);
  return ret;
}

bool MCP2515_CAN::write(const can_frame &CAN_tx_msg, bool sendMB){
  (void*) sendMB;
  bool ret = true;
  if (_mcp2515->sendMessage(&CAN_tx_msg) != MCP2515::ERROR_OK) {
      ret = false;
  }

  return ret;
}

bool MCP2515_CAN::setFilter(const uint32_t filter){
  bool ret = true;

  MCP2515::RXF filters[] = {MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2, MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5};
  for (int i=0; i<6; i++) {
    MCP2515::ERROR result = _mcp2515->setFilter(filters[i], false, filter);
    if (result != MCP2515::ERROR_OK) {
      ret = false;
    }
  }
  
  return ret;
}

bool MCP2515_CAN::setFilterMask(const uint32_t mask){
  bool ret = true;

  MCP2515::MASK masks[] = {MCP2515::MASK0, MCP2515::MASK1};
  for (int i=0; i<2; i++) {
    MCP2515::ERROR result = _mcp2515->setFilterMask(masks[i], false, mask);
    if (result != MCP2515::ERROR_OK) {
        ret = false;
      }
  }

  return ret;
}

void MCP2515_CAN::setListenOnly(bool listenOnly){
  _listenOnly = listenOnly; 
}

void MCP2515_CAN::enableLoopBack(bool yes){
  if(yes){
    _loopback = true;
  }
  else{
    _loopback = false;
  }
}

MCP2515_CAN::CAN_ERROR MCP2515_CAN::connectCan(){
  MCP2515::ERROR error = _mcp2515->setBitrate(_bitrate, _canClock);
  if (error != MCP2515::ERROR_OK) {
    setError(ERROR_MCP2515_BITRATE);
    return ERROR_MCP2515_BITRATE;
  }

  if (_loopback) {
      error = _mcp2515->setLoopbackMode();
  } else if (_listenOnly) {
      error = _mcp2515->setListenOnlyMode();
  } else {
      error = _mcp2515->setNormalMode();
  }

  if (error != MCP2515::ERROR_OK) {
    setError(ERROR_MCP2515_SET_MODE);
    return (ERROR_MCP2515_SET_MODE);
  }

  _isConnected = true;
  setError(ERROR_OK);
  return ERROR_OK;
}

MCP2515_CAN::CAN_ERROR MCP2515_CAN::disconnectCan(){
  _isConnected = false;
  _mcp2515->setConfigMode();
  setError(ERROR_OK);
  return ERROR_OK;
}

bool MCP2515_CAN::isConnected(){
  return _isConnected;
}

MCP2515_CAN::CAN_ERROR MCP2515_CAN::checkErrorCounter() {
  char out[15];
  CAN_ERROR error;
  const uint8_t errorCount = _mcp2515->errorCountRX();

  // check for an increase in receive error count.  If there is, send
  // error frame.
  if(_rxErrorCount != errorCount) {
    _rxErrorCount = errorCount;
    error = createErrorStatus('s', out, sizeof(out));
    if (error != ERROR_OK) {
        return error;
    }
  }
  return ERROR_OK;
}


MCP2515_CAN::CAN_ERROR MCP2515_CAN::createErrorStatus(const char error, char *buffer, const int length) {
    uint16_t i = 0;
    if (length < 5) {
        return ERROR_BUFFER_OVERFLOW;
    }
    buffer[i++] = 'e';
    buffer[i++] = '1';
    buffer[i++] = error;
    buffer[i++] = CR;
    buffer[i] = '\0';
    return ERROR_OK;
}

MCP2515_CAN::CAN_ERROR MCP2515_CAN::createBusState(const char state, char *buffer, const int length) {
    uint16_t i = 0;
    uint8_t rxErrorCount, txErrorCount;
    if (length < 10) {
        return ERROR_BUFFER_OVERFLOW;
    }
    rxErrorCount = _mcp2515->errorCountRX();
    txErrorCount = _mcp2515->errorCountTX();
    buffer[i++] = 's';
    buffer[i++] = state;
    buffer[i++] = CR;
    buffer[i] = '\0';
    return ERROR_OK;
}


void MCP2515_CAN::processError() {
  uint8_t errFlags = _mcp2515->getErrorFlags();
  char out[15];
  CAN_ERROR error;
  CAN_STATE state;
  char state_char;

  if (errFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
      _mcp2515->clearRXnOVRFlags();
      error = createErrorStatus('o', out, sizeof(out));
      if (error != (CAN_ERROR)ERROR_OK) {
        return;
      }
  }

  state = _mcp2515->getBusState();
  if (state != _state) {
    _state = state;
    switch (state) {
      case BUS_ACTIVE:
        state_char = 'a';
        break;
      case BUS_PASSIVE:
        state_char = 'p';
        break;
      case BUS_WARN:
        state_char = 'w';
        break;
      case BUS_OFF:
        state_char = 'f';
        break;
      default:
        break;
    }
    error = createBusState(state_char, out, sizeof(out));
    if (error != ERROR_OK) {
        return;
    }
  }
  return;
}


void MCP2515_CAN::pollReceiveCan(){
  if (!isConnected()) {
    setError(ERROR_NOT_CONNECTED);
    return;
  }

  while (_mcp2515->checkReceive()) {
    can_frame frame;
    if (_mcp2515->readMessage(&frame) != MCP2515::ERROR_OK) {
        setError(ERROR_MCP2515_READ);
        return;
    }
    addToRingBuffer(rxRing, frame);
  }
    setError(ERROR_OK);
}

void MCP2515_CAN::processInterrupt(){
  bool msg_received = false;

  if (!this->isConnected()) {
    setError(ERROR_NOT_CONNECTED);
    return;
  }

  uint8_t irq = _mcp2515->getInterrupts();

  if (irq & MCP2515::CANINTF_ERRIF) {
      setError(ERROR_MCP2515_ERRIF);
      processError();
      _mcp2515->clearERRIF();
  }

  if (irq & MCP2515::CANINTF_RX0IF) {
      CAN_ERROR error = receiveCan(MCP2515::RXB0);
      if (error != ERROR_OK) {
        setError(error);
        return;
      } else {
        msg_received = true;
      }
  }

  if (irq & MCP2515::CANINTF_RX1IF) {
      CAN_ERROR error = receiveCan(MCP2515::RXB1);
      if (error != ERROR_OK) {
        setError(error);
        return;
      } else {
          msg_received = true;
      }
  }

  if (msg_received) {
    checkErrorCounter();
  }

  if (irq & MCP2515::CANINTF_WAKIF) {
      setError(ERROR_MCP2515_WAKIF);
      _mcp2515->clearWAKIF();
  }

  if (irq & MCP2515::CANINTF_MERRF) {
      setError(ERROR_MCP2515_MERRF);
      _mcp2515->clearMERR();
  }

  setError(ERROR_OK);
}


MCP2515_CAN::CAN_ERROR MCP2515_CAN::receiveCan(const MCP2515::RXBn rxBuffer){
  if (!isConnected()) {
      return ERROR_NOT_CONNECTED;
  }

  can_frame frame;
  MCP2515::ERROR result = _mcp2515->readMessage(rxBuffer, &frame);
  if (result == MCP2515::ERROR_NOMSG) {
      return ERROR_OK;
  }
  if (result != MCP2515::ERROR_OK) {
      return ERROR_MCP2515_READ;
  }

  addToRingBuffer(rxRing, frame);
  return ERROR_OK;
}



bool MCP2515_CAN::addToRingBuffer(RingbufferTypeDef &ring, const can_frame &msg){
  uint16_t nextEntry = (ring.head + 1) % ring.size;

  // check if the ring buffer is full
  if(nextEntry == ring.tail)
  {
    return(false);
  }

  // add the element to the ring */
  memcpy((void *)&ring.buffer[ring.head],(void *)&msg, sizeof(can_frame));

  // bump the head to point to the next free entry
  ring.head = nextEntry;

  return(true);
}

bool MCP2515_CAN::removeFromRingBuffer(RingbufferTypeDef &ring, can_frame &msg){
  // check if the ring buffer has data available
  if(isRingBufferEmpty(ring) == true)
  {
      return(false);
  }

  // copy the message
  memcpy((void *)&msg,(void *)&ring.buffer[ring.tail], sizeof(can_frame));
  //printHook(&ring.buffer[ring.tail]); 
  // bump the tail pointer
  ring.tail =(ring.tail + 1) % ring.size;
  return(true);
}

void MCP2515_CAN::initializeBuffers(void){
    if(isInitialized()) { return; }

    if(rx_buffer==0)
    {
      rx_buffer = new can_frame[sizeRxBuffer];
    }
    initRingBuffer(rxRing, rx_buffer, sizeRxBuffer);
}

void MCP2515_CAN::initRingBuffer(RingbufferTypeDef &ring, volatile can_frame *buffer, uint32_t size){
    ring.buffer = buffer;
    ring.size = size;
    ring.head = 0;
    ring.tail = 0;
}

bool MCP2515_CAN::isRingBufferEmpty(RingbufferTypeDef &ring){
  if(ring.head == ring.tail)
	{
    return(true);
  }

  return(false);

}
uint32_t MCP2515_CAN::ringBufferCount(RingbufferTypeDef &ring){
  int32_t entries;
  entries = ring.head - ring.tail;

  if(entries < 0)
  {
      entries += ring.size;
  }
  return((uint32_t)entries);
}
