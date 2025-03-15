#include "LegoSensor.h"

namespace LegoSensor {

// -----------------------------------------------------------------------------
// ModeInfo Implementation
// -----------------------------------------------------------------------------
ModeInfo::ModeInfo() 
  : nameReceived(false), rawReceived(false), pctReceived(false), siReceived(false),
    unitsReceived(false), formatReceived(false),
    rawMin(0), rawMax(1023),
    pctMin(0), pctMax(100),
    siMin(0), siMax(1023),
    dataSets(0), dataFormat(0), figures(0), decimals(0)
{
  name[0] = '\0';
  units[0] = '\0';
}

// -----------------------------------------------------------------------------
// SerialDevice Implementation
// -----------------------------------------------------------------------------
SerialDevice::SerialDevice(HardwareSerial& serialPort)
  : port(serialPort),
    lowSpeedBaud(2400),
    state(State::CONFIRM_LINE_ALIVE),
    lastByteTime(0),
    lastNackTime(0),
    curMessageByteIndex(0),
    messageLength(0),
    xorChecksum(0),
    frameHandler(nullptr),
    frameHandlerContext(nullptr),
    highSpeedActive(false),
    initCycleComplete(false),
    highSpeedBaudRate(0),
    lastError(SensorError::None)
{}

void SerialDevice::begin() {
  port.begin(lowSpeedBaud);
  flushSerialBuffer();
  lastByteTime = millis();
  lastNackTime = millis();
  state = State::CONFIRM_LINE_ALIVE;
  resetFrameParser();
  initCycleComplete = false;
  highSpeedActive = false;
  highSpeedBaudRate = 0;
  lastError = SensorError::None;
}

void SerialDevice::setFrameHandler(FrameHandler handler, void* context) {
  frameHandler = handler;
  frameHandlerContext = context;
}

void SerialDevice::process() {
  unsigned long currentTime = millis();
  switch (state) {
    case State::CONFIRM_LINE_ALIVE:
      if (port.available() > 0) {
        port.read();
        lastByteTime = currentTime;
        state = State::WAIT_FOR_PAUSE;
      }
      break;
    case State::WAIT_FOR_PAUSE:
      while (port.available() > 0) {
        port.read();
        lastByteTime = currentTime;
      }
      if (currentTime - lastByteTime >= kPauseDuration) {
        flushSerialBuffer();
        resetFrameParser();
        state = State::COLLECT_INIT;
      }
      break;
    case State::COLLECT_INIT:
      while (port.available() > 0) {
        byte incomingByte = port.read();
        lastByteTime = currentTime;
        processFrameByte(incomingByte);
      }
      break;
    case State::HIGH_SPEED_DATA:
      while (port.available() > 0) {
        byte incomingByte = port.read();
        lastByteTime = currentTime;
        processFrameByte(incomingByte);
      }
      if (currentTime - lastNackTime >= 100) {
        port.write(BYTE_NACK);
        lastNackTime = currentTime;
      }
      break;
  }
}

void SerialDevice::switchToHighSpeed(unsigned long baudRate) {
  highSpeedActive = true;
  initCycleComplete = false;
  port.write(BYTE_ACK);
  port.flush();
  delay(2);
  port.end();
  delay(2);
  port.begin(baudRate);
}

SensorError SerialDevice::getLastError() const { return lastError; }
void SerialDevice::clearLastError() { lastError = SensorError::None; }

bool SerialDevice::isInitCycleComplete() const { return initCycleComplete; }
bool SerialDevice::HighSpeedActive() const { return highSpeedActive; }

void SerialDevice::sendWithChecksum(byte inarr[], int length) {
  byte chk = 255;
  for (int i = 0; i < length; i++) {
    chk ^= inarr[i];
    port.write(inarr[i]);
  }
  port.write(chk);
}

void SerialDevice::flushSerialBuffer() {
  while (port.available() > 0) {
    port.read();
  }
}

void SerialDevice::resetFrameParser() {
  curMessageByteIndex = 0;
  messageLength = 0;
  xorChecksum = 0;
}

void SerialDevice::handleSyncLoss() {
  state = State::CONFIRM_LINE_ALIVE;
  highSpeedActive = false;
  port.flush();
  port.end();
  port.begin(lowSpeedBaud);
  resetFrameParser();
  initCycleComplete = false;
  lastError = SensorError::SyncLoss;
}

void SerialDevice::processFrameByte(byte data) {
  if (curMessageByteIndex < kInputBufferSize) {
    inputBuffer[curMessageByteIndex] = data;
  } else {
    lastError = SensorError::BufferOverflow;
    resetFrameParser();
    return;
  }
  xorChecksum ^= data;
  if (curMessageByteIndex == 0) {
    byte frameDataLength = 1 << (((data & 0b00111000) >> 3));
    byte frameCmd = (data & 0xC0);
    if (frameCmd == MESSAGE_SYS) {
      messageLength = 1;
    } else if (frameCmd == MESSAGE_INFO) {
      messageLength = frameDataLength + 3;
    } else {
      messageLength = frameDataLength + 2;
    }
  }
  curMessageByteIndex++;
  if (curMessageByteIndex >= messageLength) {
    if (!isChecksumValid()) {
      lastError = SensorError::ChecksumError;
      if (state == State::HIGH_SPEED_DATA) {
        handleSyncLoss();
      }
    } else {
      byte firstByte = inputBuffer[0];
      byte frameCmd = firstByte & 0xC0;
      byte frameMode = firstByte & 0x07;
      if (state == State::COLLECT_INIT && frameCmd == MESSAGE_CMD && frameMode == CMD_SPEED) {
        unsigned long tmpBaud = UlongFromBytes(inputBuffer + 1);
        highSpeedBaudRate = tmpBaud;
      }
      if (state == State::COLLECT_INIT && frameCmd == MESSAGE_SYS && inputBuffer[0] == BYTE_ACK) {
        if (highSpeedBaudRate == 0) { highSpeedBaudRate = 115200; }
        switchToHighSpeed(highSpeedBaudRate);
        state = State::HIGH_SPEED_DATA;
        initCycleComplete = true;
      }
      if (frameHandler) {
        frameHandler(frameHandlerContext, inputBuffer, messageLength, frameCmd, frameMode);
      }
    }
    resetFrameParser();
  }
}

bool SerialDevice::isChecksumValid() { return (xorChecksum == 255) || (messageLength == 1); }

unsigned long SerialDevice::UlongFromBytes(byte bytes[]) {
  return ((unsigned long)bytes[3] << 24) | ((unsigned long)bytes[2] << 16) |
         ((unsigned long)bytes[1] << 8)  | (unsigned long)bytes[0];
}

// -----------------------------------------------------------------------------
// Sensor Implementation
// -----------------------------------------------------------------------------
Sensor::Sensor(HardwareSerial& serialPort)
  : serialDevice(serialPort),
    sensorHighSpeedBaud(115200),
    dataCallback(nullptr),
    currentModeIndex(0),
    SENSOR_TypeId(0),
    SENSOR_NumModes(0),
    SENSOR_ExtMode(0),
    initCompleteCallback(nullptr)
{
  for (int i = 0; i < 16; i++) {
    modeInfos[i] = ModeInfo();
  }
}

void Sensor::begin() {
  serialDevice.begin();
  serialDevice.setFrameHandler(frameCallback, this);
}

void Sensor::process() { serialDevice.process(); }

void Sensor::setMode(byte modeNum) {
  byte arr[2];
  arr[0] = 0x43; // MESSAGE_CMD | CMD_SELECT
  arr[1] = modeNum;
  serialDevice.sendWithChecksum(arr, 2);
}

void Sensor::subscribeDataMessages(DataMessageCallback callback) { dataCallback = callback; }

void Sensor::subscribeInitializationComplete(InitializationCompleteCallback callback, void* context) {
  initCompleteCallback = callback;
  initCompleteCallbackContext = context;
}

bool Sensor::waitForInitialization(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (!isConnected()) {
    process();
    if (millis() - start > timeoutMs) return false;
    delay(5);
  }
  return true;
}

SensorStatus Sensor::getStatus() const {
  SensorStatus status;
  status.deviceType = SENSOR_TypeId;
  status.numModes = SENSOR_NumModes;
  status.extMode = SENSOR_ExtMode;
  status.modeInfos = modeInfos;
  return status;
}

bool Sensor::isConnected() const { return serialDevice.HighSpeedActive(); }

const char* Sensor::getMode(int index) const {
  return (index >= 0 && index < SENSOR_NumModes) ? modeInfos[index].name : "";
}

const ModeInfo* Sensor::getModeInfo(int index) const {
  return (index >= 0 && index < SENSOR_NumModes) ? &modeInfos[index] : nullptr;
}

byte Sensor::getNumModes() const { return SENSOR_NumModes; }

SensorError Sensor::getLastError() const { return serialDevice.getLastError(); }
void Sensor::clearLastError() { serialDevice.clearLastError(); }

void Sensor::frameCallback(void* context, byte* data, int length, byte frameCmd, byte frameMode) {
  Sensor* self = static_cast<Sensor*>(context);
  self->onFrameReceived(data, length, frameCmd, frameMode);
}

void Sensor::onFrameReceived(byte* data, int length, byte frameCmd, byte frameMode) {
  if (frameCmd == MESSAGE_SYS) {
    if (data[0] == BYTE_ACK) {
      if (serialDevice.isInitCycleComplete()) {
        enableHighSpeedMode();
        if (initCompleteCallback) { initCompleteCallback(initCompleteCallbackContext); }
        return;
      }
    }
    return;
  }
  
  switch (frameCmd) {
    case MESSAGE_CMD:   handleCommandMessage(data, length, frameMode); break;
    case MESSAGE_INFO:  handleInfoMessage(data, length, frameMode);  break;
    case MESSAGE_DATA:  handleDataMessage(data, length, frameMode);  break;
    default: break;
  }
}

unsigned long Sensor::UlongFromBytes(byte bytes[]) {
  return ((unsigned long)bytes[3] << 24) | ((unsigned long)bytes[2] << 16) |
         ((unsigned long)bytes[1] << 8)  | (unsigned long)bytes[0];
}

void Sensor::setHighSpeedBaud(unsigned long baud) { sensorHighSpeedBaud = baud; }

void Sensor::handleCommandMessage(byte* data, int length, byte frameMode) {
  switch (frameMode) {
    case CMD_TYPE:
      SENSOR_TypeId = data[1];
      break;
    case CMD_MODES:
      SENSOR_NumModes = data[1] + 1;
      break;
    case CMD_SPEED: {
      unsigned long tmpBaud = UlongFromBytes(data + 1);
      setHighSpeedBaud(tmpBaud);
      break;
    }
    case CMD_EXT_MODE:
      SENSOR_ExtMode = data[1];
      break;
    default: break;
  }
}

void Sensor::handleInfoMessage(byte* data, int length, byte frameMode) {
  uint8_t actualMode = frameMode;
  if (data[1] & INFO_MODE_PLUS_8) { actualMode += 8; }
  uint8_t infoType = data[1] & ~INFO_MODE_PLUS_8;
  int offset = ((data[1] & INFO_MODE_PLUS_8) ? 3 : 2);
  
  switch(infoType) {
    case INFO_NAME: {
      int nameLen = length - offset - 1;
      if (nameLen > 11) nameLen = 11;
      strncpy(modeInfos[actualMode].name, (const char*)(data + offset), nameLen);
      modeInfos[actualMode].name[nameLen] = '\0';
      modeInfos[actualMode].nameReceived = true;
      break;
    }
    case INFO_RAW:
      if (length >= offset + 8 + 1) {
        memcpy(&modeInfos[actualMode].rawMin, data + offset, 4);
        memcpy(&modeInfos[actualMode].rawMax, data + offset + 4, 4);
        modeInfos[actualMode].rawReceived = true;
      }
      break;
    case INFO_PCT:
      if (length >= offset + 8 + 1) {
        memcpy(&modeInfos[actualMode].pctMin, data + offset, 4);
        memcpy(&modeInfos[actualMode].pctMax, data + offset + 4, 4);
        modeInfos[actualMode].pctReceived = true;
      }
      break;
    case INFO_SI:
      if (length >= offset + 8 + 1) {
        memcpy(&modeInfos[actualMode].siMin, data + offset, 4);
        memcpy(&modeInfos[actualMode].siMax, data + offset + 4, 4);
        modeInfos[actualMode].siReceived = true;
      }
      break;
    case INFO_UNITS: {
      int unitLen = length - offset - 1;
      if (unitLen > 4) unitLen = 4;
      strncpy(modeInfos[actualMode].units, (const char*)(data + offset), unitLen);
      modeInfos[actualMode].units[unitLen] = '\0';
      modeInfos[actualMode].unitsReceived = true;
      break;
    }
    case INFO_FORMAT:
      if (length >= offset + 4 + 1) {
        modeInfos[actualMode].dataSets = data[offset];
        modeInfos[actualMode].dataFormat = data[offset+1];
        modeInfos[actualMode].figures = data[offset+2];
        modeInfos[actualMode].decimals = data[offset+3];
        modeInfos[actualMode].formatReceived = true;
      }
      break;
    default: break;
  }
}

void Sensor::handleDataMessage(byte* data, int length, byte frameMode) {
  char jsonDataMessage[50];
  switch (frameMode) {
    case 0: {
      strcpy(jsonDataMessage, "{\"color\":\"");
      switch (data[1]) {
        case 0: strcat(jsonDataMessage, "Black"); break;
        case 3: strcat(jsonDataMessage, "Blue"); break;
        case 5: strcat(jsonDataMessage, "Green"); break;
        case 7: strcat(jsonDataMessage, "Yellow"); break;
        case 9: strcat(jsonDataMessage, "Red"); break;
        case 10: strcat(jsonDataMessage, "White"); break;
        case 255: strcat(jsonDataMessage, "Too Far"); break;
        default: strcat(jsonDataMessage, "???"); break;
      }
      strcat(jsonDataMessage, "\"}");
      break;
    }
    case 1:
      sprintf(jsonDataMessage, "{\"distance\":%d}", data[1]);
      break;
    case 2:
      sprintf(jsonDataMessage, "{\"counter\":%lu}", UlongFromBytes(data + 1));
      break;
    case 3:
      sprintf(jsonDataMessage, "{\"reflected_light\":%d}", data[1]);
      break;
    case 4:
      sprintf(jsonDataMessage, "{\"ambient_light\":%d}", data[1]);
      break;
    case 5:
      sprintf(jsonDataMessage, "{\"write_color_mode\":%d}", data[1]);
      break;
    case 6:
      sprintf(jsonDataMessage, "{\"rgbi_mode\":%d}", data[1]);
      break;
    case 7:
      sprintf(jsonDataMessage, "{\"ir_tx\":%d}", data[1]);
      break;
    default:
      sprintf(jsonDataMessage, "{\"unknownmode\":%d}", data[1]);
      break;
  }
  if (dataCallback) { dataCallback(jsonDataMessage, strlen(jsonDataMessage)); }
}

void Sensor::enableHighSpeedMode() { serialDevice.switchToHighSpeed(sensorHighSpeedBaud); }

} // end namespace LegoSensor
