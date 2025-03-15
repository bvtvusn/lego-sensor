#ifndef LEGO_SENSOR_H
#define LEGO_SENSOR_H

#include <Arduino.h>

namespace LegoSensor {

// -----------------------------------------------------------------------------
// Protocol Constant Definitions
// -----------------------------------------------------------------------------
constexpr byte MESSAGE_SYS  = 0x00;   // System message (0b00 << 6)
constexpr byte MESSAGE_CMD  = 0x40;   // Command message (0b01 << 6)
constexpr byte MESSAGE_INFO = 0x80;   // Info message (0b10 << 6)
constexpr byte MESSAGE_DATA = 0xC0;   // Data message (0b11 << 6)

constexpr byte LENGTH_1   = 0x00;     // 1 byte  (0b000 << 3)
constexpr byte LENGTH_2   = 0x08;     // 2 bytes (0b001 << 3)
constexpr byte LENGTH_4   = 0x10;     // 4 bytes (0b010 << 3)
constexpr byte LENGTH_8   = 0x18;     // 8 bytes (0b011 << 3)
constexpr byte LENGTH_16  = 0x20;     // 16 bytes (0b100 << 3)
constexpr byte LENGTH_32  = 0x28;     // 32 bytes (0b101 << 3)

constexpr byte BYTE_SYNC = 0x00;      // Synchronization byte
constexpr byte BYTE_NACK = 0x02;      // Not acknowledge (keep-alive) byte
constexpr byte BYTE_ACK  = 0x04;      // Acknowledge byte

// Command message IDs
constexpr byte CMD_TYPE     = 0x00;
constexpr byte CMD_MODES    = 0x01;
constexpr byte CMD_SPEED    = 0x02;
constexpr byte CMD_SELECT   = 0x03;
constexpr byte CMD_WRITE    = 0x04;
constexpr byte CMD_EXT_MODE = 0x06;
constexpr byte CMD_VERSION  = 0x07;

// Info message IDs
constexpr byte INFO_NAME        = 0x00;
constexpr byte INFO_RAW         = 0x01;
constexpr byte INFO_PCT         = 0x02;
constexpr byte INFO_SI          = 0x03;
constexpr byte INFO_UNITS       = 0x04;
constexpr byte INFO_MAPPING     = 0x05;
constexpr byte INFO_MODE_COMBOS = 0x06;
constexpr byte INFO_UNK7        = 0x07;
constexpr byte INFO_UNK8        = 0x08;
constexpr byte INFO_UNK9        = 0x09;
constexpr byte INFO_UNK10       = 0x0a;
constexpr byte INFO_UNK11       = 0x0b;
constexpr byte INFO_UNK12       = 0x0c;
constexpr byte INFO_MODE_PLUS_8 = 0x20;
constexpr byte INFO_FORMAT      = 0x80;

// INFO_FORMAT data formats
constexpr byte DATA8  = 0x00;
constexpr byte DATA16 = 0x01;
constexpr byte DATA32 = 0x02;
constexpr byte DATAF  = 0x03;

// -----------------------------------------------------------------------------
// Error Handling
// -----------------------------------------------------------------------------
enum class SensorError {
  None = 0,
  BufferOverflow,
  ChecksumError,
  SyncLoss,
  Timeout
};

// -----------------------------------------------------------------------------
// Callback Type Definitions
// -----------------------------------------------------------------------------
typedef void (*FrameHandler)(void* context, byte* data, int length, byte frameCmd, byte frameMode);
typedef void (*DataMessageCallback)(char* data, int length);
typedef void (*InitializationCompleteCallback)(void* context);

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------
struct ModeInfo {
  char name[12];       // Up to 11 characters plus null terminator
  bool nameReceived;
  bool rawReceived;
  float rawMin;
  float rawMax;
  bool pctReceived;
  float pctMin;
  float pctMax;
  bool siReceived;
  float siMin;
  float siMax;
  bool unitsReceived;
  char units[5];       // Up to 4 characters plus null terminator
  bool formatReceived;
  uint8_t dataSets;
  uint8_t dataFormat;
  uint8_t figures;
  uint8_t decimals;
  
  ModeInfo();
};

struct SensorStatus {
  byte deviceType;
  byte numModes;
  byte extMode;
  const ModeInfo* modeInfos; // Array of ModeInfo (size == numModes)
};

// -----------------------------------------------------------------------------
// SerialDevice Class (Low-Level Serial Communication)
// -----------------------------------------------------------------------------
class SerialDevice {
public:
  SerialDevice(HardwareSerial& serialPort);
  void begin();
  void setFrameHandler(FrameHandler handler, void* context);
  void process();
  void switchToHighSpeed(unsigned long baudRate);
  bool isInitCycleComplete() const;
  bool HighSpeedActive() const;
  void sendWithChecksum(byte inarr[], int length);
  
  SensorError getLastError() const;
  void clearLastError();
  
private:
  enum class State {
    CONFIRM_LINE_ALIVE,
    WAIT_FOR_PAUSE,
    COLLECT_INIT,
    HIGH_SPEED_DATA
  };
  
  HardwareSerial& port;
  unsigned long lowSpeedBaud;
  State state;
  unsigned long lastByteTime;
  unsigned long lastNackTime;
  bool highSpeedActive;
  bool initCycleComplete;
  unsigned long highSpeedBaudRate;
  
  static const unsigned long kPauseDuration = 300UL;
  static const int kInputBufferSize = 100;
  
  byte inputBuffer[kInputBufferSize];
  int curMessageByteIndex;
  int messageLength;
  byte xorChecksum;
  
  FrameHandler frameHandler;
  void* frameHandlerContext;
  
  SensorError lastError;
  
  void flushSerialBuffer();
  void resetFrameParser();
  void handleSyncLoss();
  void processFrameByte(byte data);
  bool isChecksumValid();
  unsigned long UlongFromBytes(byte bytes[]);
};

// -----------------------------------------------------------------------------
// Sensor Class (High-Level Sensor API)
// -----------------------------------------------------------------------------
class Sensor {
public:
  // The constructor now accepts a reference to a HardwareSerial.
  // The default is Serial1 if no port is specified.
  Sensor(HardwareSerial& serialPort = Serial1);
  void begin();
  void process();
  void setMode(byte modeNum);
  
  void subscribeDataMessages(DataMessageCallback callback);
  void subscribeInitializationComplete(InitializationCompleteCallback callback, void* context);
  
  bool waitForInitialization(unsigned long timeoutMs);
  SensorStatus getStatus() const;
  bool isConnected() const;
  const char* getMode(int index) const;
  const ModeInfo* getModeInfo(int index) const;
  byte getNumModes() const;
  
  // Expose error handling from the underlying SerialDevice.
  SensorError getLastError() const;
  void clearLastError();
  
private:
  SerialDevice serialDevice;
  unsigned long sensorHighSpeedBaud;
  byte SENSOR_TypeId;
  byte SENSOR_NumModes;
  byte SENSOR_ExtMode;
  ModeInfo modeInfos[16];
  DataMessageCallback dataCallback;
  byte currentModeIndex;
  
  InitializationCompleteCallback initCompleteCallback;
  void* initCompleteCallbackContext;
  
  static void frameCallback(void* context, byte* data, int length, byte frameCmd, byte frameMode);
  void onFrameReceived(byte* data, int length, byte frameCmd, byte frameMode);
  unsigned long UlongFromBytes(byte bytes[]);
  void setHighSpeedBaud(unsigned long baud);
  void handleCommandMessage(byte* data, int length, byte frameMode);
  void handleInfoMessage(byte* data, int length, byte frameMode);
  void handleDataMessage(byte* data, int length, byte frameMode);
  void enableHighSpeedMode();
};

} // namespace LegoSensor

#endif // LEGO_SENSOR_H
