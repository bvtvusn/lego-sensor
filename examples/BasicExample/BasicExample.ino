#include <LegoSensor.h>
using namespace LegoSensor;

// This function prints all collected mode information.
void printAllModeInfo(const SensorStatus& status) {
  Serial.println("Collected Mode Information:");
  for (int i = 0; i < status.numModes; i++) {
    Serial.print("Mode ");
    Serial.print(i);
    Serial.print(": Name=");
    Serial.print(status.modeInfos[i].name);
    if (status.modeInfos[i].rawReceived) {
      Serial.print(", Raw=[");
      Serial.print(status.modeInfos[i].rawMin);
      Serial.print(",");
      Serial.print(status.modeInfos[i].rawMax);
      Serial.print("]");
    }
    if (status.modeInfos[i].pctReceived) {
      Serial.print(", Pct=[");
      Serial.print(status.modeInfos[i].pctMin);
      Serial.print(",");
      Serial.print(status.modeInfos[i].pctMax);
      Serial.print("]");
    }
    if (status.modeInfos[i].siReceived) {
      Serial.print(", SI=[");
      Serial.print(status.modeInfos[i].siMin);
      Serial.print(",");
      Serial.print(status.modeInfos[i].siMax);
      Serial.print("]");
    }
    if (status.modeInfos[i].unitsReceived) {
      Serial.print(", Units=");
      Serial.print(status.modeInfos[i].units);
    }
    if (status.modeInfos[i].formatReceived) {
      Serial.print(", Format: DataSets=");
      Serial.print(status.modeInfos[i].dataSets);
      Serial.print(", DataFormat=");
      Serial.print(status.modeInfos[i].dataFormat);
      Serial.print(", Figures=");
      Serial.print(status.modeInfos[i].figures);
      Serial.print(", Decimals=");
      Serial.print(status.modeInfos[i].decimals);
    }
    Serial.println();
  }
}

// Create a sensor object that uses a user-chosen serial port.
// For example, use Serial (the USB serial port) instead of Serial1.
Sensor sensor(Serial1);

unsigned long lastModeSwitchTime = 0;
int currentMode = 0;

void myDataCallback(char* data, int length) {
  Serial.print("Data callback: ");
  Serial.println(data);
}

void myInitializationCompleteCallback(void* context) {
  Serial.println("Handshake complete! All mode data is now available.");
  SensorStatus status = sensor.getStatus();
  printAllModeInfo(status);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  sensor.begin();
  sensor.subscribeDataMessages(myDataCallback);
  sensor.subscribeInitializationComplete(myInitializationCompleteCallback, nullptr);
  Serial.println("Sensor started.");
  lastModeSwitchTime = millis();
  
  if (sensor.waitForInitialization(10000)) {
    Serial.println("Sensor initialization complete.");
    SensorStatus status = sensor.getStatus();
    Serial.print("Device Type: ");
    Serial.println(status.deviceType);
    Serial.print("Number of Modes: ");
    Serial.println(status.numModes);
  } else {
    Serial.println("Sensor initialization timed out.");
  }
}

void loop() {
  sensor.process();
  
  if (millis() - lastModeSwitchTime > 10000) {
    lastModeSwitchTime = millis();
    currentMode = (currentMode + 1) % max(1, (int)sensor.getNumModes());
    sensor.setMode(currentMode);
    Serial.print("Switching to mode ");
    Serial.print(currentMode);
    Serial.print(" : ");
    Serial.println(sensor.getMode(currentMode));
    
    SensorError err = sensor.getLastError();
    if (err != SensorError::None) {
      Serial.print("Error occurred: ");
      switch (err) {
        case SensorError::BufferOverflow: Serial.println("Buffer Overflow"); break;
        case SensorError::ChecksumError: Serial.println("Checksum Error"); break;
        case SensorError::SyncLoss: Serial.println("Sync Loss"); break;
        case SensorError::Timeout: Serial.println("Timeout"); break;
        default: Serial.println("Unknown Error"); break;
      }
      sensor.clearLastError();
    }
  }
}
