#include <SPI.h>
#include <WiFiNINA.h>
#include <LegoSensor.h>
using namespace LegoSensor;

//-----------------------------------------------------
// Debug Macros
//-----------------------------------------------------
#define DEBUG_ENABLED true
#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

//-----------------------------------------------------
// WiFi Credentials (update these for your network)
//-----------------------------------------------------
char ssid[] = "mySSID";
char pass[] = "mypassword"; 

//-----------------------------------------------------
// Global Web Server on port 80
//-----------------------------------------------------
WiFiServer server(80);

//-----------------------------------------------------
// HTML Page stored in PROGMEM
//-----------------------------------------------------
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Sensor Control Panel</title>
  <style>
    body { font-family: sans-serif; margin: 20px; }
    textarea { width: 100%; height: 150px; margin-bottom: 20px; }
    select, button { padding: 8px; font-size: 16px; }
    h1, h2 { color: #333; }
  </style>
</head>
<body>
  <h1>Sensor Control Panel</h1>
  <div>
    <label for="modeSelect">Select Sensor Mode:</label>
    <select id="modeSelect"></select>
    <button onclick="setSensorMode()">Set Mode</button>
  </div>
  <div>
    <h2>Sensor Modes JSON</h2>
    <textarea id="modesBox" readonly></textarea>
  </div>
  <div>
    <h2>Sensor Data JSON</h2>
    <textarea id="dataBox" readonly></textarea>
  </div>
  <script>
    function fetchModes() {
      fetch('/api/sensor/modes')
        .then(response => response.json())
        .then(json => {
          // Update the text area with JSON data
          document.getElementById('modesBox').value = JSON.stringify(json, null, 2);
          // Populate the dropdown for mode selection
          const select = document.getElementById('modeSelect');
          select.innerHTML = "";
          json.modes.forEach(mode => {
            let option = document.createElement('option');
            option.value = mode.index;
            option.text = mode.index + " - " + mode.name;
            select.appendChild(option);
          });
        })
        .catch(err => console.error("Error fetching modes:", err));
    }
    
    function fetchSensorData() {
      fetch('/api/sensor/data')
        .then(response => response.json())
        .then(json => {
          document.getElementById('dataBox').value = JSON.stringify(json, null, 2);
        })
        .catch(err => console.error("Error fetching sensor data:", err));
    }
    
    function setSensorMode() {
      const mode = document.getElementById('modeSelect').value;
      fetch('/api/sensor/mode?mode=' + mode)
        .then(response => response.json())
        .then(json => {
          alert("Sensor mode set to " + mode);
          // Optionally, refresh modes after setting
          fetchModes();
        })
        .catch(err => console.error("Error setting sensor mode:", err));
    }
    
    // Refresh modes every 5 seconds and sensor data every second.
    setInterval(fetchModes, 5000);
    setInterval(fetchSensorData, 1000);
    fetchModes();
    fetchSensorData();
  </script>
</body>
</html>
)rawliteral";

//-----------------------------------------------------
// Global sensor object and data storage
//-----------------------------------------------------
Sensor sensor(Serial1);
String sensorData = "";  // Latest sensor data (JSON)

//-----------------------------------------------------
// Helper: Send a JSON response
//-----------------------------------------------------
void sendJsonResponse(WiFiClient &client, const String &json) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Connection: close"));
  client.println();
  client.println(json);
}

//-----------------------------------------------------
// Sensor Callback Functions
//-----------------------------------------------------
void sensorDataCallback(char* data, int length) {
  sensorData = String(data);
  DEBUG_PRINT(F("Sensor Data Callback: "));
  DEBUG_PRINTLN(sensorData);
}

void sensorInitCompleteCallback(void* context) {
  DEBUG_PRINTLN(F("Sensor initialization complete!"));
  SensorStatus status = sensor.getStatus();
  DEBUG_PRINT(F("Device Type: "));
  DEBUG_PRINTLN(status.deviceType);
  DEBUG_PRINT(F("Number of Modes: "));
  DEBUG_PRINTLN(status.numModes);
}

//-----------------------------------------------------
// WiFi Setup Function
//-----------------------------------------------------
void setupWiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    DEBUG_PRINTLN(F("WiFi module not found"));
    while (true);
  }
  
  DEBUG_PRINT(F("Connecting to "));
  DEBUG_PRINTLN(ssid);
  int status = WiFi.begin(ssid, pass);
  while (status != WL_CONNECTED) {
    delay(1000);
    DEBUG_PRINT(F("."));
    status = WiFi.status();
  }
  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("WiFi connected"));
  DEBUG_PRINT(F("IP Address: "));
  DEBUG_PRINTLN(WiFi.localIP());
}

//-----------------------------------------------------
// Helper: Serve an HTML page from PROGMEM
//-----------------------------------------------------
void serveHtmlPage(WiFiClient &client, const char* htmlPage) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();
  client.print((__FlashStringHelper*)htmlPage);
}

//-----------------------------------------------------
// API Endpoint Handler: Sensor Modes
//-----------------------------------------------------
void serveSensorModes(WiFiClient &client) {
  SensorStatus status = sensor.getStatus();
  String json = "{";
  json += "\"deviceType\":" + String(status.deviceType) + ",";
  json += "\"extMode\":" + String(status.extMode) + ",";
  json += "\"numModes\":" + String(status.numModes) + ",";
  json += "\"modes\":[";
  for (int i = 0; i < status.numModes; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"name\":\"" + String(status.modeInfos[i].name) + "\"";
    if (status.modeInfos[i].rawReceived) {
      json += ",\"rawMin\":" + String(status.modeInfos[i].rawMin, 2) + ",\"rawMax\":" + String(status.modeInfos[i].rawMax, 2);
    }
    if (status.modeInfos[i].pctReceived) {
      json += ",\"pctMin\":" + String(status.modeInfos[i].pctMin, 2) + ",\"pctMax\":" + String(status.modeInfos[i].pctMax, 2);
    }
    if (status.modeInfos[i].siReceived) {
      json += ",\"siMin\":" + String(status.modeInfos[i].siMin, 2) + ",\"siMax\":" + String(status.modeInfos[i].siMax, 2);
    }
    if (status.modeInfos[i].unitsReceived) {
      json += ",\"units\":\"" + String(status.modeInfos[i].units) + "\"";
    }
    if (status.modeInfos[i].formatReceived) {
      json += ",\"dataSets\":" + String(status.modeInfos[i].dataSets);
      json += ",\"dataFormat\":" + String(status.modeInfos[i].dataFormat);
      json += ",\"figures\":" + String(status.modeInfos[i].figures);
      json += ",\"decimals\":" + String(status.modeInfos[i].decimals);
    }
    json += "}";
  }
  json += "]}";
  
  sendJsonResponse(client, json);
}

//-----------------------------------------------------
// API Endpoint Handler: Set Sensor Mode
//-----------------------------------------------------
void serveSensorModeSet(WiFiClient &client, String &request) {
  int modeIndex = -1;
  int pos = request.indexOf("mode=");
  if (pos != -1) {
    int start = pos + 5;
    int end = request.indexOf(" ", start);
    if (end == -1) end = request.length();
    String modeStr = request.substring(start, end);
    modeIndex = modeStr.toInt();
  }
  
  SensorStatus status = sensor.getStatus();
  if (modeIndex >= 0 && modeIndex < status.numModes) {
    sensor.setMode(modeIndex);
    String json = "{\"result\":\"Mode set to " + String(modeIndex) + "\"}";
    sendJsonResponse(client, json);
  } else {
    client.println(F("HTTP/1.1 400 Bad Request"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("{\"error\":\"Invalid mode\"}"));
  }
}

//-----------------------------------------------------
// API Endpoint Handler: Sensor Data
//-----------------------------------------------------
void serveSensorData(WiFiClient &client) {
  String json = (sensorData.length() > 0) ? sensorData : "{}";
  sendJsonResponse(client, json);
}

//-----------------------------------------------------
// Main Web Request Handler
//-----------------------------------------------------
void handleClientRequest(WiFiClient &client) {
  String request = "";
  while (client.connected() && client.available()) {
    String line = client.readStringUntil('\n');
    request += line + "\n";
    if (line.length() == 1) break;  // End of headers
  }
  DEBUG_PRINTLN(F("HTTP Request:"));
  DEBUG_PRINTLN(request);
  
  if (request.indexOf("GET /api/sensor/modes") >= 0) {
    serveSensorModes(client);
  }
  else if (request.indexOf("GET /api/sensor/mode") >= 0) {
    // Added branch to set sensor mode
    serveSensorModeSet(client, request);
  }
  else if (request.indexOf("GET /api/sensor/data") >= 0) {
    serveSensorData(client);
  }
  else {
    serveHtmlPage(client, HTML_PAGE);
  }
  
  delay(1);
  client.stop();
  DEBUG_PRINTLN(F("Client disconnected"));
}

//-----------------------------------------------------
// Setup Function
//-----------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  DEBUG_PRINTLN(F("Starting WiFi setup..."));
  setupWiFi();
  server.begin();
  DEBUG_PRINTLN(F("HTTP server started"));
  
  // Initialize sensor on Serial1
  sensor.begin();
  sensor.subscribeDataMessages(sensorDataCallback);
  sensor.subscribeInitializationComplete(sensorInitCompleteCallback, nullptr);
  
  DEBUG_PRINTLN(F("Sensor starting..."));
  if (sensor.waitForInitialization(10000)) {
    DEBUG_PRINTLN(F("Sensor initialization complete."));
  } else {
    DEBUG_PRINTLN(F("Sensor initialization timed out."));
  }
}

//-----------------------------------------------------
// Main Loop
//-----------------------------------------------------
void loop() {
  sensor.process();
  
  SensorError err = sensor.getLastError();
  if (err != SensorError::None) {
    DEBUG_PRINT(F("Sensor Error: "));
    switch (err) {
      case SensorError::BufferOverflow: DEBUG_PRINTLN(F("Buffer Overflow")); break;
      case SensorError::ChecksumError:  DEBUG_PRINTLN(F("Checksum Error")); break;
      case SensorError::SyncLoss:       DEBUG_PRINTLN(F("Sync Loss")); break;
      case SensorError::Timeout:        DEBUG_PRINTLN(F("Timeout")); break;
      default:                          DEBUG_PRINTLN(F("Unknown Error")); break;
    }
    sensor.clearLastError();
  }
  
  WiFiClient client = server.available();
  if (client) {
    DEBUG_PRINTLN(F("\nNew client connected"));
    handleClientRequest(client);
  }
}
