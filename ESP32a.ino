#include <Wire.h>
#include <MS5837.h>
#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <ArduinoOTA.h>
#include <Update.h>

#define CUSTOM_SDA_PIN 21 
#define CUSTOM_SCL_PIN 22 

MS5837 sensor;

const char* ssid     = "SSCFloat";
const char* password = "DT1234dt";

WebServer server(80);

float pressures[120];      
float temperatures[120];   
int sensorIdx = 0;
int iterationCount = 0;    

const int dirPin  = 4;  
const int stepPin = 16; 

// --- PINS RETAINED AS REQUESTED ---
#define BUTTON_PIN_1 19    
#define BUTTON_PIN_2 18    
#define ESP_BOOT_BUTTON 0    

// --- EXPANDED SETTINGS ---
int motorSpeed = 1500;       
int bottomDelayMs = 5000;   
int topDelayMs = 2000;
bool startClockwise = true;  
bool continuousLoop = false; 

// --- SYSTEM STATES & OVERRIDES ---
SemaphoreHandle_t dataLock = NULL;
SemaphoreHandle_t motorStatusLock = NULL;
QueueHandle_t motorCommandQueue = NULL;

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

volatile bool motorBusy = false;
volatile bool overrideTriggered = false; // The absolute interrupt flag
String currentMotorState = "IDLE";      

void pulseMotor() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(motorSpeed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(motorSpeed);
}

// Smart delay that instantly breaks if the override button is pressed
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while(millis() - start < ms) {
    if(overrideTriggered) return;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// --- FULLY INTERRUPTIBLE SEQUENCE ---
void runStepperSequence() {
    do {
        if(overrideTriggered) return;
        
        // --- LEG 1 ---
        currentMotorState = "Moving to Sensor 1";
        int firstDir = startClockwise ? LOW : HIGH;
        int firstButton = startClockwise ? BUTTON_PIN_1 : BUTTON_PIN_2;
        digitalWrite(dirPin, firstDir);  
        
        while(digitalRead(firstButton) == HIGH) {  
            if(overrideTriggered) return; 
            pulseMotor();
            taskYIELD();
        }
        
        if(overrideTriggered) return;
        
        // --- BOTTOM WAIT ---
        currentMotorState = "Waiting (Bottom Delay)";
        smartDelay(bottomDelayMs);
        
        if(overrideTriggered) return;
        
        // --- LEG 2 (Return Trip) ---
        currentMotorState = "Moving to Sensor 2";
        int secondDir = startClockwise ? HIGH : LOW;
        int secondButton = startClockwise ? BUTTON_PIN_2 : BUTTON_PIN_1;
        digitalWrite(dirPin, secondDir); 
        
        while(digitalRead(secondButton) == HIGH) {  
            if(overrideTriggered) return; 
            pulseMotor();
            taskYIELD();
        }

        if(overrideTriggered) return;

        // --- TOP WAIT ---
        currentMotorState = "Waiting (Top Delay)";
        smartDelay(topDelayMs);

    } while(continuousLoop && !overrideTriggered);

    if(!overrideTriggered) {
        currentMotorState = "IDLE";
    }
}

void motorTask(void *parameter) {
  bool command;
  while(1) {
    if(xQueueReceive(motorCommandQueue, &command, portMAX_DELAY)) {
      if (xSemaphoreTake(motorStatusLock, portMAX_DELAY)) {
        motorBusy = true;
        xSemaphoreGive(motorStatusLock);
      }
      
      runStepperSequence();
      
      if (xSemaphoreTake(motorStatusLock, portMAX_DELAY)) {
        motorBusy = false;
        if(!overrideTriggered) currentMotorState = "IDLE";
        xSemaphoreGive(motorStatusLock);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void sensorTask(void *parameter) {
  while(1) {
    sensor.read();
    if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
      pressures[sensorIdx] = sensor.pressure();
      temperatures[sensorIdx] = sensor.temperature();
      sensorIdx = (sensorIdx + 1) % 120;
      iterationCount++;
      xSemaphoreGive(dataLock);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void webTask(void *parameter) {
  while(1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void handleData() {
  if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
    int count = iterationCount < 120 ? iterationCount : 120;
    String data = "";
    for (int i = 0; i < count; i++) {
      int idx = (sensorIdx + i) % 120;
      data += String(pressures[idx]) + "," + String(temperatures[idx]) + "\n";
    }
    xSemaphoreGive(dataLock);
    server.send(200, "text/plain", data);
  }
}

void handleStatus() {
  bool isBusy = false;
  if (xSemaphoreTake(motorStatusLock, portMAX_DELAY)) {
    isBusy = motorBusy;
    xSemaphoreGive(motorStatusLock);
  }

  float lastPressure = 0;
  float lastTemp = 0;
  if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
    if (iterationCount > 0) {
      int lastIdx = (sensorIdx == 0) ? 119 : sensorIdx - 1;
      lastPressure = pressures[lastIdx];
      lastTemp = temperatures[lastIdx];
    }
    xSemaphoreGive(dataLock);
  }

  String json = "{";
  json += "\"motorBusy\":" + String(isBusy ? "true" : "false") + ",";
  json += "\"state\":\"" + currentMotorState + "\",";
  json += "\"btn1\":" + String(digitalRead(BUTTON_PIN_1)) + ",";
  json += "\"btn2\":" + String(digitalRead(BUTTON_PIN_2)) + ",";
  json += "\"pressure\":" + String(lastPressure) + ",";
  json += "\"temp\":" + String(lastTemp) + ",";
  json += "\"startDir\":\"" + String(startClockwise ? "C" : "AC") + "\",";
  json += "\"uptime\":" + String(millis() / 1000);
  json += "}";

  server.send(200, "application/json", json);
}

void handleControl() {
  if(server.hasArg("action")) {
    if(server.hasArg("speed")) motorSpeed = server.arg("speed").toInt();
    if(server.hasArg("bottomWait")) bottomDelayMs = server.arg("bottomWait").toInt();
    if(server.hasArg("topWait")) topDelayMs = server.arg("topWait").toInt();
    if(server.hasArg("direction")) startClockwise = (server.arg("direction") == "c");
    continuousLoop = server.hasArg("continuous"); 

    if(server.arg("action") == "start") {
      bool canStart = false;
      if (xSemaphoreTake(motorStatusLock, portMAX_DELAY)) {
        canStart = !motorBusy;
        xSemaphoreGive(motorStatusLock);
      }
      if (canStart) {
        bool command = true;
        xQueueSend(motorCommandQueue, &command, portMAX_DELAY);
      }
    }
  }

  // --- NEW SLEEK DASHBOARD UI ---
  String html = R"rawliteral(
  <!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>
    :root { --bg: #1e1e2e; --surface: #2a2a35; --primary: #89b4fa; --text: #cdd6f4; --success: #a6e3a1; --danger: #f38ba8; }
    body { background: var(--bg); color: var(--text); font-family: system-ui, sans-serif; margin: 20px; }
    .header { text-align: center; border-bottom: 2px solid var(--surface); padding-bottom: 10px; margin-bottom: 20px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 20px; }
    .card { background: var(--surface); padding: 20px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.3); }
    h3 { margin-top: 0; color: var(--primary); font-size: 1.2em; border-bottom: 1px solid #45475a; padding-bottom: 8px;}
    label { display: block; margin: 10px 0 5px; font-size: 0.9em; color: #a6adc8; }
    input[type=number], select { width: 100%; padding: 10px; background: #181825; color: var(--text); border: 1px solid #45475a; border-radius: 6px; box-sizing: border-box; margin-bottom: 10px;}
    .checkbox-row { display: flex; align-items: center; gap: 10px; margin-bottom: 15px; font-size: 0.9em; color: #a6adc8;}
    .btn { display: block; width: 100%; padding: 12px; border: none; border-radius: 6px; font-weight: bold; cursor: pointer; transition: 0.2s; margin-bottom: 10px; font-size: 1em; }
    .btn-save { background: #45475a; color: var(--text); }
    .btn-save:hover { background: #585b70; }
    .btn-start { background: var(--success); color: #111; }
    .btn-start:hover { opacity: 0.8; }
    .btn:disabled { background: #313244; color: #585b70; cursor: not-allowed; }
    .debug-row { display: flex; justify-content: space-between; padding: 10px 0; border-bottom: 1px solid #313244; font-family: monospace; font-size: 0.95em;}
    .debug-row:last-child { border-bottom: none; }
    .val-high { color: var(--success); font-weight: bold; }
    .val-low { color: var(--danger); font-weight: bold; }
    pre { background: #11111b; color: var(--success); padding: 10px; border-radius: 6px; height: 250px; overflow-y: auto; font-size: 13px; border: 1px solid #313244;}
    .live-state { font-size: 1.1em; font-weight: bold; color: #f9e2af; text-align: center; margin-bottom: 15px; padding: 10px; background: #181825; border-radius: 6px;}
  </style>
  <script>
    function updateDashboard() {
      fetch('/data').then(r => r.text()).then(data => {
        const preview = document.getElementById('dataPreview');
        const isScrolled = preview.scrollHeight - preview.clientHeight <= preview.scrollTop + 1;
        preview.innerText = data;
        if(isScrolled) preview.scrollTop = preview.scrollHeight;
      });
      fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('stateText').innerText = data.state;
        
        const b1 = document.getElementById('dbgBtn1');
        b1.innerText = data.btn1 === 0 ? 'TRIGGERED (LOW)' : 'DEFAULT (HIGH)';
        b1.className = data.btn1 === 0 ? 'val-low' : 'val-high';
        
        const b2 = document.getElementById('dbgBtn2');
        b2.innerText = data.btn2 === 0 ? 'TRIGGERED (LOW)' : 'DEFAULT (HIGH)';
        b2.className = data.btn2 === 0 ? 'val-low' : 'val-high';
        
        document.getElementById('dbgPress').innerText = data.pressure.toFixed(2) + ' mbar';
        document.getElementById('dbgTemp').innerText = data.temp.toFixed(2) + ' °C';
        document.getElementById('dbgDir').innerText = data.startDir === 'C' ? 'Clockwise' : 'Anti-Clockwise';
        
        let hrs = Math.floor(data.uptime / 3600);
        let mins = Math.floor((data.uptime % 3600) / 60);
        let secs = data.uptime % 60;
        document.getElementById('dbgTime').innerText = `${hrs}h ${mins}m ${secs}s`;
      });
    }
    setInterval(updateDashboard, 500);
    window.onload = updateDashboard;
  </script>
  </head><body>
  <div class='header'>
    <h2>SSC System Dashboard</h2>
  </div>
  <div class='grid'>
  )rawliteral";

  // Panel 1: Settings
  html += "<div class='card'><h3>Motor Configuration</h3>";
  html += "<form action='/control' method='GET'>";
  html += "<label>Motor Speed Delay (&micro;s):</label><input type='number' name='speed' value='" + String(motorSpeed) + "'>";
  html += "<label>Bottom Wait Time (ms):</label><input type='number' name='bottomWait' value='" + String(bottomDelayMs) + "'>";
  html += "<label>Top Wait Time (ms):</label><input type='number' name='topWait' value='" + String(topDelayMs) + "'>";
  html += "<label>Start Direction:</label><select name='direction'>";
  html += "<option value='c'" + String(startClockwise ? " selected" : "") + ">Clockwise (C)</option>";
  html += "<option value='ac'" + String(!startClockwise ? " selected" : "") + ">Anti-Clockwise (AC)</option></select>";
  html += "<div class='checkbox-row'><input type='checkbox' name='continuous' value='1'" + String(continuousLoop ? " checked" : "") + "> Run continuous infinite loop</div>";
  
  bool isBusy = false;
  if (xSemaphoreTake(motorStatusLock, portMAX_DELAY)) {
    isBusy = motorBusy;
    xSemaphoreGive(motorStatusLock);
  }
  
  if (!isBusy) {
    html += "<button type='submit' name='action' value='update' class='btn btn-save'>Save Parameters</button>";
    html += "<button type='submit' name='action' value='start' class='btn btn-start'>START SEQUENCE</button>";
  } else {
    html += "<button class='btn btn-start' disabled>SYSTEM RUNNING...</button>";
  }
  html += "</form></div>";

  // Panel 2: Telemetry & Debug
  html += "<div class='card'><h3>Live Telemetry</h3>";
  html += "<div class='live-state' id='stateText'>Loading State...</div>";
  html += "<div class='debug-row'><span>Start Direction:</span><span id='dbgDir' class='val-high'>-</span></div>";
  html += "<div class='debug-row'><span>Switch 1 (Pin 19):</span><span id='dbgBtn1'>-</span></div>";
  html += "<div class='debug-row'><span>Switch 2 (Pin 18):</span><span id='dbgBtn2'>-</span></div>";
  html += "<div class='debug-row'><span>Fluid Pressure:</span><span id='dbgPress' class='val-high'>-</span></div>";
  html += "<div class='debug-row'><span>Fluid Temp:</span><span id='dbgTemp' class='val-high'>-</span></div>";
  html += "<div class='debug-row'><span>System Uptime:</span><span id='dbgTime' class='val-high'>-</span></div>";
  html += "</div>";

  // Panel 3: Data Stream
  html += "<div class='card'><h3>Sensor Data Buffer</h3>";
  html += "<pre id='dataPreview'>Loading stream...</pre>";
  html += "<a href='/data' target='_blank' style='color: var(--primary); text-decoration: none; font-size: 0.9em;'>[ View Raw CSV Endpoint ]</a>";
  html += "</div></div></body></html>";
  
  server.send(200, "text/html", html);
}

void setup() {
    Serial.begin(115200);  
    delay(1000);
    
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);

    if (!sensor.init()) {
        Serial.println("Sensor failed to initialize!");
        // We let it continue so the web server still works for debugging
    } else {
        sensor.setFluidDensity(997);
    }
    
    pinMode(dirPin, OUTPUT);  
    pinMode(stepPin, OUTPUT); 
    
    pinMode(BUTTON_PIN_1, INPUT_PULLUP); 
    pinMode(BUTTON_PIN_2, INPUT_PULLUP); 
    pinMode(ESP_BOOT_BUTTON, INPUT_PULLUP); 
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("Access Point started. IP: ");
    Serial.println(WiFi.softAPIP());
    
    ArduinoOTA.onStart([]() { Serial.println("Start OTA"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd OTA"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("Error[%u]: ", error); });
    ArduinoOTA.begin();
    
    server.on("/data", handleData);
    server.on("/status", handleStatus); 
    server.on("/control", handleControl);
    server.begin();

    dataLock = xSemaphoreCreateMutex();
    motorCommandQueue = xQueueCreate(1, sizeof(bool));
    motorStatusLock = xSemaphoreCreateMutex();
    
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, &sensorTaskHandle, 0);
    xTaskCreatePinnedToCore(webTask, "WebTask", 4096, NULL, 1, &webTaskHandle, 0);
    xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, NULL, 3, &motorTaskHandle, 1);
}

unsigned long lastButtonPress = 0;

void loop() {
  ArduinoOTA.handle();      
  
  // --- THE ABSOLUTE OVERRIDE LOGIC ---
  if (digitalRead(ESP_BOOT_BUTTON) == LOW && (millis() - lastButtonPress > 1000)) {
    lastButtonPress = millis();
    Serial.println("OVERRIDE BUTTON PRESSED!");

    // 1. Tell the running motor task to instantly abort its current loop
    overrideTriggered = true;
    currentMotorState = "OVERRIDE TRIGGERED!";
    
    // 2. Wait just a brief moment for the motor sequence to safely exit
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 3. Flip the direction variable
    startClockwise = !startClockwise; 
    
    // 4. Reset the abort flag
    overrideTriggered = false;
    
    // 5. Clear any old commands in the queue, and instantly push the new start command
    xQueueReset(motorCommandQueue);
    bool command = true;
    xQueueSend(motorCommandQueue, &command, portMAX_DELAY);
  }

  vTaskDelay(pdMS_TO_TICKS(50)); 
}