#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_PCF8574.h> 
#include <WebSocketsServer.h>

// --- Network Credentials ---
const char* ssid = "Tatakae!!!";
const char* password = "nisarga2812";

// WebSockets Server on Port 81
WebSocketsServer webSocket = WebSocketsServer(81);

// ESP32 Pin Settings
const int BUTTON_SCAN_PIN = 4;  // Button 1: Mode/Reset
const int BUTTON_NAV_PIN  = 5;  // Button 2: History Navigation

// DEDICATED HARDWARE I2C PINS
// Bus 0 (Wire): Sensor and Live Screen share this line
const int I2C_SDA_LIVE = 21;    
const int I2C_SCL_LIVE = 22;    

// Bus 1 (Wire1): History Screen lives alone here
const int I2C_SDA_HIST = 25;    
const int I2C_SCL_HIST = 26;    

// --- DUAL LCD CONFIGURATION ---
LiquidCrystal_PCF8574 lcdLive(0x27);       
LiquidCrystal_PCF8574 lcdHistory(0x27);   

// Sensor Engine (Initialized on Bus 0 / Wire)
Adafruit_MPU6050 mpu;

// Operational States
enum DeviceState { IDLE, MEASURING, STOPPED };
DeviceState currentState = IDLE;

// Timing Variables (Button 1)
unsigned long buttonPressTime = 0;
bool isButtonPressed = false;
const unsigned long LONG_PRESS_TIME = 2000; 
int lastButtonState = HIGH;

// Timing Variables (Button 2)
int lastNavButtonState = HIGH;

// Tremor Tracking Engine Parameters
float currentTremorScore = 0.0;
float baselineScore = -1.0; 
float accelerationMagnitude = 0.0;
float lastMagnitude = 0.0;
float tremorSum = 0.0;
int sampleCount = 0;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 50; // Local logic interval (20Hz sampling)

unsigned long lastSendTime = 0;
const int sendInterval = 100; // WebSocket transmission interval (10Hz)

// Array Storage & Navigation
const int MAX_HISTORY = 5;       
float tremorHistory[MAX_HISTORY]; 
int savedReadingsCount = 0;      
int historyViewIndex = 0;

// Forward Declarations
void clearHistoryArray();
void sampleSensorAndStream();
void handleScanButton();
void handleNavButton();
void updateDisplays();
void showWelcomeScreen();
void displayFeedback(const char* message);
void addReadingToArray(float newScore);
const char* getSeverityStatus(float score);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
  delay(500); 
  Serial.begin(115200);
  
  pinMode(BUTTON_SCAN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_NAV_PIN, INPUT_PULLUP);

  clearHistoryArray();

  // 1. Initialize Hardware I2C Buses
  Wire.begin(I2C_SDA_LIVE, I2C_SCL_LIVE, 100000);
  Wire1.begin(I2C_SDA_HIST, I2C_SCL_HIST, 100000);

  // 2. Initialize LCD Screens
  lcdLive.begin(16, 2, Wire);
  lcdLive.setBacklight(255);

  lcdHistory.begin(16, 2, Wire1); 
  lcdHistory.setBacklight(255);
  
  // 3. Connect to the Accelerometer
  if (!mpu.begin(0x68, &Wire)) {
    lcdLive.clear();
    lcdLive.setCursor(0, 0);
    lcdLive.print("MPU6050 Error!");
    
    lcdHistory.clear();
    lcdHistory.setCursor(0, 0);
    lcdHistory.print("Check Sensor!");
    while(1) { delay(100); }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 

  showWelcomeScreen();
  
  lcdLive.clear();
  lcdHistory.clear();

  // 4. Initialize Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // 5. Start WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  // Keep web socket server handling client requests
  webSocket.loop();

  handleScanButton();
  handleNavButton();

  // Combined tracking mechanism using non-blocking structural timers
  unsigned long currentTime = millis();
  
  if (currentState == MEASURING) {
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
      lastSampleTime = currentTime;
      sampleSensorAndStream();
    }
  } else {
    // If we aren't scanning, we still want to keep streaming baseline data to the web app
    if (currentTime - lastSendTime >= sendInterval) {
      lastSendTime = currentTime;
      
      // Send flat inactive telemetry string when not monitoring local calculations
      String jsonString = "{\"freq\":0.00,\"amp\":0.00}";
      webSocket.broadcastTXT(jsonString);
    }
  }
  
  static unsigned long lastDisplayTime = 0;
  if (millis() - lastDisplayTime >= 250) {
    lastDisplayTime = millis();
    updateDisplays();
  }
}

void sampleSensorAndStream() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate raw amplitude magnitude
  accelerationMagnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
  
  // --- Local LCD Processing Logic ---
  float delta = abs(accelerationMagnitude - lastMagnitude);
  lastMagnitude = accelerationMagnitude;
  
  if (delta < 0.18) delta = 0;
  tremorSum += delta;
  sampleCount++;
  
  if (sampleCount >= 20) { 
    float averageFluctuation = tremorSum / sampleCount;
    currentTremorScore = (averageFluctuation - 0.2) * (10.0 / (3.8 - 0.2));
    currentTremorScore = constrain(currentTremorScore, 0.0, 10.0);

    tremorSum = 0;
    sampleCount = 0;
  }

  // --- Remote WebSocket Streaming Logic ---
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;

    float amplitude = accelerationMagnitude - 9.81; // Net dynamic structural forces minus gravity
    if (amplitude < 0) amplitude = 0;

    float frequency = 0.0;
    if (amplitude > 0.5) {
      frequency = 5.5; // Emulated Parkinson's baseline profile frequency
    }

    String jsonString = "{\"freq\":" + String(frequency, 2) + ",\"amp\":" + String(amplitude, 2) + "}";
    webSocket.broadcastTXT(jsonString);
  }
}

void handleScanButton() {
  int reading = digitalRead(BUTTON_SCAN_PIN);
  
  if (reading == LOW && lastButtonState == HIGH && !isButtonPressed) {
    buttonPressTime = millis();
    isButtonPressed = true;
  }

  if (reading == LOW && isButtonPressed) {
    if (millis() - buttonPressTime >= LONG_PRESS_TIME) {
      baselineScore = -1.0;
      currentTremorScore = 0.0;
      currentState = IDLE;
      isButtonPressed = false; 
      clearHistoryArray(); 
      displayFeedback("SYSTEM RESET");
      lcdLive.clear();
      lcdHistory.clear();
    }
  }

  if (reading == HIGH && lastButtonState == LOW && isButtonPressed) {
    unsigned long pressDuration = millis() - buttonPressTime;
    
    if (pressDuration < LONG_PRESS_TIME) {
      if (currentState == IDLE || currentState == STOPPED) {
        currentState = MEASURING;
        tremorSum = 0;
        sampleCount = 0;
      } else if (currentState == MEASURING) {
        currentState = STOPPED;
        if (baselineScore < 0) {
          baselineScore = currentTremorScore;
        }
        addReadingToArray(currentTremorScore);
      }
    }
    isButtonPressed = false;
  }
  lastButtonState = reading;
}

void handleNavButton() {
  int reading = digitalRead(BUTTON_NAV_PIN);

  if (reading == LOW && lastNavButtonState == HIGH) {
    delay(50);
    if (savedReadingsCount > 0) {
      historyViewIndex = (historyViewIndex + 1) % savedReadingsCount;
    }
  }
  lastNavButtonState = reading;
}

void addReadingToArray(float newScore) {
  for (int i = MAX_HISTORY - 1; i > 0; i--) {
    tremorHistory[i] = tremorHistory[i - 1];
  }
  tremorHistory[0] = newScore;
  
  if (savedReadingsCount < MAX_HISTORY) {
    savedReadingsCount++;
  }
  historyViewIndex = 0;
}

void clearHistoryArray() {
  for (int i = 0; i < MAX_HISTORY; i++) {
    tremorHistory[i] = 0.0;
  }
  savedReadingsCount = 0;
  historyViewIndex = 0;
}

const char* getSeverityStatus(float score) {
  if (score <= 2.0) return " STB ";
  if (score <= 6.0) return " MOD ";
  return " SVR ";
}

void updateDisplays() {
  // --- DISPLAY 1: LIVE VITALS MONITOR ---
  lcdLive.setCursor(0, 0);
  if (currentState == IDLE)           lcdLive.print("MODE: READY    ");
  else if (currentState == MEASURING) lcdLive.print("MODE: SCANNING ");
  else                                lcdLive.print("MODE: PAUSED   ");
  
  lcdLive.setCursor(0, 1);
  lcdLive.print("SCORE: ");
  lcdLive.print(currentTremorScore, 1);
  lcdLive.print(" ");
  lcdLive.print(getSeverityStatus(currentTremorScore));

  // --- DISPLAY 2: MULTI-ARRAY HISTORY MONITOR ---
  lcdHistory.setCursor(0, 0);
  lcdHistory.print("1:"); lcdHistory.print(tremorHistory[0], 1);
  lcdHistory.print(" 2:"); lcdHistory.print(tremorHistory[1], 1);
  lcdHistory.print("3:"); lcdHistory.print(tremorHistory[2], 1);
  
  lcdHistory.setCursor(0, 1);
  lcdHistory.print(" 4:"); lcdHistory.print(tremorHistory[3], 1);
  lcdHistory.print(" 5:"); lcdHistory.print(tremorHistory[4], 1);
  
  lcdHistory.print("[");
  if (savedReadingsCount == 0) {
    lcdHistory.print("-");
  } else {
    lcdHistory.print(historyViewIndex + 1);
  }
  lcdHistory.print("]");
}

void showWelcomeScreen() {
  lcdLive.clear();
  lcdLive.setCursor(2, 0);
  lcdLive.print("TremorTrack");
  lcdLive.setCursor(0, 1);
  lcdLive.print("Vitals Monitor");

  lcdHistory.clear();
  lcdHistory.setCursor(2, 0);
  lcdHistory.print("Data Analytics");
  lcdHistory.setCursor(1, 1);
  lcdHistory.print("5-Slot DATA");
  
  delay(2500);
}

void displayFeedback(const char* message) {
  lcdLive.clear();
  lcdLive.setCursor(2, 0);
  lcdLive.print(message);
  
  lcdHistory.clear();
  lcdHistory.setCursor(0, 0);
  lcdHistory.print("Clearing Slots..");
  delay(1200);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("[%u] Connected from webpage!\n", num);
      break;
    case WStype_TEXT:
      break;
  }
}