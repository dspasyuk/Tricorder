#include "config.h"
#include <Wire.h>
#include "DFRobot_AS7341.h"
#include <ArduinoJson.h>
#include "echarts_min_js.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Configuration
#define OLED_SDA 5
#define OLED_SCL 6
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define DISPLAY_OFFSET_X 28
#define DISPLAY_OFFSET_Y 15
#define USABLE_WIDTH 72
#define USABLE_HEIGHT 40

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Scan timing
unsigned long scanStartTime = 0;
unsigned long scanElapsedTime = 0;
unsigned long lastDisplayUpdate = 0;

// Global object definitions
WebServer server(80);
Preferences preferences;
DFRobot_AS7341 as7341;

// WiFi configuration
String ssid = "";
String password = "";
bool apMode = false;

// Data collection variables
bool collecting = false;
unsigned int accumulationTime = 1; // seconds
unsigned int scanInterval = 2; // seconds
const int interpolationPoints = 100;

// AS7341 channel center wavelengths (nm)
const float channelWavelengths[10] = {
  415, 445, 480, 515, 555, 590, 630, 680, 910, 550
};
const char* channelNames[10] = {
  "F1 (415nm)", "F2 (445nm)", "F3 (480nm)", "F4 (515nm)", 
  "F5 (555nm)", "F6 (590nm)", "F7 (630nm)", "F8 (680nm)", 
  "NIR (910nm)", "Clear"
};

// Store raw and normalized readings
float normalizedReadings[10] = {0};
float interpolatedSpectrum[interpolationPoints][2];

DFRobot_AS7341::sModeOneData_t data1;
DFRobot_AS7341::sModeTwoData_t data2;

// Display helper function - prints text at offset coordinates
void displayPrint(int x, int y, const char* text) {
  display.setCursor(DISPLAY_OFFSET_X + x, DISPLAY_OFFSET_Y + y);
  display.print(text);
}

void displayPrintln(int x, int y, const char* text) {
  display.setCursor(DISPLAY_OFFSET_X + x, DISPLAY_OFFSET_Y + y);
  display.println(text);
}

// Update OLED display with current status
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: WiFi Status
  displayPrint(0, 0, apMode ? "AP Mode" : "WiFi OK");
  
  // Line 2: IP or status
  if (!apMode && WiFi.status() == WL_CONNECTED) {
    displayPrint(0, 10, WiFi.localIP().toString().c_str());
  } else if (apMode) {
    displayPrint(0, 10, "192.168.4.1");
  }
  
  // Line 3: Collection status
  if (collecting) {
    displayPrint(0, 20, "SCANNING");
  } else {
    displayPrint(0, 20, "Ready");
  }
  
  // Line 4: Show peak channel
  if (normalizedReadings[0] > 0) {
    float maxVal = 0;
    int maxIdx = 0;
    for (int i = 0; i < 8; i++) { // Only visible channels
      if (normalizedReadings[i] > maxVal) {
        maxVal = normalizedReadings[i];
        maxIdx = i;
      }
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%.0fnm", channelWavelengths[maxIdx]);
    displayPrint(0, 30, buf);
  }
  
  display.display();
}

// Cubic interpolation for smoother curves (Catmull-Rom spline)
float cubicInterpolate(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  float a0 = -0.5*p0 + 1.5*p1 - 1.5*p2 + 0.5*p3;
  float a1 = p0 - 2.5*p1 + 2*p2 - 0.5*p3;
  float a2 = -0.5*p0 + 0.5*p2;
  float a3 = p1;
  return a0*t3 + a1*t2 + a2*t + a3;
}

// Read and accumulate sensor data
void readAndAccumulate() {
  long accumulatedReadings[10] = {0};
  int readingCount = 0;
  unsigned long startTime = millis();

  Serial.println("Starting accumulation...");
  
  // Update display to show scanning
  collecting = true;
  updateDisplay();

  while (millis() - startTime < accumulationTime * 1000) {
    as7341.startMeasure(as7341.eF1F4ClearNIR);
    data1 = as7341.readSpectralDataOne();
    
    as7341.startMeasure(as7341.eF5F8ClearNIR);
    data2 = as7341.readSpectralDataTwo();

    accumulatedReadings[0] += data1.ADF1;
    accumulatedReadings[1] += data1.ADF2;
    accumulatedReadings[2] += data1.ADF3;
    accumulatedReadings[3] += data1.ADF4;
    accumulatedReadings[4] += data2.ADF5;
    accumulatedReadings[5] += data2.ADF6;
    accumulatedReadings[6] += data2.ADF7;
    accumulatedReadings[7] += data2.ADF8;
    accumulatedReadings[8] += data1.ADNIR;
    accumulatedReadings[9] += data1.ADCLEAR;
    
    readingCount++;
    delay(50);
  }

  Serial.print("Accumulated ");
  Serial.print(readingCount);
  Serial.println(" readings.");

  // Average the readings from all 10 channels
  float avgReadings[10];
  for (int i = 0; i < 10; i++) {
    avgReadings[i] = (readingCount > 0) ? (float)accumulatedReadings[i] / readingCount : 0;
    normalizedReadings[i] = avgReadings[i];
  }
  
  collecting = false;
  updateDisplay();
}

// Get interpolated value at a specific wavelength
float getInterpolatedValue(float wavelength) {
    const float visWavelengths[] = {415, 445, 480, 515, 555, 590, 630, 680};
    const int numVisChannels = 8;

    float extendedWavelengths[numVisChannels + 2];
    float extendedReadings[numVisChannels + 2];

    extendedWavelengths[0] = 380;
    extendedReadings[0] = 0.0;

    for (int i = 0; i < numVisChannels; i++) {
        extendedWavelengths[i + 1] = visWavelengths[i];
        extendedReadings[i + 1] = normalizedReadings[i];
    }

    extendedWavelengths[numVisChannels + 1] = 750;
    extendedReadings[numVisChannels + 1] = 0.0;

    const int numExtendedPoints = numVisChannels + 2;

    if (wavelength <= extendedWavelengths[0]) return extendedReadings[0];
    if (wavelength >= extendedWavelengths[numExtendedPoints - 1]) return extendedReadings[numExtendedPoints - 1];

    int lowerIdx = 0;
    for (int i = 0; i < numExtendedPoints - 1; i++) {
        if (wavelength >= extendedWavelengths[i] && wavelength <= extendedWavelengths[i + 1]) {
            lowerIdx = i;
            break;
        }
    }

    float t = (wavelength - extendedWavelengths[lowerIdx]) / (extendedWavelengths[lowerIdx + 1] - extendedWavelengths[lowerIdx]);

    float p0 = (lowerIdx > 0) ? extendedReadings[lowerIdx - 1] : extendedReadings[lowerIdx];
    float p1 = extendedReadings[lowerIdx];
    float p2 = extendedReadings[lowerIdx + 1];
    float p3 = (lowerIdx < numExtendedPoints - 2) ? extendedReadings[lowerIdx + 2] : extendedReadings[lowerIdx + 1];

    return cubicInterpolate(p0, p1, p2, p3, t);
}

// Generate the interpolated spectrum
void generateInterpolatedSpectrum() {
  float minWavelength = 380;
  float maxWavelength = 750;
  float step = (maxWavelength - minWavelength) / (interpolationPoints - 1);

  for (int i = 0; i < interpolationPoints; i++) {
    float wavelength = minWavelength + i * step;
    interpolatedSpectrum[i][0] = wavelength;
    float intensity = getInterpolatedValue(wavelength);
    interpolatedSpectrum[i][1] = max(0.0f, intensity);
  }
}

void handleStatus() {
  StaticJsonDocument<200> doc;
  doc["startTime"] = scanStartTime;
  doc["elapsedTime"] = scanElapsedTime;
  doc["heap"] = ESP.getFreeHeap();
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleLight() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      as7341.enableLed(true);
      server.send(200, "text/plain", "Light ON");
    } else {
      as7341.enableLed(false);
      server.send(200, "text/plain", "Light OFF");
    }
  } else {
    server.send(400, "text/plain", "Missing state parameter");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- OLED Display Initialization (First!) ---
  Serial.println("Initializing OLED Display...");
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed!");
  } else {
    Serial.println("OLED initialized!");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    displayPrintln(0, 0, "Spectro");
    displayPrintln(0, 10, "meter");
    displayPrintln(0, 20, "Starting");
    display.display();
  }

  // --- I2C and Sensor Initialization ---
  Serial.println("Initializing Sensor...");
  // Reinitialize Wire for sensor on different pins
  Wire.end();
  Wire.begin(SDA_PIN, SCL_PIN);
  
  while (as7341.begin() != 0) {
    Serial.println("Could not find AS7341 sensor! Check wiring.");
    
    // Show error on display
    Wire.end();
    Wire.begin(OLED_SDA, OLED_SCL);
    display.clearDisplay();
    displayPrintln(0, 0, "Sensor");
    displayPrintln(0, 10, "Error!");
    display.display();
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    
    delay(3000);
  }
  
  Serial.println("AS7341 sensor found!");
  as7341.setAtime(100);
  as7341.setAGAIN(128);
  as7341.setAstep(999);
  as7341.enableSpectralMeasure(true);

  // --- WiFi Initialization ---
  Serial.println("Initializing WiFi...");
  preferences.begin("wifi-config", false);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  preferences.end();

  // Update display during WiFi connection
  Wire.end();
  Wire.begin(OLED_SDA, OLED_SCL);
  display.clearDisplay();
  displayPrintln(0, 0, "Connect");
  displayPrintln(0, 10, "WiFi...");
  display.display();
  Wire.end();
  Wire.begin(SDA_PIN, SCL_PIN);

  if (ssid.length() > 0) {
    Serial.println("Attempting to connect to saved WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Starting AP mode.");
    startAPMode();
  } else {
    Serial.println("Connected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    apMode = false;
  }

  // --- Server Initialization ---
  Serial.println("Initializing HTTP server...");
  server.on("/", handleRoot);
  
  server.on("/echarts.min.js", HTTP_GET, [](){
    server.sendHeader("Content-Encoding", "gzip");
    server.sendHeader("Cache-Control", "max-age=86400");
    server.send_P(200, "application/javascript", 
                  (const char*)ECHARTS_JS_GZ, 
                  ECHARTS_JS_GZ_LEN);
  });
  
  server.on("/config", handleConfig);
  server.on("/save-wifi", HTTP_POST, handleSaveWifi);
  server.on("/start", handleStart);
  server.on("/data", handleData);
  server.on("/getconfig", handleGetConfig);
  server.on("/setconfig", HTTP_POST, handleSetConfig);
  server.on("/reset-wifi", handleResetWifi);
  server.on("/status", handleStatus);
  server.on("/light", handleLight);
  
  server.begin();
  Serial.println("HTTP server started. Tricorder is ready.");
  
  // Final display update
  Wire.end();
  Wire.begin(OLED_SDA, OLED_SCL);
  updateDisplay();
  Wire.end();
  Wire.begin(SDA_PIN, SCL_PIN);
}

void loop() {
  server.handleClient();
  
  // Update display every 2 seconds
  if (millis() - lastDisplayUpdate > 2000) {
    Wire.end();
    Wire.begin(OLED_SDA, OLED_SCL);
    updateDisplay();
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    lastDisplayUpdate = millis();
  }
}