#ifndef CONFIG_H
#define CONFIG_H

// Include libraries INSIDE the header guard
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// I2C pins for AS7341 sensor
#define SDA_PIN 2
#define SCL_PIN 3

// Global objects
extern WebServer server;
extern Preferences preferences;

// WiFi configuration
extern String ssid;
extern String password;
extern bool apMode;

// Data collection variables
extern bool collecting;
extern unsigned int accumulationTime;
extern const int interpolationPoints;
extern float interpolatedSpectrum[100][2];

// Function declarations from tricorder.ino
void readAndAccumulate();
void generateInterpolatedSpectrum();
float getInterpolatedValue(float wavelength);
float cubicInterpolate(float p0, float p1, float p2, float p3, float t);

// Function declarations from web_handlers.ino
void handleRoot();
void handleConfig();
void handleSaveWifi();
void handleStart();
void handleData();
void handleGetConfig();
void handleSetConfig();
void handleResetWifi();
void startAPMode();

#endif