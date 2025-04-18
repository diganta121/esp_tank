#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <BTS7960.h>

// Motor pins
#define A1 27
#define A2 26
#define B1 25
#define B2 33

#define SPA 13
#define SPB 14

#define GUN 18

BTS7960 Rmotor(SPA, A1, A2);
BTS7960 Lmotor(SPB, B1, B2);

const unsigned int commandInterval = 25; // update time
unsigned long lastCommandTime = 0;
unsigned long lastCommandTime2 = 0; // for lost contact
unsigned long currMilis = 0;

uint8_t newMACAddress[] = {0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E};

typedef struct struct_message {
  int RState;
  int LState;
  int GunState;
} struct_message;

struct_message myData = {0, 0, 0};

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(myData)) {
    memcpy(&myData, incomingData, sizeof(myData));
    lastCommandTime2 = millis(); // reset timer on successful data
  } else {
    Serial.println("Received data size mismatch!");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(GUN, OUTPUT);

  WiFi.mode(WIFI_STA);
  Serial.println("Started");

  if (esp_wifi_set_mac(WIFI_IF_STA, newMACAddress) != ESP_OK) {
    Serial.println("Failed to set MAC address");
  } else {
    Serial.println("MAC address changed successfully");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  currMilis = millis();

  if (currMilis - lastCommandTime > commandInterval) {
    commands();
    lastCommandTime = millis();
  }

  // If no command received for more than 2 seconds, stop motors
  if (currMilis - lastCommandTime2 > 2000) {
    shutdownMotors();
  }

  // If no command received for more than 10 seconds, restart ESP
  if (currMilis - lastCommandTime2 > 10000) {
    Serial.println("Connection lost. Restarting ESP...");
    delay(500);
    esp_restart();
  }
}

void commands() {
  int R = constrain(myData.RState, -255, 255);
  int L = constrain(myData.LState, -255, 255);

  digitalWrite(GUN, (myData.GunState > 20) ? HIGH : LOW);

  if (abs(R) > 10) Rmotor.Enable(); else Rmotor.Disable();
  if (R > 10) Rmotor.TurnLeft(abs(R));
  else if (R < -10) Rmotor.TurnRight(abs(R));
  else Rmotor.Stop();

  if (abs(L) > 10) Lmotor.Enable(); else Lmotor.Disable();
  if (L > 10) Lmotor.TurnLeft(abs(L));
  else if (L < -10) Lmotor.TurnRight(abs(L));
  else Lmotor.Stop();

  Serial.print("L: ");
  Serial.print(L);
  Serial.print(" R: ");
  Serial.println(R);
}

void shutdownMotors() {
  Serial.println("No signal - shutting down motors.");
  Rmotor.Disable();
  Lmotor.Disable();
  Rmotor.Stop();
  Lmotor.Stop();
  digitalWrite(GUN, LOW);
}
