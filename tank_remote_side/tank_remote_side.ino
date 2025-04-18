// tank remote side dig
#include <esp_now.h>
#include <WiFi.h>

const int deadzone = 10;

const int RAxisPin = 33;
const int LAxisPin = 32;

const int LSPButton = 22;
const int RSPButton = 21;

#define LED_PIN 19

int Rvalue = 0;
int Lvalue = 0;

// bool RFB = false;
// bool RBB = false;

// bool LFB = false;
// bool LBB = false;

int Rsp = 1;
int Lsp = 1;



// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };  // car

const int send_rate = 30;
int sendvar = 0;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int RState;  // -2, -1 , 0 ,1 , 2
  int LState;
  int GunState;
} struct_message;

// Create a struct_message called Data & peer
struct_message Data = {0 , 0 , 0};
struct_message PrevData;
esp_now_peer_info_t peerInfo;

// Variables to store default joystick positions (calibration offsets)
int defaultRvalue = 0;
int defaultLvalue = 0;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  status == ESP_NOW_SEND_SUCCESS ? digitalWrite(LED_PIN, HIGH) : errBlink();
}

void errBlink(){
  for (int i = 0; i < 3; i++) {  // Blink LED for failure
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// Function to calibrate joystick
void calibrateJoystick() {
    int totalR = 0, totalL = 0;
    int samples = 7; // Number of samples to average for calibration

    Serial.println("Calibrating joystick...");
    for (int i = 0; i < samples; i++) {
        totalR += analogReadSmooth(RAxisPin);
        totalL += analogReadSmooth(LAxisPin);
        delay(10); // Small delay between samples
    }

    // Calculate average default position
    defaultRvalue = 2047 - (totalR / samples);
    defaultLvalue = 2047- (totalL / samples);

    Serial.print("Default RAxis: ");
    Serial.println(defaultRvalue);
    Serial.print("Default LAxis: ");
    Serial.println(defaultLvalue);
}

int climit(int n) {
  if (n >= 255) {
    return 255;
  } 
  else if (n <= -255) {
    return -255;
  } 
  else {
    return n;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(RAxisPin, INPUT);
  pinMode(LAxisPin, INPUT);

  // pinMode(RBButton, INPUT);
  // pinMode(LBButton, INPUT);

  pinMode(RSPButton, INPUT);
  pinMode(LSPButton, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  // Calibrate joystick
  calibrateJoystick();
}


bool button_state(int a) {
  int s = 0;
  for (int i = 0; i < 4; i++) {
    s += digitalRead(a);
    delay(3);
  }
  return (s > 2) ? true : false;
}

int analogReadSmooth(int pin) {
    int total = 0;
    for (int i = 0; i < 3; i++) {
        total += analogRead(pin);
        delay(5);
    }
    return total / 3;
}



// bool DataDiff(){
//   // to check if Data and dataprev is different
//   if(Data.LState == PrevData.LState && Data.RState == PrevData.RState){
//     return false;
//   }
//   PrevData.LState = Data.LState;
//   PrevData.RState = Data.RState;
//   return true;
// }

int stick_value(int sp) {
  int abs_sp = abs(sp);
  int output = 0;

  if (abs_sp <= deadzone) {  // Deadzone
    output = 0;
  } else if (abs_sp <= 250) {                // analog range
    output = map(abs_sp, deadzone, 255, 30, 255);  // Scale to 70-280
  }
  else{
    output = 255;
  }

  if (sp <0){
    output *= -1;
  }
  return output;
}

void loop() {
  Rvalue = analogReadSmooth(RAxisPin) + defaultRvalue;
  Lvalue = analogReadSmooth(LAxisPin) + defaultLvalue;

  Rsp = !button_state(RSPButton);
  Lsp = !button_state(LSPButton);

  Serial.print(Rvalue);
  Serial.print(" ");
  Serial.print(Lvalue);
  Serial.print(" ");

  Rvalue = climit(stick_value(map(Rvalue,4095-defaultRvalue,defaultRvalue,-255,255))); // reversed //stick_value(Rvalue);
  Lvalue = climit(stick_value(map(Lvalue,4095-defaultLvalue,defaultLvalue,255,-255)));//stick_value(Lvalue);

  if (Rsp) {
    Data.GunState = 255;
  }
  else{
    Data.GunState = 0;
  }

  if (Lsp) {
    Rvalue = Rvalue;
    Lvalue = Lvalue;
  }
  else{
    Rvalue = Rvalue/2;
    Lvalue = Lvalue/2;
  }

  Data.RState = Rvalue;
  Data.LState = Lvalue;
  
  Serial.print(Rvalue);
  Serial.print(" ");
  Serial.println(Lvalue);

  // === Send message via ESP-NOW ===
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Data, sizeof(Data));

  if (result == ESP_OK) {
    Serial.println("S");
  } else {
    Serial.println("E");
  }

  delay(5);
}
