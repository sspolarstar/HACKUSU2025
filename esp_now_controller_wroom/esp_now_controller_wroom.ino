//This file constains the code that programs the Controller

#include <esp_now.h>
#include <WiFi.h>


#define DEBOUNCE_DELAY 20  // 20 ms debounce delay


struct __attribute__((packed)) Message {
  uint32_t key;    // Access code
  int16_t  x1;     // Position vector - x desired (-1024 to 1024)
  int16_t  x2;     // unused - always zero
  int16_t  y1;     // Position vector - y desired (-1024 to 1024)
  int16_t  y2;     // rotational vector - turns bot about center (-1024 to 1024)
  bool     bumpL;   // Toggle weapon halt
  bool     bumpR;   // Weapon direction toggle
  bool     stickL;  // (unreliable - ignored)
  bool     stickR;  // inverts robot position
};

Message data;

// Broadcast MAC address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Pin definitions
const int PIN_BUMPR = 26;
const int PIN_BUMPL = 27;

const int PIN_STICKL = 33;
const int PIN_STICKR = 32;

const int PIN_ANALOG_Y1 = 34;
const int PIN_ANALOG_X1 = 39;

const int PIN_ANALOG_Y2 = 36;
const int PIN_ANALOG_X2 = 35;

bool bumpr_state = false, bumpl_state = false, stickl_state = false, stickr_state = false;
unsigned long bumpr_last_change = 0, bumpl_last_change = 0, stickl_last_change = 0, stickr_last_change = 0;


// Joystick calibration ranges
struct JoystickCalibration {
    int min;
    int max;
    int deadzone;
};

const JoystickCalibration X1_CAL = {0, 4096, 120};    // X1 has 0-3200 range
const JoystickCalibration Y1_CAL = {0, 4096, 130};    // Y1 has 100-4100 range
const JoystickCalibration X2_CAL = {0, 4096, 120};    // Y2 has 0-3740 range
const JoystickCalibration Y2_CAL = {0, 4096, 115};    // Y2 has 0-3740 range

// Callback for when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {  
  // Initialize data structure
  data.key = 0x5C077BAD;
  
  // Configure input pins
  pinMode(PIN_BUMPR, INPUT_PULLUP);
  pinMode(PIN_BUMPL, INPUT_PULLUP);
  pinMode(PIN_STICKL, INPUT_PULLUP);
  pinMode(PIN_STICKR, INPUT_PULLUP);

  // Set up WiFi station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    return;
  }

  // Register callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return;
  }

  // Debug output of calibration ranges
  Serial.println("Calibration ranges:");
  Serial.printf("X1: %d to %d\n", X1_CAL.min, X1_CAL.max);
  Serial.printf("Y1: %d to %d\n", Y1_CAL.min, Y1_CAL.max);
  Serial.printf("Y2: %d to %d\n", Y2_CAL.min, Y2_CAL.max);
}

// Function to handle joystick reading with calibration and deadzone
int16_t readJoystick(int pin, const JoystickCalibration &cal) {
  int raw = analogRead(pin);
  int mapped = map(raw, cal.min, cal.max, 1024, -1024);
  mapped = constrain(mapped, -1024, 1024);
  
  if (abs(mapped) < cal.deadzone) {
    return 0;
  }
  return mapped;
}

void loop() {
  unsigned long current_time = millis();

  // Debounce PIN_BUMPR
  bool bumpr_reading = !digitalRead(PIN_BUMPR);
  if (bumpr_reading != bumpr_state && (current_time - bumpr_last_change) > DEBOUNCE_DELAY) {
    bumpr_last_change = current_time;
    bumpr_state = bumpr_reading;
    if (bumpr_state) {  // Toggle only on state change
      data.bumpR = !data.bumpR;
    }
  }

  // Debounce PIN_BUMPL
  bool bumpl_reading = !digitalRead(PIN_BUMPL);
  if (bumpl_reading != bumpl_state && (current_time - bumpl_last_change) > DEBOUNCE_DELAY) {
    bumpl_last_change = current_time;
    bumpl_state = bumpl_reading;
    if (bumpl_state) {  // Toggle only on state change
      data.bumpL = !data.bumpL;
    }
  }

  // Debounce PIN_STICKL
  bool stickl_reading = !digitalRead(PIN_STICKL);
  if (stickl_reading != stickl_state && (current_time - stickl_last_change) > DEBOUNCE_DELAY) {
    stickl_last_change = current_time;
    stickl_state = stickl_reading;
    if (stickl_state) {  // Toggle only on button press (not release)
      data.stickL = !data.stickL;
    }
  }

  // Debounce PIN_STICKR
  bool stickr_reading = !digitalRead(PIN_STICKR);
  if (stickr_reading != stickr_state && (current_time - stickr_last_change) > DEBOUNCE_DELAY) {
    stickr_last_change = current_time;
    stickr_state = stickr_reading;
    if (stickr_state) {  // Toggle only on button press (not release)
      data.stickR = !data.stickR;
    }
  }

  // Read analog inputs with their specific calibrations
  data.x1 = readJoystick(PIN_ANALOG_X1, X1_CAL);
  data.y1 = readJoystick(PIN_ANALOG_Y1, Y1_CAL);
  data.x1 = readJoystick(PIN_ANALOG_X2, X2_CAL);
  data.y2 = readJoystick(PIN_ANALOG_Y2, Y2_CAL);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));
  
  if (result != ESP_OK) {
    //correction logic... if I cared.
  }

  // Maintain 20Hz update rate
  delay(50);
}