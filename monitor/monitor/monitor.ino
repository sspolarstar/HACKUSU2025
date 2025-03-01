#include <esp_now.h>
#include <WiFi.h>

#define CAMERA_KEY 0xDEADBEEF

struct __attribute__((packed)) CorrectVect
{
  struct __attribute__((packed)) percentage {
    uint8_t val{0};

    constexpr explicit percentage(const uint8_t val) : val(std::clamp(val, MIN, MAX)) {}

    static constexpr inline uint8_t MIN = 0;
    static constexpr inline uint8_t MAX = 100;
  };

  uint32_t msg_key{0xDEADBEEF};
  int16_t x;
  int16_t y;
  percentage confidence{0};
  bool in_frame{false};
};

CorrectVect cameraIn;
bool newCameraMessage = 0;

// Callback when data is received
void OnDataRecv(const esp_now_recv_info* mac, const uint8_t *incomingData, int len) {
  // Copy incoming data
  //memcpy(&messageIn, incomingData, sizeof(messageIn));

  // Nicks psuedocode
  uint8_t msg_buffer[32] = {0};
  memcpy(&msg_buffer, incomingData, len);

  uint32_t msg_key = msg_buffer[3] << 24 | msg_buffer[2] << 16 | msg_buffer[1] << 8 | msg_buffer[0];
  switch (msg_key) {
    case CAMERA_KEY:
        memcpy(&cameraIn, incomingData, sizeof(cameraIn));
        newCameraMessage = 1;
        break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set up receive callback  
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Configuration OK...");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(newCameraMessage == 1) {
    newCameraMessage = 0;
    Serial.printf("Msg:\n"
                  "  x: %d\n"
                  "  y: %d\n"
                  "  confidence: %d\n"
                  "  in_frame: %d\n",
                  cameraIn.x, cameraIn.y, cameraIn.confidence.val, cameraIn.in_frame);
  }

}
