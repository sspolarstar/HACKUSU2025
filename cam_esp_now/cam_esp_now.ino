#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>

#define CHUNK_SIZE 240  // Bytes per chunk (8 bytes used by header; total < 250 bytes)

// Broadcast address for ESP-NOW (send to all peers)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Custom header for each image packet
typedef struct __attribute__((packed)) {
  uint16_t frameIndex;    // Frame counter
  uint16_t packetIndex;   // Packet number within the frame
  uint16_t totalPackets;  // Total packets for this frame
  uint16_t payloadSize;   // Number of valid data bytes in this packet
} image_packet_header_t;

uint16_t frameIndex = 0;

void initCamera() {
  camera_config_t config;
  // AI Thinker board pin configuration
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Set to QQVGA (160x120) for low resolution streaming.
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 10;  // Lower quality yields higher compression
  config.fb_count = 1;       // Use a single frame buffer

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true) { delay(1000); } // program killed
  }
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) { delay(1000); } //kill program
  }
  
  // Add the broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;    // Use the current WiFi channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
  }
}

void setup() {
  Serial.begin(115200);
  initCamera();
  initESPNow();
  Serial.println("ESP32-CAM ESP-NOW Streamer Started");
}

void loop() {
  uint32_t frameStart = millis();

  // Capture a frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    Serial.println("check for overheading");
    return;
  }
  
  // Calculate total number of packets needed for this frame
  uint16_t totalPackets = (fb->len + CHUNK_SIZE - 1) / CHUNK_SIZE;
  
  // Split and send each packet
  for (uint16_t packetIndex = 0; packetIndex < totalPackets; packetIndex++) {
    Serial.printf("sending %d of %d \n\r", packetIndex, totalPackets);
    uint16_t offset = packetIndex * CHUNK_SIZE;
    uint16_t remaining = fb->len - offset;
    uint16_t payloadSize = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
    
    // Create a buffer for the header plus payload
    uint8_t packetBuffer[sizeof(image_packet_header_t) + payloadSize];
    
    // Fill header information
    image_packet_header_t header;
    header.frameIndex = frameIndex;
    header.packetIndex = packetIndex;
    header.totalPackets = totalPackets;
    header.payloadSize = payloadSize;
    
    memcpy(packetBuffer, &header, sizeof(header));
    memcpy(packetBuffer + sizeof(header), fb->buf + offset, payloadSize);
    
    // Send packet via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, packetBuffer, sizeof(packetBuffer));
    if (result != ESP_OK) {
      Serial.printf("Error sending packet %d of frame %d: %d\n", packetIndex, frameIndex, result);
    }
    
    // A very brief delay can help avoid overwhelming the radio
    delay(1);
  }
  Serial.println("Image Sent");
  
  // Return the frame buffer to free memory
  esp_camera_fb_return(fb);
  
  frameIndex++;  // Increment frame counter
  
  // Maintain a target of 20 frames per second (approx. 50ms per frame)
  uint32_t frameTime = millis() - frameStart;
  if (frameTime < 50) {
    delay(50 - frameTime);
  }
}
