#include <WiFi.h>
#include <esp_now.h>
#include <TFT_eSPI.h>      // TFT library with DMA support (if configured)
#include <JPEGDecoder.h>   // JPEG decoding library

// Maximum JPEG image size (adjust as needed)
#define MAX_JPEG_SIZE 65536

// Expected JPEG image dimensions (set transmitter accordingly)
// Here we assume 80x60 for a smaller image; update as needed.
#define IMG_WIDTH 160
#define IMG_HEIGHT 120
#define FRAMEBUFFER_SIZE (IMG_WIDTH * IMG_HEIGHT * sizeof(uint16_t))

// ESP-NOW image packet header (8 bytes)
typedef struct __attribute__((packed)) {
  uint16_t frameIndex;    // Frame counter
  uint16_t packetIndex;   // Packet number within the frame
  uint16_t totalPackets;  // Total packets for this frame
  uint16_t payloadSize;   // Valid payload size in this packet
} image_packet_header_t;

// Global buffer for assembling JPEG data
uint8_t jpegBuffer[MAX_JPEG_SIZE];
volatile uint32_t jpegBufferLen = 0;
volatile uint16_t currentFrameIndex = 0;
volatile bool frameComplete = false;

// Double-buffering: two framebuffers for the decoded image
uint16_t* frameBuffer0 = nullptr;
uint16_t* frameBuffer1 = nullptr;
uint16_t* currentDrawBuffer = nullptr;  // buffer into which we decode the new frame
uint16_t* displayBuffer     = nullptr;  // buffer currently displayed

TFT_eSPI tft = TFT_eSPI();

// ESP-NOW receive callback (using updated signature)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  if (data_len < sizeof(image_packet_header_t)) return;
  
  image_packet_header_t header;
  memcpy(&header, data, sizeof(header));
  
  // If a new frame starts, reset the JPEG buffer
  if (header.frameIndex != currentFrameIndex) {
    currentFrameIndex = header.frameIndex;
    jpegBufferLen = 0;
  }
  
  // Append payload data from the packet
  if (jpegBufferLen + header.payloadSize < MAX_JPEG_SIZE) {
    memcpy(jpegBuffer + jpegBufferLen, data + sizeof(header), header.payloadSize);
    jpegBufferLen += header.payloadSize;
  }
  
  // When the final packet for this frame is received...
  if (header.packetIndex == header.totalPackets - 1) {
    frameComplete = true;
  }
}

// Decode the JPEG image into the provided off-screen buffer.
// This function iterates through each MCU block from the JPEG decoder
// and writes the pixel data into the buffer.
void decodeToBuffer(uint16_t* buffer, int imageWidth, int imageHeight) {
  // (Optional) Clear the buffer.
  memset(buffer, 0, imageWidth * imageHeight * sizeof(uint16_t));
  
  // Process each MCU block
  while (JpegDec.read()) {
    int mcu_x = JpegDec.MCUx * JpegDec.MCUWidth;
    int mcu_y = JpegDec.MCUy * JpegDec.MCUHeight;
    int blockWidth  = (mcu_x + JpegDec.MCUWidth  > imageWidth)  ? (imageWidth  - mcu_x) : JpegDec.MCUWidth;
    int blockHeight = (mcu_y + JpegDec.MCUHeight > imageHeight) ? (imageHeight - mcu_y) : JpegDec.MCUHeight;
    
    for (int y = 0; y < blockHeight; y++) {
      int destIndex = (mcu_y + y) * imageWidth + mcu_x;
      int srcIndex = y * JpegDec.MCUWidth;
      memcpy(buffer + destIndex, JpegDec.pImage + srcIndex, blockWidth * sizeof(uint16_t));
    }
  }
}

// Process a complete JPEG frame: decode into the offscreen buffer, swap buffers, and display.
void processFrame() {
  Serial.printf("Frame %d complete; size: %d bytes\n", currentFrameIndex, jpegBufferLen);
  
  if (JpegDec.decodeArray(jpegBuffer, jpegBufferLen)) {
    // In this example we expect the JPEG image dimensions to match IMG_WIDTH and IMG_HEIGHT.
    int imageW = JpegDec.width;
    int imageH = JpegDec.height;
    
    // Decode the JPEG image into the current drawing buffer
    decodeToBuffer(currentDrawBuffer, imageW, imageH);
    
    // Swap the buffers so that the freshly decoded frame becomes the display buffer
    uint16_t* temp = currentDrawBuffer;
    currentDrawBuffer = displayBuffer;
    displayBuffer = temp;
    
    // Center the image on the TFT display
    int16_t x = (tft.width()  - imageW) / 2;
    int16_t y = (tft.height() - imageH) / 2;
    
    // Push the complete image to the display in one DMA-optimized call.
    tft.pushImage(x, y, imageW, imageH, displayBuffer);
  } else {
    Serial.println("JPEG decode failed.");
  }
  
  // Reset for the next frame
  frameComplete = false;
  jpegBufferLen = 0;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize the TFT display
  tft.init();
  tft.setRotation(1);       // Landscape orientation
  tft.setSwapBytes(true);   // Adjust color byte order if necessary for your display
  tft.fillScreen(TFT_BLACK);
  
  // Allocate the double framebuffers
  frameBuffer0 = (uint16_t*)malloc(FRAMEBUFFER_SIZE);
  frameBuffer1 = (uint16_t*)malloc(FRAMEBUFFER_SIZE);
  if (!frameBuffer0 || !frameBuffer1) {
    Serial.println("Failed to allocate framebuffers");
    while (true) { delay(1000); }
  }
  currentDrawBuffer = frameBuffer0;
  displayBuffer     = frameBuffer1;
  
  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("CYD ESP-NOW Receiver with Double Buffering Started");
}

void loop() {
  if (frameComplete) {
    processFrame();
  }
}
