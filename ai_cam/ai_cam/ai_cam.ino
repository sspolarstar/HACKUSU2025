#include <nwad123-hackusu-25_inferencing.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/fomo.h>
#include <algorithm>
#include <numeric>
#include <esp_now.h>
#include <WiFi.h>

using eloq::camera;
using eloq::ei::fomo;

// ----- TYPES -----
struct point {
  int16_t x{ 0 };
  int16_t y{ 0 };

  static inline constexpr int16_t MAX_Y = 1023;
  static inline constexpr int16_t MAX_X = 1023;
  static inline constexpr int16_t MIN_Y = -1024;
  static inline constexpr int16_t MIN_X = -1024;

  point& operator=(const point& other) {
    x = std::clamp(other.x, MIN_X, MAX_X);
    y = std::clamp(other.y, MIN_Y, MAX_Y);
    return *this;
  }

  point& operator=(point&& other) noexcept {
    x = std::clamp(other.x, MIN_X, MAX_X);
    y = std::clamp(other.y, MIN_Y, MAX_Y);
    return *this;
  }

  point(const int16_t x, const int16_t y)
    : x(std::clamp(x, MIN_X, MAX_X)), y(std::clamp(y, MIN_Y, MAX_Y)) {}
  point(const point& other)
    : x(std::clamp(other.x, MIN_X, MAX_X)), y(std::clamp(other.y, MIN_Y, MAX_Y)) {}
  point(point&& other) noexcept
    : x(std::clamp(other.x, MIN_X, MAX_X)), y(std::clamp(other.y, MIN_Y, MAX_Y)) {}
};

using msg_key_t = uint32_t;

struct __attribute__((packed)) position
{
  msg_key_t msg_key{0xDEADBEEF};
  int16_t x;
  int16_t y;

  static constexpr inline auto NOT_IN_FRAME = std::numeric_limits<int16_t>::max();
};

struct sender 
{
  static constexpr inline uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
};

// ----- GLOBAL / STATIC VARS -----
esp_now_peer_info_t peerInfo = {0};

// ----- FUNCTIONS -----

/**
 * @brief Initializes and starts the camera and object detection model.
 * 
 * This function prints status messages at each step of initialization.
 */
__attribute__((always_inline)) 
auto start_camera() -> void {
  Serial.println("----- CAMERA & OBJ DETECTION MODEL STARTUP -----");

  // cam setup
  camera.pinout.aithinker();
  camera.brownout.disable();

  // Object detection setup
  camera.resolution.yolo();
  camera.pixformat.rgb565();

  // Camera initialization
  while (!camera.begin().isOk()) {
    Serial.println(camera.exception.toString());
  }

  // Camera status message
  Serial.println("[CAM] Camera Startup OK");
}

/**
 * @brief Locates a target object in the camera's view.
 * 
 * @return std::optional<point> The coordinates of the target object if found, otherwise std::nullopt.
 */
[[nodiscard]] __attribute__((always_inline)) 
auto locate_target() -> std::optional<point> {
  // capture a single frame, if the frame fails return nullopt
  if (!camera.capture().isOk()) {
    Serial.println(camera.exception.toString());
    return std::nullopt;
  }

  // run image detection on the model, if image detection
  // fails, return nullopt
  if (!fomo.run().isOk()) {
    Serial.println(fomo.exception.toString());
    return std::nullopt;
  }

  // print a status message about the image recognition
  Serial.printf("[CAM] Image recognition completed in %dm.\n",
                fomo.benchmark.millis());

  // if image detection succeeded but we didn't recognize an object
  // also return a nullopt
  if (!fomo.foundAnyObject()) {
    Serial.println("[CAM] No objects in view.");
    return std::nullopt;
  }

  // TODO: if we have multiple matches this ignores them, we may want to change
  // this?

  /*
  When an object is recognized a "bounding box" is returned by the 
  fomo detection model. The bounding box coordinates are similar to
  the image coordinates, so we need to map them from that coordinate
  system to a new one:

  (0,0)         (image.width,0)
    +------------+
    | (x,y)      |
    |    +----+  |
    |    |    |  |
    |    +----+  |
    |            |
    +------------+
  (0,image.height)

  */

  // Capture the target bounding box
  const auto target_bounding_box = fomo.at(0);

  // Derive the X,Y point of the center of the object relative to the
  // center of the camera view. We represent this point as the
  // 2d vector x,y
  const auto img_width = camera.resolution.getWidth();
  const auto img_height = camera.resolution.getHeight();

  // the first portion of this equation, b.x + (b.width / 2), locates
  // point at the center of the bounding box, and the second part of
  // the equation, - (img_width / 2), moves the point relative to the
  // center of the camera view
  const auto x = target_bounding_box.x + (target_bounding_box.width / 2) - (img_width / 2);
  const auto y = target_bounding_box.y + (target_bounding_box.height / 2) - (img_height / 2);

  // Now we need to scale this code from the range of image width
  // to the range set by the point class
  const auto x_scaled = x * point::MAX_X / img_width;
  const auto y_scaled = y * point::MAX_Y / img_height;

  // Print out a status message about the point found
  Serial.printf("[CAM] Located object at (x, y): (%d, %d).\n",
                x_scaled, y_scaled);

  return point{ x_scaled, y_scaled };
}

/**
 * @brief Callback function for ESP-NOW send status.
 * 
 * @param mac_addr The MAC address of the device the message was sent to.
 * @param status The status of the send operation.
 */
auto on_send(const uint8_t *mac_addr, esp_now_send_status_t status) -> void {
  Serial.print("[ESP NOW] Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

/**
 * @brief Initializes and starts ESP-NOW for wireless communication.
 * 
 * This function sets the device as a Wi-Fi station, initializes ESP-NOW,
 * registers a callback for send status, and adds a peer for communication.
 */
__attribute__((always_inline))
auto start_esp_now() -> void
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP NOW] Error initializing ESP-NOW");
    return;
  }

  // Set up the callback for every packet transmission
  esp_now_register_send_cb(on_send);

  // Register peer
  // TODO: since we know who our peer is not sure why don't 
  // just do this at compile time
  memcpy(peerInfo.peer_addr, sender::broadcast_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("[ESP NOW] Failed to add peer");
  }

  // Status message 
  Serial.println("[ESP NOW] ESP Now Startup OK");
}

// ----- SETUP -----
void setup() {
  delay(1000);
  Serial.begin(115200);

  start_camera();
  start_esp_now();
}

// ----- LOOP -----
void loop() {
  const auto star_loc = locate_target();

  // set up a message position to send across esp now
  position msg;

  // if the star was located, set the messages x and y values
  if (star_loc.has_value()) {
    msg.x = star_loc->x;
    msg.y = star_loc->y;
  }
  else {
    msg.x = position::NOT_IN_FRAME;
    msg.y = position::NOT_IN_FRAME;
  }

  // Send the message out over esp now
  const auto result = esp_now_send(sender::broadcast_address, (uint8_t *) &msg, sizeof(msg));
   
  if (result == ESP_OK) {
    Serial.println("[ESP NOW] Sent position with success.");
  }
  else {
    Serial.println("[ESP NOW] Error sending the position.");
  }
}