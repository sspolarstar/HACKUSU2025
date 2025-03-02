#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>  // Add Servo library for ESC control

/*
Weapon ESC is the little bee - BLHeli_s 30A ESC Signal pin is connected to Esp8266 pin D0.

Drive motor (N20) is controlled by an ESC (not a true ESC) H-bridge DRV8833. D5&D6 connect to in 1&2 and D7&D8 connect to in 3&4

Robot is powered by a 30C 2S 450 mAh Lipo battery

Bot will be remote controlled via ESP-now. The protocol should send a packet at 20Hz. (every 50ms). If no packet is recieved for over 1 second the bot must go into a shut down mode (wheel motors to 0 and weapon spinner to halted with breaking)
*/

// Correct GPIO pin definitions for ESP8266
// #define WeaponSignal 16  // D0 = GPIO 16
#define WheelLF 12      // D5 = GPIO 14
#define WheelLR 14      // D6 = GPIO 12
#define WheelRF 15      // D7 = GPIO 13
#define WheelRR 13      // D8 = GPIO 15

#define ServoLR 5      // D1 = GPIO 5
#define ServoUD 4     // D2 = GPIO 4

#define WHEEL_DIAMETER 30.0    // mm
#define AXLE_LENGTH 115.0      // mm

#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)

#define WHEEL_DIAMETER_MM 30.0
#define AXLE_LENGTH_MM 115.0
#define MOTOR_MAX_SPEED 1023  // Maximum speed for the motors

// Conversion factor for wheel speeds (linear and rotational components)
#define MM_TO_SPEED_FACTOR (MOTOR_MAX_SPEED / (WHEEL_DIAMETER_MM * PI))

//Packet Keys for data received
#define MESSAGE_KEY 0x5C077BAD
#define CAMERA_KEY 0xDEADBEEF
// Create servo object to control the weapon ESC
Servo udPWM;
Servo lrPWM;

// Struct to receive data 
struct __attribute__((packed)) Message{
  uint32_t key; //Code to manage access to device
  int16_t x1;  //Position vector - x desired (-1024 to 1024)
  int16_t x2;  //unused - always zero
  int16_t y1;  //Position vector - y desired (-1024 to 1024)
  int16_t y2;  //rotational vector - turns bot about center (-1024 to 1024)
  //uint16_t trigger; //unused - always zero
  bool     bumpL;   //Toggle weapon halt (0 - bot is not allowed to spin weapon, weapon breaks are engaged.) (1 - bot is spinning weapon)
  bool     bumpR;   //Weapon active toggles (0 - counter rotation) (1 - normal rotation)
  bool     stickL;  // (button is unreliable. best to ignore it.)
  bool     stickR;  //inverts robot position
};

// Struct to recieve camera correction vector
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
Message messageIn;
CorrectVect cameraIn;
bool newMessage = 0;
bool newCameraMessage = 0;

// Safety timeout variables
unsigned long lastMessageTime = 0;
unsigned long lastCamTime = 0;
const unsigned long CAM_TIMEOUT = 250;
const unsigned long TIMEOUT_DURATION = 1000; // 1 second timeout

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  // Copy incoming data
  //memcpy(&messageIn, incomingData, sizeof(messageIn));

  // Nicks psuedocode
  uint8_t msg_buffer[sizeof(Message)] = {0};
  memcpy(&msg_buffer, incomingData, len);

  uint32_t msg_key = msg_buffer[3] << 24 | msg_buffer[2] << 16 | msg_buffer[1] << 8 | msg_buffer[0];
  // Serial.printf("KEY FOUND = %x \n", msg_key);
  // this might also work: uint32_t msg_key = *((uint_32_t*)&msg_buffer);
  switch (msg_key) {
    case MESSAGE_KEY:
        // Serial.println("Message from Ctnl");
        // Message message = (Message)msg_buffer;
        // you might have to write this as Message message = *((Message*)&msg_buffer);
        memcpy(&messageIn, msg_buffer, sizeof(messageIn));
        newMessage = 1;
        lastMessageTime = millis(); // Update last message timestamp
        digitalWrite(BUILTIN_LED, LOW);
        break;
    case CAMERA_KEY:
        static int count;
        Serial.printf("count: %d", count);
        lastCamTime = millis();
        count++;
        // Serial.println("Message from cam");
        memcpy(&cameraIn, incomingData, sizeof(cameraIn));
        break;
    default:
      // Serial.println("bad key!");
      break;
  }

}


void setup() {
  // Initialize Serial first for debugging
  Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect
  // }
  // Serial.println("Starting up...");
  
  // Configure pins
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(WheelLF, OUTPUT);
  pinMode(WheelLR, OUTPUT);
  pinMode(WheelRF, OUTPUT);
  pinMode(WheelRR, OUTPUT);
  
  // Initialize arm PWM
  udPWM.attach(ServoUD);
  udPWM.writeMicroseconds(1500);
  lrPWM.attach(ServoLR);
  lrPWM.writeMicroseconds(1500);
  
  // Initialize all drive outputs to 0
  analogWrite(WheelLF, 0);
  analogWrite(WheelLR, 0);
  analogWrite(WheelRF, 0);
  analogWrite(WheelRR, 0);
  
  digitalWrite(BUILTIN_LED, LOW);
  
  // Set PWM frequency for drive motors
  analogWriteFreq(1000);
  
 // Set device as WiFi station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  } else{
    // Serial.println("esp now initialized");
  }

  // Set up receive callback
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}


void blink(){
  //change LED color
  if(digitalRead(BUILTIN_LED) == HIGH){
    digitalWrite(BUILTIN_LED, LOW);
  } else {
    digitalWrite(BUILTIN_LED, HIGH);
  }
}
// Function to control drive motors
void controlDriveMotors(int x, int y) {

  // Normalize inputs to the -1023 to 1023 range
  x = constrain(x, -1023, 1023)/32;
  y = constrain(y, -1023, 1023)/32;

  float leftSpeed;
  float rightSpeed;

    // Calculate wheel speeds
    leftSpeed  = y ;//+ (AXLE_LENGTH_MM / 2.0) * MM_TO_SPEED_FACTOR;
    rightSpeed = y ;//- (AXLE_LENGTH_MM / 2.0) * MM_TO_SPEED_FACTOR;

    // Add strafe component
    leftSpeed += x;
    rightSpeed -= x;

    // Normalize speeds to the motor maximum speed
    float maxMagnitude = max(abs(leftSpeed), abs(rightSpeed));
    if (maxMagnitude > MOTOR_MAX_SPEED) {
      leftSpeed = (leftSpeed / maxMagnitude) * MOTOR_MAX_SPEED;
      rightSpeed = (rightSpeed / maxMagnitude) * MOTOR_MAX_SPEED;
    }

    leftSpeed = -leftSpeed;
    rightSpeed = -rightSpeed;


  // Set motor speeds for the left side
  if (leftSpeed >= 0) {
    analogWrite(WheelLF, leftSpeed);
    analogWrite(WheelLR, 0);
  } else {
    analogWrite(WheelLF, 0);
    analogWrite(WheelLR, -leftSpeed);
  }

  // Set motor speeds for the right side
  if (rightSpeed >= 0) {
    analogWrite(WheelRF, rightSpeed);
    analogWrite(WheelRR, 0);
  } else {
    analogWrite(WheelRF, 0);
    analogWrite(WheelRR, -rightSpeed);
  }


}

void shutdownSystems() {
  // Stop all motors
  analogWrite(WheelLF, 0);
  analogWrite(WheelLR, 0);
  analogWrite(WheelRF, 0);
  analogWrite(WheelRR, 0);

}

void controlServoMotors(const CorrectVect& camera_data){

  // check confidence range is within 50%
  const bool is_confident = (camera_data.confidence.val > 50);
  // check if a target is in range
  const bool in_frame = camera_data.in_frame;

  // determine if we should attempt to target something
  const bool should_target = is_confident and in_frame;

  // --- up/down control ---
  const auto y_control = [&](){
    if (should_target) {
      return camera_data.y < 0 ? 1550 : 1450;
    } else {
      return 1500;
    }
  }();
  udPWM.writeMicroseconds(y_control);


  // --- left/right control ---
  const auto x_control = [&](){
    if (should_target) {
      return camera_data.x < 0 ? 1550 : 1450;
    } else {
      return 1500;
    }
  }();
  lrPWM.writeMicroseconds(x_control);
}


void loop() {
  
  //Control servos first
  if(messageIn.bumpR){
    if(messageIn.x2 < -500){
      lrPWM.writeMicroseconds(1600);
    } else if(messageIn.x2 > 500) {
      lrPWM.writeMicroseconds(1400);
    }else {
      lrPWM.writeMicroseconds(1500);
    }

    if(messageIn.y2 < -500){
      udPWM.writeMicroseconds(1600);
    } else if(messageIn.y2 > 500) {
      udPWM.writeMicroseconds(1400);
    } else {
      udPWM.writeMicroseconds(1500);
    }
  } else{

  
    if(millis() - lastCamTime > CAM_TIMEOUT ) {
      lrPWM.writeMicroseconds(1500);
      udPWM.writeMicroseconds(1500);
    } else {
      Serial.printf("running cam now");
      controlServoMotors(cameraIn);
    }
  }

  // Check for timeout
  if (millis() - lastMessageTime > TIMEOUT_DURATION) {
    shutdownSystems();
    digitalWrite(BUILTIN_LED, HIGH);//blink(); // Visual indication of timeout
    return;
  }


  if (newMessage == 1) {
    newMessage = 0;
    
    controlDriveMotors(messageIn.x1, messageIn.y1);
  }

  
}
