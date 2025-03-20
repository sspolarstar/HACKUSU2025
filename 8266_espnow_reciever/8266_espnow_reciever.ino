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
#define WeaponSignal 4  // D0 = GPIO 16
#define WheelLF 12      // D5 = GPIO 14
#define WheelLR 14      // D6 = GPIO 12
#define WheelRF 15      // D7 = GPIO 13
#define WheelRR 13      // D8 = GPIO 15

// #define ServoLR 5      // D1 = GPIO 5
// #define ServoUD 4     // D2 = GPIO 4

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
// #define CAMERA_KEY 0xDEADBEEF
// Create servo object to control the weapon ESC
// Servo udPWM;
// Servo lrPWM;
Servo weaponESC;
// Struct to receive data 
struct __attribute__((packed)) Message{
  uint32_t key; //Code to manage access to device
  int16_t x1;  //Position vector - x desired (-1024 to 1024)
  int16_t x2;  //unused - always zero
  int16_t y1;  //Position vector - y desired (-1024 to 1024)
  int16_t y2;  //rotational vector - turns bot about center (-1024 to 1024)
  bool     bumpL;   //Toggle weapon halt (0 - bot is not allowed to spin weapon, weapon breaks are engaged.) (1 - bot is spinning weapon)
  bool     bumpR;   //Weapon active toggles (0 - counter rotation) (1 - normal rotation)
  bool     stickL;  //swaps drive modes
  bool     stickR;  //inverts robot position
};

// Struct to recieve camera correction vector
Message messageIn;

bool newMessage = 0;
bool newCameraMessage = 0;

// Safety timeout variables
unsigned long lastMessageTime = 0;
const unsigned long TIMEOUT_DURATION = 1000; // 1 second timeout

bool weaponEnabled = false;
bool weaponSpeed = false; // true = fast, false = slow

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  // Copy incoming data
  memcpy(&messageIn, incomingData, sizeof(messageIn));
  if(messageIn.key != 0x5C077BAD){
    Serial.println("key Rejected");
    return;
  }
  newMessage = 1;
  lastMessageTime = millis(); // Update last message timestamp
  digitalWrite(BUILTIN_LED, LOW);
  static int count = 0;
  // Print received message
  // Serial.println("Received message: " + String(count));
  count++;

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
  
  // Initialize weapon ESC
  weaponESC.attach(WeaponSignal);
  weaponESC.writeMicroseconds(1000);  // Set to zero throttle

  // Initialize all drive outputs to 0
  analogWrite(WheelLF, 0);
  analogWrite(WheelLR, 0);
  analogWrite(WheelRF, 0);
  analogWrite(WheelRR, 0);
  
  digitalWrite(BUILTIN_LED, LOW);
  
  // Set PWM frequency for drive motors
  analogWriteFreq(30000);
  
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

void controlWeapon() {
  if (!weaponEnabled || !messageIn.bumpL) {
    weaponESC.writeMicroseconds(1000);  // Zero throttle with brake
    return;
  }
  
  // If weapon enabled and allowed to spin
  int weaponSpeed = messageIn.bumpR ? 2000 : 1200;  // Full throttle or slow speed
  weaponESC.writeMicroseconds(weaponSpeed);
}

void blink(){
  //change LED color
  if(digitalRead(BUILTIN_LED) == HIGH){
    digitalWrite(BUILTIN_LED, LOW);
  } else {
    digitalWrite(BUILTIN_LED, HIGH);
  }
}

// Enhanced drive motor control function with active braking
void controlDriveMotors(int x, int y, int y2) {

    // Properly constrain all inputs
    if(y < -800){
      y = 1023;
    }

    x = x/1.2;

    // Declare speed variables
    int leftSpeed = 0;
    int rightSpeed = 0;

    // Enhanced control mixing based on drive mode
    if (messageIn.stickL == 1) {
        // Tank drive mode
        leftSpeed = y;
        rightSpeed = y2;
    } else {
        // Standard arcade drive mode
        leftSpeed = y;
        rightSpeed = y;
        
        // Add rotation component
        leftSpeed += x;
        rightSpeed -= x;

    }

    // Apply direction inversion if needed
    if (messageIn.stickR != 1) {
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
    }

    // Re-constrain after all calculations

    // Normal motor control (when not actively braking)
    // Set motor speeds for the left side
    // if (leftSpeed >= 0) {
    //     analogWrite(WheelLF, 1023-leftSpeed);
    //     analogWrite(WheelLR, 900);
    // } else {
    //     analogWrite(WheelLF, 900);
    //     analogWrite(WheelLR, 1023+leftSpeed);
    // }

    // // Set motor speeds for the right side
    // if (rightSpeed >= 0) {
    //     analogWrite(WheelRF, 1023-rightSpeed);
    //     analogWrite(WheelRR, 900);
    // } else {
    //     analogWrite(WheelRF, 900);
    //     analogWrite(WheelRR, 1023+rightSpeed);
    // }

    leftSpeed = pow(leftSpeed / 1023.0, 3) * 1023.0;
    rightSpeed = pow(rightSpeed / 1023.0, 3) * 1023.0;

    leftSpeed = constrain(leftSpeed, -1023, 1023);
    rightSpeed = constrain(rightSpeed, -1023, 1023);
    Serial.printf("y: %d, \t x: %d, \t Left speed: %d, \t RightSpeed: %d \n", y, x, leftSpeed, rightSpeed);

    const int Speed_Control = 110;

    if (leftSpeed >= 0) {
        leftSpeed = leftSpeed < Speed_Control ? Speed_Control : leftSpeed;
        analogWrite(WheelLF, leftSpeed);
        analogWrite(WheelLR, Speed_Control);
    } else {
        leftSpeed = leftSpeed > -Speed_Control ? -Speed_Control : leftSpeed;
        analogWrite(WheelLF, Speed_Control);
        analogWrite(WheelLR, -leftSpeed);
    }

    // Set motor speeds for the right side
    if (rightSpeed >= 0) {
        rightSpeed = rightSpeed < Speed_Control ? Speed_Control : rightSpeed;
        analogWrite(WheelRF, rightSpeed);
        analogWrite(WheelRR, Speed_Control);
    } else {
        rightSpeed = rightSpeed > -Speed_Control ? -Speed_Control : rightSpeed;
        analogWrite(WheelRF, Speed_Control);
        analogWrite(WheelRR, -rightSpeed);
    }
}



void shutdownSystems() {
  // Stop all motors
  weaponESC.writeMicroseconds(1000);
  
  applyActiveBrake();
}


// You can optionally add this helper function to implement active braking directly
void applyActiveBrake() {
    // This function applies brief opposing force to quickly stop motors
    // Apply full power in opposite direction
    analogWrite(WheelLF, 1023);
    analogWrite(WheelLR, 1023);
    analogWrite(WheelRF, 1023);
    analogWrite(WheelRR, 1023);
    
}


void loop() {
   // Check for timeout
  if (millis() - lastMessageTime > TIMEOUT_DURATION) {
    
    shutdownSystems();
    digitalWrite(BUILTIN_LED, HIGH);//blink(); // Visual indication of timeout
    return;
  }
  
  if (newMessage == 1) {
    newMessage = 0;
    
    // Update weapon state
    weaponEnabled = messageIn.bumpL;
    
    // Control systems
    controlWeapon();
    controlDriveMotors(messageIn.x1, messageIn.y1, messageIn.y2);
  }

  
  
}
