#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>  // Servo library for ESC control

/*
Weapon ESC is the little bee - BLHeli_s 30A ESC.
Drive motors are now controlled via a 2‑port ESC (Payne RC) that accepts PWM signals:
   - Reverse: 800–1100 μs (800 = full reverse)
   - Neutral: ~1500 μs
   - Forward: 1900–2200 μs (2200 = full forward)
The signal wires are connected to a D1 Mini clone.
Remote control is via ESP‑NOW, with a 20Hz update rate.
A 1‑second timeout stops the drive and weapon.
*/

// --- Pin definitions ---
// Weapon ESC
#define WeaponSignal 14   // (D0 equivalent)

// Drive ESC signal pins (using the new ESC for drive motors)
#define LeftSignal 4     
#define RightSignal 5    

// Packet Keys for data received
#define MESSAGE_KEY 0x5C077BAD

// Create servo objects for the weapon and drive ESCs
Servo weaponESC;
Servo leftESC;
Servo rightESC;

// Struct to receive data 
struct __attribute__((packed)) Message{
  uint32_t key;     // Code to manage access to device
  int16_t x1;       // Position vector - x desired (-1024 to 1024)
  int16_t x2;       // Unused – always zero
  int16_t y1;       // Position vector – y desired (-1024 to 1024)
  int16_t y2;       // Rotational vector – turns bot about center (-1024 to 1024)
  bool    bumpL;    // Toggle weapon halt (0: weapon halted, 1: weapon allowed)
  bool    bumpR;    // Weapon active toggles (0: counter rotation, 1: normal rotation)
  bool    stickL;   // Swaps drive modes (tank vs. arcade)
  bool    stickR;   // Inverts robot drive direction
};

Message messageIn;
bool newMessage = false;

// Timeout variables
unsigned long lastMessageTime = 0;
const unsigned long TIMEOUT_DURATION = 1000; // 1 second timeout

// For drive control scaling and deadzone
const int MOTOR_MAX = 1023;
const int DEADZONE = 50;  // Small deadzone around 0

// Callback when data is received via ESP‑NOW
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&messageIn, incomingData, sizeof(messageIn));
  if(messageIn.key != MESSAGE_KEY) {
    Serial.println("Key rejected");
    return;
  }
  newMessage = true;
  lastMessageTime = millis(); // Update last message time
  digitalWrite(BUILTIN_LED, LOW);
}

void setup() {
  Serial.begin(115200);
  
  // Set built-in LED for debugging
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  
  // Initialize drive ESCs (as servos)
  leftESC.attach(LeftSignal, 800, 2200);
  rightESC.attach(RightSignal, 800, 2200);
  
  // Initialize drive motors to neutral (1500 μs)
  leftESC.writeMicroseconds(1500);
  rightESC.writeMicroseconds(1500);
  
  // Initialize weapon ESC
  weaponESC.attach(WeaponSignal);
  weaponESC.writeMicroseconds(1000);  // Zero throttle for weapon
  
  // Set device as WiFi station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP‑NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP‑NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

// Control the weapon ESC (unchanged from before)
void controlWeapon() {
  if (!messageIn.bumpL) {
    weaponESC.writeMicroseconds(1000);  // Zero throttle with brake
    return;
  }
  
  // If weapon is enabled and allowed to spin
  int weaponSpeed = messageIn.bumpR ? 2000 : 1200;  // Fast or slow speed
  weaponESC.writeMicroseconds(weaponSpeed);
}

// Map a motor speed (-MOTOR_MAX to MOTOR_MAX) to the appropriate ESC pulse
int speedToPulse(int speed) {
  // Use 1500 μs for neutral
  if (abs(speed) < DEADZONE) return 1500;
  
  if (speed > 0) {
    // Map positive speeds from DEADZONE...MOTOR_MAX to 1900...2200 μs
    return map(speed, DEADZONE, MOTOR_MAX, 1900, 2200);
  } else {
    // Map negative speeds from -MOTOR_MAX...-DEADZONE to 800...1100 μs
    return map(speed, -MOTOR_MAX, -DEADZONE, 800, 1100);
  }
}

// Enhanced drive motor control using servo signals for ESC control
void controlDriveMotors(int x, int y, int y2) {
  int leftSpeed = 0;
  int rightSpeed = 0;

  // Apply drive mode mixing
  if (messageIn.stickL) {  // Tank drive mode
    leftSpeed = y;
    rightSpeed = y2;
  } else {  // Arcade drive mode
    leftSpeed = y + x;   // Add rotational component to left side
    rightSpeed = y - x;  // Subtract rotational component from right side
  }

  // Invert directions if needed
  if (!messageIn.stickR) {
    leftSpeed = -leftSpeed;
    rightSpeed = -rightSpeed;
  }

  // (Previously we applied a cubic scaling. For finer control, we now use the raw values.)
  leftSpeed = constrain(leftSpeed, -MOTOR_MAX, MOTOR_MAX);
  rightSpeed = constrain(rightSpeed, -MOTOR_MAX, MOTOR_MAX);

  Serial.printf("Inputs: x=%d, y=%d, y2=%d => leftSpeed=%d, rightSpeed=%d\n", x, y, y2, leftSpeed, rightSpeed);

  // Convert speeds to ESC pulses
  int leftPulse = speedToPulse(leftSpeed);
  int rightPulse = speedToPulse(rightSpeed);
  
  leftESC.writeMicroseconds(leftPulse);
  rightESC.writeMicroseconds(rightPulse);
}

// Shutdown procedure: stop weapon and set drive ESCs to neutral
void shutdownSystems() {
  weaponESC.writeMicroseconds(1000);
  leftESC.writeMicroseconds(1500);
  rightESC.writeMicroseconds(1500);
  digitalWrite(BUILTIN_LED, HIGH);
}

void loop() {
  // Timeout check: if no message for over TIMEOUT_DURATION, shutdown systems.
  if (millis() - lastMessageTime > TIMEOUT_DURATION) {
    shutdownSystems();
    return;
  }
  
  if (newMessage) {
    newMessage = false;
    
    // Update weapon state and control systems
    controlWeapon();
    controlDriveMotors(messageIn.x1, messageIn.y1, messageIn.y2);
  }
}
