#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// Pin Definitions
// ============================================================================
#define PIN_SPEED       2     // Hall effect sensor (interrupt)
#define PIN_BRAKE       3     // Brake signal (active HIGH via optocoupler)
#define PIN_CLUTCH      4     // Clutch signal (active HIGH)
#define PIN_KILL        5     // Kill switch (active HIGH)
#define PIN_SET_BTN     7     // SET button (pulldown, active HIGH)
#define PIN_SERVO       9     // Servo output

// ============================================================================
// OLED Setup (128x64, I2C address 0x3C)
// ============================================================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================================
// Servo
// ============================================================================
Servo throttleServo;

// ============================================================================
// Speed Measurement Constants 
// ============================================================================
const float WHEEL_CIRCUMFERENCE_M = 1.57;
const float SPEED_CONSTANT = WHEEL_CIRCUMFERENCE_M * 3.6;  // = 5.652
// speed (km/h) = pulses_per_sec * SPEED_CONSTANT

// ============================================================================
// Controller Parameters 
// ============================================================================
const float V_REF_KPH = 50.0;           // setpoint (km/h)
const float Kp = 0.67;                  // proportional gain
const float Ki = 0.48;                  // integral gain (s^-1)
const float Ts = 0.1;                   // sampling period (100 ms)
const float DEADBAND_KPH = 2.0;         // dead-band threshold (km/h)

const int SERVO_REST = 90;
const int SERVO_MAX = 180;

// ============================================================================
// Moving Average Filter
// ============================================================================
#define FILTER_WINDOW_SIZE 3
float speed_buffer[FILTER_WINDOW_SIZE] = {0};
int buffer_index = 0;
bool buffer_filled = false;

// ============================================================================
// Global Variables
// ============================================================================
volatile unsigned long last_pulse_time = 0;
volatile float instantaneous_speed = 0.0;
volatile bool new_speed_ready = false;
volatile unsigned long pulse_interval = 0;  // store interval for safety

float filtered_speed = 0.0;
float u_current = 0.36;      
float u_prev = 0.36;
float error_prev = 0.0;
bool cruise_active = false;
unsigned long last_control_time = 0;
unsigned long last_display_update = 0;
unsigned long last_set_time = 0;

// Debounce timers
unsigned long last_brake_time = 0;
unsigned long last_clutch_time = 0;
unsigned long last_kill_time = 0;
const unsigned long DEBOUNCE_MS = 50;

// ============================================================================
// Interrupt Service Routine - Hall Effect Sensor
// ============================================================================
void onSpeedPulse() {
  unsigned long now = micros();
  unsigned long dt = now - last_pulse_time;
  last_pulse_time = now;
  
  // Only update if interval is reasonable (between 1ms and 1 second)
  if (dt > 1000 && dt < 1000000) {
    // Store interval for speed calculation in main loop
    pulse_interval = dt;
    new_speed_ready = true;
  }
}

// ============================================================================
// Helper: Map throttle command (0-1) to servo angle (90-180)
// ============================================================================
int throttleToServoAngle(float cmd) {
  // cmd: 0 = rest position (servo 90°), 1 = full throttle (servo 180°)
  int angle = SERVO_REST + (int)(cmd * (SERVO_MAX - SERVO_REST));
  return constrain(angle, SERVO_REST, SERVO_MAX);
}

// ============================================================================
// Helper: Apply throttle to servo
// ============================================================================
void setThrottle(float cmd) {
  int angle = throttleToServoAngle(cmd);
  throttleServo.write(angle);
}

// ============================================================================
// Helper: Disengage cruise (return to rest position)
// ============================================================================
void disengageCruise(const char* reason) {
  if (cruise_active) {
    cruise_active = false;
    setThrottle(0.0);           // servo to rest position
    u_current = 0.0;
    u_prev = 0.0;
    error_prev = 0.0;
    
    // Update OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("CRUISE DISENGAGED");
    display.setCursor(0, 10);
    display.println(reason);
    display.display();
    delay(500);
    
    Serial.print("DISENGAGED: ");
    Serial.println(reason);
  }
}

// ============================================================================
// Helper: Display status on OLED
// ============================================================================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.print("Speed: ");
  display.print(filtered_speed, 1);
  display.println(" km/h");
  
  display.print("Set: ");
  display.print(V_REF_KPH, 0);
  display.println(" km/h");
  
  display.print("Cruise: ");
  display.println(cruise_active ? "ON" : "OFF");
  
  if (cruise_active) {
    display.print("Throttle: ");
    display.print((int)(u_current * 100));
    display.println("%");
  }
  
  display.display();
}

// ============================================================================
// Moving Average Filter
// ============================================================================
float movingAverageFilter(float new_sample) {
  // Filter out unrealistic values
  if (new_sample < 0 || new_sample > 120) {
    return filtered_speed;  // return last valid value
  }
  
  speed_buffer[buffer_index] = new_sample;
  buffer_index = (buffer_index + 1) % FILTER_WINDOW_SIZE;
  
  if (buffer_index == 0) {
    buffer_filled = true;
  }
  
  int num_samples = buffer_filled ? FILTER_WINDOW_SIZE : buffer_index;
  if (num_samples == 0) num_samples = 1;
  
  float sum = 0;
  for (int i = 0; i < num_samples; i++) {
    sum += speed_buffer[i];
  }
  return sum / num_samples;
}

// ============================================================================
// Calculate speed from pulse interval
// ============================================================================
float calculateSpeed() {
  if (pulse_interval > 0) {
    float pulses_per_sec = 1000000.0 / (float)pulse_interval;
    float speed = pulses_per_sec * SPEED_CONSTANT;
    // Sanity check
    if (speed > 0 && speed < 120) {
      return speed;
    }
  }
  return 0.0;
}

// ============================================================================
// Discrete-Time Velocity-Form PI Controller 
// ============================================================================
float piController(float v_meas_kph, float v_ref_kph) {
  float error = v_ref_kph - v_meas_kph;
  
  // Dead-band: no control action if error is within threshold
  if (abs(error) <= DEADBAND_KPH) {
    return u_prev;
  }
  
  // Velocity-form PI update 
  float delta_u = Kp * (error - error_prev) + Ki * Ts * error;
  float u_new = u_prev + delta_u;
  
  // Clamp throttle command to [0, 1]
  if (u_new > 1.0) u_new = 1.0;
  if (u_new < 0.0) u_new = 0.0;
  
  // Update states for next iteration
  error_prev = error;
  u_prev = u_new;
  
  return u_new;
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Cruise Control System Initializing...");
  
  // ------------------------- Pin Modes -------------------------
  pinMode(PIN_SPEED, INPUT_PULLUP);
  pinMode(PIN_BRAKE, INPUT);
  pinMode(PIN_CLUTCH, INPUT);
  pinMode(PIN_KILL, INPUT);
  pinMode(PIN_SET_BTN, INPUT);
  
  // ------------------------- Servo -------------------------
  throttleServo.attach(PIN_SERVO);
  setThrottle(0.0);     // start at rest position
  delay(500);
  
  // ------------------------- OLED -------------------------
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Cruise Control");
    display.println("System Ready");
    display.display();
  }
  
  // ------------------------- Interrupt -------------------------
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), onSpeedPulse, RISING);
  
  // Initialize timing
  last_control_time = millis();
  last_display_update = millis();
  
  Serial.println("System Ready. Press SET to engage at 50 km/h");
  Serial.println("Speed (km/h), Throttle, Cruise State");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  unsigned long now = millis();
  
  // ------------------------- Read Speed from ISR -------------------------
  if (new_speed_ready) {
    new_speed_ready = false;
    float raw_speed = calculateSpeed();
    
    // Apply moving average filter
    filtered_speed = movingAverageFilter(raw_speed);
  }
  
  // ------------------------- Safety Disengagement -------------------------
  // Brake check (active HIGH)
  if (digitalRead(PIN_BRAKE) == HIGH) {
    if (now - last_brake_time > DEBOUNCE_MS) {
      last_brake_time = now;
      if (cruise_active) {
        disengageCruise("BRAKE APPLIED");
      }
    }
  }
  
  // Clutch check (active HIGH)
  if (digitalRead(PIN_CLUTCH) == HIGH) {
    if (now - last_clutch_time > DEBOUNCE_MS) {
      last_clutch_time = now;
      if (cruise_active) {
        disengageCruise("CLUTCH ENGAGED");
      }
    }
  }
  
  // Kill switch check (active HIGH)
  if (digitalRead(PIN_KILL) == HIGH) {
    if (now - last_kill_time > DEBOUNCE_MS) {
      last_kill_time = now;
      if (cruise_active) {
        disengageCruise("KILL SWITCH ACTIVATED");
      }
    }
  }
  
  // ------------------------- SET Button -------------------------
  if (digitalRead(PIN_SET_BTN) == HIGH) {
    if (now - last_set_time > 500) {  // 500ms debounce
      last_set_time = now;
      
      if (!cruise_active) {
        // Engagement condition: speed >= 40 km/h 
        if (filtered_speed >= 40.0) {
          cruise_active = true;
          
          // Bumpless transfer: initialize controller states with current throttle
          error_prev = V_REF_KPH - filtered_speed;
          u_prev = u_current;
          
          Serial.print("CRUISE ENGAGED at ");
          Serial.print(filtered_speed);
          Serial.println(" km/h");
          
          display.clearDisplay();
          display.setTextSize(1);
          display.setCursor(0, 0);
          display.println("CRUISE ACTIVE");
          display.setCursor(0, 10);
          display.print("Setpoint: ");
          display.print(V_REF_KPH, 0);
          display.println(" km/h");
          display.display();
          delay(1000);
        } else {
          // Speed too low for cruise engagement
          display.clearDisplay();
          display.setTextSize(1);
          display.setCursor(0, 0);
          display.print("Speed too low!");
          display.setCursor(0, 10);
          display.print("Need >40 km/h");
          display.display();
          delay(1000);
        }
      } else {
        // Manual disengagement via SET button (kill switch alternative)
        disengageCruise("MANUAL DISENGAGEMENT");
      }
    }
  }
  
  // ------------------------- Control Loop (every Ts = 100 ms) -------------------------
  if ((now - last_control_time) >= (Ts * 1000)) {
    last_control_time = now;
    
    if (cruise_active) {
      // Compute new throttle command using PI controller
      u_current = piController(filtered_speed, V_REF_KPH);
      
      // Apply throttle to servo
      setThrottle(u_current);
      
      // Log data for Serial monitoring
      Serial.print(filtered_speed);
      Serial.print(",");
      Serial.print(u_current);
      Serial.print(",");
      Serial.println(cruise_active ? "1" : "0");
    }
  }
  
  // ------------------------- Update Display (every 200 ms) -------------------------
  if ((now - last_display_update) >= 200) {
    last_display_update = now;
    updateDisplay();
  }
  
  // Small delay to prevent excessive CPU usage
  delay(5);
}