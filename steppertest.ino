#include <math.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#define DEBUG

#define LDR_PIN A0
#define SERVO_PIN 11
#define CALIBRATE_PIN A1
#define TEST_PIN A2
#define CYCLE_PIN A3

#define STEPS_TO_DEGREES 360 / 16384  // 16384 steps in 1 revolution (4096 steps per revolution of the stepper motor, 1:4 gear ratio)

#define LCD_COLS 16
#define LCD_ROWS 2

uint8_t degrees_symbol[8] = {
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
};

struct laser_diode {
  int id;
  int wavelength;
  String wavelength_string;
  int pos;
  int pin;
};

laser_diode laser_diode_0 = {0, 450, "450nm", 2, 6};
laser_diode laser_diode_1 = {1, 520, "520nm", 45, 7};
laser_diode laser_diode_2 = {2, 635, "635nm", 89, 8};
laser_diode laser_diode_3 = {3, 660, "660nm", 134, 9};
laser_diode laser_diode_4 = {4, 670, "670nm", 176, 10};
laser_diode selected_diode;

int time = 0;
AccelStepper stepper(AccelStepper::HALF4WIRE, 5, 3, 4, 2, false);
Servo servo;
hd44780_I2Cexp lcd;
int min_pos;         // In ticks
double max_degrees;  // In degrees

int calibrate(laser_diode diode) {
  #ifdef DEBUG
    Serial.print("calibration | ");
    Serial.println(diode.id);
  #endif

  int ldr_val = 0;  // Directly proportional to voltage (5V), range is [0, 1023]
  int prev_ldr_val = 0;
  int min_ldr_val = INFINITY;  // Directly proportional to voltage (5V), range is [0, 1023]
  int pos = 0;                 // In stepper steps
  int min_pos = 0;             // In stepper steps
  int min_min_pos = 0;
  int max_min_pos = 0;
  int overflow_max_min_pos = 0;

  servo.write(diode.pos);  // Move to correct laser diode
  stepper.enableOutputs();

  delay(800);

  stepper.setCurrentPosition(0);

  stepper.moveTo(8192);  // Instruct the stepper motor to move 2 rotations, polarising film moves half a rotation

  while (true) {    // Checks if the stepper is at the target position: If no, run the codeblock below to find maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
    stepper.run();  // This function needs to be called as frequently as possible to update the stepper motor and make it move

    prev_ldr_val = ldr_val;
    ldr_val = analogRead(LDR_PIN);
    pos = stepper.currentPosition();

    #ifdef DEBUG  // Serial debug outputs will slow down the code if they run continuously
      if (!digitalRead(CALIBRATE_PIN)) {
        Serial.println(ldr_val);
        Serial.println(pos);
      }
    #endif

    if (ldr_val < min_ldr_val) {  // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
      min_ldr_val = ldr_val;
      min_min_pos = max_min_pos = pos;

      #ifdef DEBUG
        Serial.print("NEW MIN: ");  // Prints the recorded maximum and the position at which it was recorded
        Serial.print(min_ldr_val);
        Serial.print(" | ");
        Serial.println(pos);
      #endif
    } else if (ldr_val == min_ldr_val) {
      if (ldr_val > prev_ldr_val) {  // Updates minimum position at which the minimum transmission is recorded only if the previous reading was greater than the current reading
        min_min_pos = pos;
      }

      max_min_pos = pos;                                     // Updates maximum position at which the minimum transmission is recorded
    } else if (ldr_val < min_ldr_val && min_min_pos == 0) {  // If minimum transmission range starts at 0, update overflow_max_min_pos in case the minimum transmission range overflows
      overflow_max_min_pos = max_min_pos;
    }

    if (!stepper.isRunning()) {
      break;
    }
  }

  stepper.setCurrentPosition(0);

  if (max_min_pos == 8192 && overflow_max_min_pos > 0) {  // Minmum transmission range 'overflows' from 8192 back to 0
    max_min_pos = overflow_max_min_pos;                   // Set maximum position of minimum transmission range to previously recorded value
    min_min_pos -= 8192;                                  // Minimum position of minimum transmission will be negative to account for overflow
  }

  min_pos = (double)min_min_pos / 2 + (double)max_min_pos / 2;  // Find the middle of the minimum transmission range

  stepper.moveTo(min_pos);  // Stepper motor is instructed to move to the position at which the maximum reading was recorded, this is basically calibrating the polarising film to the blank cuvette

  while (stepper.isRunning()) {  // Checks if the stepper is at the target position: If no, call the .run() function so the stepper motor moves to the target position | If yes, while loop breaks and code moves on to the next segment
    stepper.run();
  }

  delay(50);
  stepper.disableOutputs(); // Stop powering the stepper motor to save power and prevent overheating

  #ifdef DEBUG
    Serial.println("Calibrated to blank cuvette");
    Serial.print("min_pos: ");
    Serial.println(min_pos);
    Serial.print("min_min_pos: ");
    Serial.println(min_min_pos);
    Serial.print("max_min_pos: ");
    Serial.println(max_min_pos);
    Serial.print("min_ldr_val: ");
    Serial.println(min_ldr_val);
  #endif

  return min_pos;
}

int test(laser_diode diode) {
  #ifdef DEBUG
    Serial.print("test | ");
    Serial.println(diode.id);
  #endif

  int ldr_val = 0;  // Directly proportional to voltage (5V), range is [0, 1023]
  int prev_ldr_val = 0;
  int min_ldr_val = INFINITY;  // Directly proportional to voltage (5V), range is [0, 1023]
  int pos = 0;                 // In stepper steps
  int min_pos = 0;             // In stepper steps
  int min_min_pos = 0;
  int max_min_pos = 0;
  int overflow_max_min_pos = 0;

  servo.write(diode.pos);  // Move to correct laser diode
  stepper.enableOutputs();

  delay(800);

  stepper.setCurrentPosition(0);  // 'Tares' the stepper motor so now the 0 position is at the position of the lowest LDR reading

  stepper.moveTo(8192);  // Instruct the stepper motor to move 2 rotations, polarising film moves half a rotation

  while (true) {    // Checks if the stepper is at the target position: If no, run the codeblock below to find maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
    stepper.run();  // This function needs to be called as frequently as possible to update the stepper motor and make it move

    prev_ldr_val = ldr_val;
    ldr_val = analogRead(LDR_PIN);
    pos = stepper.currentPosition();

    if (ldr_val < min_ldr_val) {  // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
      min_ldr_val = ldr_val;
      min_min_pos = max_min_pos = pos;

      #ifdef DEBUG
        Serial.print("NEW MIN: ");  // Prints the recorded maximum and the position at which it was recorded
        Serial.print(min_ldr_val);
        Serial.print(" | ");
        Serial.println(pos);
      #endif
    } else if (ldr_val == min_ldr_val) {
      if (ldr_val > prev_ldr_val) {  // Updates minimum position at which the minimum transmission is recorded only if the previous reading was greater than the current reading
        min_min_pos = pos;
      }

      max_min_pos = pos;                                     // Updates maximum position at which the minimum transmission is recorded
    } else if (ldr_val < min_ldr_val && min_min_pos == 0) {  // If minimum transmission range starts at 0, update overflow_max_min_pos in case the minimum transmission range overflows
      overflow_max_min_pos = max_min_pos;
    }

    if (!stepper.isRunning()) {
      break;
    }
  }

  if (max_min_pos == 8192 && overflow_max_min_pos > 0) {  // Minmum transmission range 'overflows' from 8192 back to 0
    max_min_pos = overflow_max_min_pos;                   // Set maximum position of minimum transmission range to previously recorded value
    min_min_pos -= 8192;                                  // Minimum position of minimum transmission will be negative to account for overflow
  }

  min_pos = (double)min_min_pos / 2 + (double)max_min_pos / 2;  // Find the middle of the minimum transmission range

  min_pos %= 8192;  // Limit the domain in case of overflow

  delay(50);
  stepper.disableOutputs(); // Stop powering the stepper motor to save power and prevent overheating

  #ifdef DEBUG
    Serial.print("Rotation in ticks: ");
    Serial.println(min_pos);
    Serial.print("Maximum LDR value: ");
    Serial.println(min_ldr_val);
    Serial.print("min_pos: ");
    Serial.println(min_pos);
    Serial.print("min_min_pos: ");
    Serial.println(min_min_pos);
    Serial.print("max_min_pos: ");
    Serial.println(max_min_pos);
  #endif

  return min_pos;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(CALIBRATE_PIN, INPUT_PULLUP);
  pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(CYCLE_PIN, INPUT_PULLUP);
  servo.attach(SERVO_PIN, 520, 2350);  // Set up servo PWM with manufacturer recommended limits

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  stepper.setMaxSpeed(600);  // Define motion profile of the stepper motor
  stepper.setAcceleration(200);

  lcd.begin(LCD_COLS, LCD_ROWS);  // Initialise LCD display
  lcd.createChar(0, degrees_symbol);

  selected_diode = laser_diode_0;
  servo.write(selected_diode.pos);

  lcd.setCursor(0, 0);
  lcd.print("Laser 0:");
  lcd.setCursor(0, 1);
  lcd.print(selected_diode.wavelength_string);
}

void loop() {
  bool cycle_latch = false;

  while (true) {
    if (!digitalRead(CYCLE_PIN)) {  // Cycles through laser diodes
      if (!cycle_latch) {
        lcd.clear();

        switch (selected_diode.id) {
          case 0:
            selected_diode = laser_diode_0;
            digitalWrite(laser_diode_4.pin, LOW);
            digitalWrite(laser_diode_0.pin, HIGH);
            lcd.setCursor(0, 0);
            lcd.print("Laser 0:");
            break;

          case 1:
            selected_diode = laser_diode_1;
            digitalWrite(laser_diode_0.pin, LOW);
            digitalWrite(laser_diode_1.pin, HIGH);
            lcd.setCursor(0, 0);
            lcd.print("Laser 1:");
            break;

          case 2:
            selected_diode = laser_diode_2;
            digitalWrite(laser_diode_1.pin, LOW);
            digitalWrite(laser_diode_2.pin, HIGH);
            lcd.setCursor(0, 0);
            lcd.print("Laser 2:");
            break;

          case 3:
            selected_diode = laser_diode_3;
            digitalWrite(laser_diode_2.pin, LOW);
            digitalWrite(laser_diode_3.pin, HIGH);
            lcd.setCursor(0, 0);
            lcd.print("Laser 3:");
            break;

          case 4:
            selected_diode = laser_diode_4;
            digitalWrite(laser_diode_3.pin, LOW);
            digitalWrite(laser_diode_4.pin, HIGH);
            lcd.setCursor(0, 0);
            lcd.print("Laser 4:");
            break;
        }

        lcd.setCursor(0, 1);
        lcd.print(selected_diode.wavelength_string);
      }
      cycle_latch = true;
    } else {
      cycle_latch = false;
    }

    if (!digitalRead(CALIBRATE_PIN)) {  // Run calibration
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrating...");

      calibrate(selected_diode);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrated");
    } else if (!digitalRead(TEST_PIN)) {  // Run testing
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Testing...");

      min_pos = test(selected_diode);
      max_degrees = (double)min_pos * STEPS_TO_DEGREES;  // Convert ticks to degrees

      if (max_degrees > 90) {  // If the measured rotation is greater than 180, it is recorded as a rotation in the negative direction instead
        max_degrees -= 180;
      }

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write((max_degrees > 0) ? "+" : "-");

      lcd.setCursor(1, 0);
      lcd.print(String(fabs(max_degrees), 14));

      lcd.setCursor(15, 0);
      lcd.write(0);

      lcd.setCursor(0, 1);
      lcd.print(min_pos);
    }
  }
}
