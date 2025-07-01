#include <math.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#define DEBUG

#define LDR_PIN A2
#define SERVO_PIN 3
#define ENABLE_PIN 7
#define PROCEED_PIN 6

#define STEPS_TO_DEGREES 360/16384

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
  int pos;
};

laser_diode laser_diode_0 = {0, 0, 5};
laser_diode laser_diode_1 = {1, 0, 58};
laser_diode laser_diode_2 = {2, 0, 116};
laser_diode laser_diode_3 = {3, 0, 166};

int time = 0;
AccelStepper stepper (AccelStepper::HALF4WIRE, 8, 10, 9, 11, true);
Servo servo;
hd44780_I2Cexp lcd;
int min_pos; // In ticks
double max_degrees; // In degrees

int calibrate(laser_diode diode){
  #ifdef DEBUG
    Serial.print("calibration | ");
    Serial.println(diode.id);
  #endif

  int ldr_val = 0; // Directly proportional to voltage (5V), range is [0, 1023]
  int prev_ldr_val = 0;
  int min_ldr_val = INFINITY; // Directly proportional to voltage (5V), range is [0, 1023]
  int pos = 0; // In stepper steps
  int min_pos = 0; // In stepper steps
  int min_min_pos = 0;
  int max_min_pos = 0;
  int overflow_max_min_pos = 0;

  servo.write(diode.pos); // Move to correct laser diode
  delay(1000);

  stepper.setCurrentPosition(0);

  stepper.moveTo(8192); // Instruct the stepper motor to move 2 rotations, polarising film moves half a rotation

  while (true){ // Checks if the stepper is at the target position: If no, run the codeblock below to find maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
    stepper.run(); // This function needs to be called as frequently as possible to update the stepper motor and make it move
    
    prev_ldr_val = ldr_val;
    ldr_val = analogRead(LDR_PIN);
    pos = stepper.currentPosition();

    if (!digitalRead(ENABLE_PIN)){
      Serial.println(ldr_val);
      Serial.println(pos);
    }
    if (ldr_val < min_ldr_val){ // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
      // Serial.println("x");
      // Serial.println(ldr_val);
      // Serial.println(min_ldr_val);
      min_ldr_val = ldr_val;
      min_min_pos = max_min_pos = pos;

      #ifdef DEBUG
        Serial.print("NEW MIN: "); // Prints the recorded maximum and the position at which it was recorded
        Serial.print(min_ldr_val);
        Serial.print(" | ");
        Serial.println(pos);
      #endif
    } else if (ldr_val == min_ldr_val){
      if (ldr_val < prev_ldr_val){
        Serial.println(pos);
        min_min_pos = pos;
      }

      max_min_pos = pos;
    } else if (ldr_val > min_ldr_val && min_min_pos == 0){
      overflow_max_min_pos = max_min_pos;
    }

    if (!stepper.isRunning()){
      break;
    }
  }

  stepper.setCurrentPosition(0);

  if (max_min_pos == 8192){
    max_min_pos = overflow_max_min_pos;
    min_min_pos -= 8192;
  }

  min_pos = (double)min_min_pos/2 + (double)max_min_pos/2;

  stepper.moveTo(min_pos); // Stepper motor is instructed to move to the position at which the maximum reading was recorded, this is basically calibrating the polarising film to the blank cuvette

  while (stepper.isRunning()){ // Checks if the stepper is at the target position: If no, call the .run() function so the stepper motor moves to the target position | If yes, while loop breaks and code moves on to the next segment
    stepper.run();
  }

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

int test(laser_diode diode){
  #ifdef DEBUG
    Serial.print("test | ");
    Serial.println(diode.id);
  #endif

  int ldr_val = 0; // Directly proportional to voltage (5V), range is [0, 1023]
  int prev_ldr_val = 0;
  int min_ldr_val = INFINITY; // Directly proportional to voltage (5V), range is [0, 1023]
  int pos = 0; // In stepper steps
  int min_pos = 0; // In stepper steps
  int min_min_pos = 0;
  int max_min_pos = 0;
  int overflow_max_min_pos = 0;

  servo.write(diode.pos); // Move to correct laser diode
  delay(1000);

  stepper.setCurrentPosition(0); // 'Tares' the stepper motor so now the 0 position is at the position of the lowest LDR reading
  
  stepper.moveTo(8192); // Instruct the stepper motor to move 4 rotations, polarising film moves 1 full rotation

  while (true){ // Checks if the stepper is at the target position: If no, run the codeblock below to find maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
    stepper.run(); // This function needs to be called as frequently as possible to update the stepper motor and make it move

    prev_ldr_val = ldr_val;
    ldr_val = analogRead(LDR_PIN);
    pos = stepper.currentPosition();

    if (ldr_val < min_ldr_val){ // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
      min_ldr_val = ldr_val;
      min_min_pos = max_min_pos = pos;

      #ifdef DEBUG
        Serial.print("NEW MIN: "); // Prints the recorded maximum and the position at which it was recorded
        Serial.print(min_ldr_val);
        Serial.print(" | ");
        Serial.println(pos);
      #endif
    } else if (ldr_val == min_ldr_val){
      if (ldr_val < prev_ldr_val){
        min_min_pos = pos;
      }

      max_min_pos = pos;
    } else if (ldr_val > min_ldr_val && min_min_pos == 0){
      overflow_max_min_pos = max_min_pos;
    }

    if (!stepper.isRunning()){
      break;
    }
  }

  if (max_min_pos == 8192){
    max_min_pos = overflow_max_min_pos;
    min_min_pos -= 8192; 
  }

  min_pos = (double)min_min_pos/2 + (double)max_min_pos/2;

  min_pos %= 8192; // Limit the domain
  
  #ifdef DEBUG
    Serial.print("Rotation in ticks: ");
    Serial.println(min_pos);
    Serial.print("Maximum LDR value: ");
    Serial.println(min_ldr_val);
  #endif

  return min_pos;
}

void setup(){
  Wire.begin();
  Serial.begin(9600);

  pinMode(ENABLE_PIN, INPUT_PULLUP);
  pinMode(PROCEED_PIN, INPUT_PULLUP);
  servo.attach(SERVO_PIN, 520, 2350);

  stepper.setMaxSpeed(600);
  stepper.setAcceleration(200);

  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.createChar(0, degrees_symbol);

  #warning remove me
  servo.write(laser_diode_2.pos);
}
 
void loop(){
  while (true){
    if (!digitalRead(ENABLE_PIN)){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrating...");

      calibrate(laser_diode_2);
 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrated");
    } else if (!digitalRead(PROCEED_PIN)){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Testing...");

      min_pos = test(laser_diode_2);
      max_degrees = (double)min_pos * STEPS_TO_DEGREES; // Convert ticks to degrees

      if (max_degrees > 90){ // If the measured rotation is greater than 180, it is recorded as a rotation in the negative direction instead
        max_degrees -= 180;
      }

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write((max_degrees > 0)? "+" : "-");

      lcd.setCursor(1, 0);
      lcd.print(String(fabs(max_degrees), 14));

      lcd.setCursor(15, 0);
      lcd.write(0);
      
      lcd.setCursor(0, 1);
      lcd.print(min_pos);
    }
  }

  // ldr_val = 0;
  // pos = 0;
  // min_pos = 0;

  // servo.write(laser_diode_1.pos);
  // stepper.setCurrentPosition(0);

  // while (true){ // Wait until ENABLE_PIN is set to high (button is pressed) to start the program
  //   if (!digitalRead(ENABLE_PIN)){
  //     break;
  //   }
  // }
  
  // stepper.moveTo(8192); // Instruct the stepper motor to move 4 rotations, polarising film moves 1 full rotation

  // while (stepper.isRunning()){ // Checks if the stepper is at the target position: If no, run the codeblock below to find maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
  //   ldr_val = analogRead(LDR_PIN);
  //   pos = stepper.currentPosition();

  //   // // The following lines are commented out because they make the program run too slowly. Uncomment them if you need to read ALL the LDR readings as the polarising film rotates.
  //   // Serial.print("Read: ");
  //   // Serial.print(ldr_val);
  //   // Serial.print(" | Pos: ");
  //   // Serial.println(pos);

  //   // Serial.println(ldr_val);

  //   if (ldr_val > min_ldr_val){ // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
  //     min_ldr_val = ldr_val;
  //     min_pos = pos;

  //     Serial.print("NEW MIN: "); // Prints the recorded maximum and the position at which it was recorded
  //     Serial.print(min_ldr_val);
  //     Serial.print(" | ");
  //     Serial.println(pos);
  //   }

  //   stepper.run(); // This function needs to be callaser_diode as frequently as possible to update the stepper motor and make it move
  // }

  // stepper.setCurrentPosition(0);

  // stepper.moveTo(min_pos); // Stepper motor is instructed to move to the position at which the maximum reading was recorded, this is basically calibrating the polarising film to the blank cuvette

  // while (stepper.isRunning()){ // Checks if the stepper is at the target position: If no, call the .run() function so the stepper motor moves to the target position | If yes, while loop breaks and code moves on to the next segment
  //   stepper.run();
  // }

  // Serial.println("Calibrated to blank cuvette");

  // Serial.print("min_pos: ");
  // Serial.println(min_pos);
  // Serial.print("min_ldr_val: ");
  // Serial.println(min_ldr_val);

  // // Insert cuvette and test substance

  // while (true){
  //   stepper.setCurrentPosition(0); // 'Tares' the stepper motor so now the 0 position is at the position of the lowest LDR reading

  //   while (true){ // Wait until PROCEED_PIN is set to high (button is pressed) to start the program
  //     if (!digitalRead(PROCEED_PIN)){
  //       break;
  //     }
  //   }
  
  
  //   stepper.moveTo(8192); // Instruct the stepper motor to move 4 rotations, polarising film moves 1 full rotation
  
  //   min_ldr_val = 0; // Reset the maximum recorded LDR value
  
  //   while (stepper.isRunning()){ // Checks if the stepper is at the target position: If no, run the codeblock below to find the maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
  //     ldr_val = analogRead(LDR_PIN);
  //     pos = stepper.currentPosition();
  
  //     // Serial.print("Read: ");
  //     // Serial.print(ldr_val);
  //     // Serial.print(" | Pos: ");
  //     // Serial.println(pos);
  
  //     // Serial.println(analogRead(LDR_PIN));
  
  //     if (ldr_val > min_ldr_val){ // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
  //       min_ldr_val = ldr_val;
  //       min_pos = pos; // The maximum value position recorded will be relative to the maximum value position recorded with the blank cuvette, so no extra calculation is needed to find the change in rotation.
  
  //       Serial.print("NEW MIN: ");
  //       Serial.print(min_ldr_val);
  //       Serial.print(" | ");
  //       Serial.println(pos);
  //     }
  
  //     stepper.run();
  //   }
  
  //   min_pos %= 8192; // Limit the domain

  //   Serial.print("Rotation in ticks: ");
  //   Serial.print(min_pos);
  //   Serial.println(" ticks");
  
  //   max_degrees = (double)min_pos * STEPS_TO_DEGREES; // Convert ticks to degrees
  //   if (max_degrees > 90){ // If the measured rotation is greater than 180, it is recorded as a rotation in the negative direction instead
  //     max_degrees -= 180;
  //   }
  
  //   Serial.print("Rotation in degrees: ");
  //   Serial.print(max_degrees);
  //   Serial.println(" degrees");
   
  //   Serial.print("Maximum reading: ");
  //   Serial.println(min_ldr_val);
  // }