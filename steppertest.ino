#include <math.h>
#include <AccelStepper.h>

#define LDR_PIN A2
#define ENABLE_PIN 7
#define PROCEED_PIN 6
#define STEPS_TO_DEGREES 360/16384

int ldr_val = 0; // Directly proportional to voltage (5V), range is [0, 1023]
int max_ldr_val = 0; // Directly proportional to voltage (5V), range is [0, 1023]
int pos = 0; // In stepper steps
int max_pos = 0; // In stepper steps
double max_degrees; // In degrees

int time = 0;
// AccelStepper stepper (AccelStepper::HALF4WIRE, 8, 10, 9, 11, true);
AccelStepper stepper (AccelStepper::HALF4WIRE, 8, 9, 10, 11, true);
 
void setup(){
  Serial.begin(9600);
  stepper.setMaxSpeed(600);
  stepper.setAcceleration(200);
}
 
void loop(){
  ldr_val = 0;
  max_ldr_val = 0;
  pos = 0;
  max_pos = 0;

  stepper.setCurrentPosition(0);

  while (true){ // Wait until ENABLE_PIN is set to high (button is pressed) to start the program
    if (digitalRead(ENABLE_PIN)){
      break;
    }
  }
  
  stepper.moveTo(8192); // Instruct the stepper motor to move 4 rotations, polarising film moves 1 full rotation

  while (stepper.isRunning()){ // Checks if the stepper is at the target position: If no, run the codeblock below to find maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
    ldr_val = analogRead(LDR_PIN);
    pos = stepper.currentPosition();

    // // The following lines are commented out because they make the program run too slowly. Uncomment them if you need to read ALL the LDR readings as the polarising film rotates.
    // Serial.print("Read: ");
    // Serial.print(ldr_val);
    // Serial.print(" | Pos: ");
    // Serial.println(pos);

    if (ldr_val > max_ldr_val){ // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
      max_ldr_val = ldr_val;
      max_pos = pos;

      Serial.print("NEW MAX: "); // Prints the recorded maximum and the position at which it was recorded
      Serial.print(max_ldr_val);
      Serial.print(" | ");
      Serial.println(pos);
    }

    stepper.run(); // This function needs to be called as frequently as possible to update the stepper motor and make it move
  }

  stepper.moveTo(max_pos); // Stepper motor is instructed to move to the position at which the maximum reading was recorded, this is basically calibrating the polarising film to the blank cuvette

  while (stepper.isRunning()){ // Checks if the stepper is at the target position: If no, call the .run() function so the stepper motor moves to the target position | If yes, while loop breaks and code moves on to the next segment
    stepper.run();
  }

  Serial.println("Calibrated to blank cuvette");

  Serial.print("max_pos: ");
  Serial.println(max_pos);
  Serial.print("max_ldr_val: ");
  Serial.println(max_ldr_val);

  // Insert cuvette and test substance

  while (true){
    stepper.setCurrentPosition(0); // 'Tares' the stepper motor so now the 0 position is at the position of the lowest LDR reading

    while (true){ // Wait until PROCEED_PIN is set to high (button is pressed) to start the program
      if (digitalRead(PROCEED_PIN)){
        break;
      }
    }
  
  
    stepper.moveTo(8192); // Instruct the stepper motor to move 4 rotations, polarising film moves 1 full rotation
  
    max_ldr_val = 0; // Reset the maximum recorded LDR value
  
    while (stepper.isRunning()){ // Checks if the stepper is at the target position: If no, run the codeblock below to find the maximum value and maximum value position | If yes, while loop breaks and code moves on to the next segment
      ldr_val = analogRead(LDR_PIN);
      pos = stepper.currentPosition();
  
      // Serial.print("Read: ");
      // Serial.print(ldr_val);
      // Serial.print(" | Pos: ");
      // Serial.println(pos);
  
      // Serial.println(analogRead(LDR_PIN));
  
      if (ldr_val > max_ldr_val){ // Checks if the current LDR reading is higher than the recorded maximum value: If yes, updates the maximum LDR value to the current reading and updates the position at which the maximum was recorded
        max_ldr_val = ldr_val;
        max_pos = pos; // The maximum value position recorded will be relative to the maximum value position recorded with the blank cuvette, so no extra calculation is needed to find the change in rotation.
  
        Serial.print("NEW MAX: ");
        Serial.print(max_ldr_val);
        Serial.print(" | ");
        Serial.println(pos);
      }
  
      stepper.run();
    }
  
    max_pos %= 8192; // Limit the domain

    Serial.print("Rotation in ticks: ");
    Serial.print(max_pos);
    Serial.println(" ticks");
  
    max_degrees = max_pos * STEPS_TO_DEGREES; // Convert ticks to degrees
    if (max_degrees > 90){ // If the measured rotation is greater than 180, it is recorded as a rotation in the negative direction instead
      max_degrees -= 180;
    }
  
    Serial.print("Rotation in degrees: ");
    Serial.print(max_degrees);
    Serial.println(" degrees");
   
    Serial.print("Maximum reading: ");
    Serial.println(max_ldr_val);
  }
}