#define MIN_VOLTAGE 0.0
#define MAX_VOLTAGE 5.0
#define MAX_TRAVEL 250             // Maximum travel in millimeters
#define MASS 5
#define LINEAR_POT_PIN A1
#define ROTARY_POT_PIN A0
#define LOAD_CELL_POT_PIN A2
#define REFERENCE_VOLTAGE 5
#define NUM_SAMPLES 300


int adcValues[NUM_SAMPLES];         // Array to store the last 300 ADC readings
int currentIndex = 0;              // Index to track the current sample in the array
bool arrayFilled = false; 

unsigned long lastTime = 0;         // Time of the last sample
unsigned long currentTime = 0;      // Current time

void setup() {
  Serial.begin(9600);              // Initialize serial communication at 9600 baud
  Serial.println("ADC Voltage, Displacement, Velocity, and Dynamic Force Logger");
}

void loop() {
  // Read the raw ADC value
  int lprv = analogRead(LINEAR_POT_PIN);    //linear potentiometer raw value
  int rprv = analogRead(ROTARY_POT_PIN);    //rotary potentiometer raw value
  int lcrv = analogRead(LOAD_CELL_POT_PIN);  //load cell raw value

  // Store the current ADC value in the array
  adcValues[currentIndex] = lprv;

  // Increment the index and wrap around if necessary
  currentIndex = (currentIndex + 1) % NUM_SAMPLES;

  // Set the array filled flag to true once we've wrapped around
  if (currentIndex == 0) {
    arrayFilled = true;
  }

  // Calculate the average ADC value
  int averageValue = calculateAverage();

  // Convert values
  float lpv = convertToVoltage(lprv);    //linear potentiometer voltage
  float rpv = convertToVoltage(rprv);    //rotary potentiometer voltage
  float lcv = convertToVoltage(lcrv);
  float displacement = convertToDisplacement(lprv, MIN_VOLTAGE, MAX_VOLTAGE, MAX_TRAVEL);

  // Calculate velocity in mm/s
  float velocity = calculateVelocity();

  // Calculate dynamic force based on real-time displacement

  // Output results
  Serial.print("Linear_Pot_Voltage: ");
  Serial.print(lpv, 2);
  Serial.print(",   Displacement_mm: ");
  Serial.print(displacement, 2);
  Serial.print(",   Velocity_mm/s: ");
  Serial.print(velocity, 2);
  Serial.print(",    Rotary_Pot_Voltage: ");
  Serial.print(rpv, 2);
  Serial.print(",   Load_Cell_Voltage: ");
  Serial.println(lcv , 2);

  // Delay for the next reading
  delay(100);
}

int calculateAverage() {
  long sum = 0;
  int count = arrayFilled ? NUM_SAMPLES : currentIndex;

  // Sum all stored values
  for (int i = 0; i < count; i++) {
    sum += adcValues[i];
  }

  // Return the average value as an integer
  return sum / count;
}

float calculateVelocity() {
  if (currentIndex < 2 && !arrayFilled) {
    return 0.0;  // Not enough data to calculate velocity
  }

  // Get the two most recent ADC values
  int lastIndex = (currentIndex - 1 + NUM_SAMPLES) % NUM_SAMPLES;
  int prevIndex = (currentIndex - 2 + NUM_SAMPLES) % NUM_SAMPLES;

  // Convert ADC values to displacement
  float y2 = convertToDisplacement(adcValues[lastIndex], MIN_VOLTAGE, MAX_VOLTAGE, MAX_TRAVEL);
  float y1 = convertToDisplacement(adcValues[prevIndex], MIN_VOLTAGE, MAX_VOLTAGE, MAX_TRAVEL);

  // Get time difference between the two readings
  currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;  // Time difference in milliseconds
  lastTime = currentTime;

  if (deltaTime == 0) {
    return 0.0;  // Avoid division by zero
  }

  // Calculate velocity (change in displacement over time)
  float velocity = (y2 - y1) / (deltaTime / 1000.0);  // Velocity in mm/s
  return velocity;
}


float convertToVoltage(int adcValue) {
  // Convert the ADC value to a voltage (0V to referenceVoltage)
  return (adcValue / 1023.0) * REFERENCE_VOLTAGE;
}

float convertToDisplacement(int adcValue, float minVoltage, float maxVoltage, float maxTravel) {
  float voltage = convertToVoltage(adcValue);

  // Clamp the voltage to the valid range
  if (voltage < minVoltage) {
    voltage = minVoltage;
  } else if (voltage > maxVoltage) {
    voltage = maxVoltage;
  }

  // Calculate the position percentage
  float positionPercentage = (voltage - minVoltage) / (maxVoltage - minVoltage);

  // Convert the percentage to a displacement value (in mm)
  return positionPercentage * maxTravel;
}
