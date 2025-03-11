#include <stdint.h>
#include <Wire.h>

constexpr float MIN_VOLTAGE = 0.0;
constexpr float MAX_VOLTAGE = 5.0;
constexpr int MAX_TRAVEL_MM = 250;
constexpr float MAX_INT16_T_POSITIVE = 32767.0;
constexpr float MAX_INT16_T_NEGATIVE = 32768.0;

constexpr float ADS1115_6V = 6.144;
constexpr float ADS1115_250MV = 0.256;
constexpr int NUM_SAMPLES = 16; // do not change, because division logic will have to be edited

constexpr float LOAD_CELL_MV_V = 3.2;
constexpr float MAX_LOAD_CELL_WEIGHT = 500.0;

constexpr int DELAY_MS = 250;
constexpr unsigned long BAUD_RATE = 115200;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println(F("ADC Voltage, Displacement, Velocity, and Dynamic Force Logger"));
}

void loop() {
  // Read the raw ADC value
  float linear_pot_voltage = read_linear_pot();
  float rotary_pot_voltage = read_rotary_pot();
  float load_cell_voltage = read_load_cell();

  float displacement = convert_to_displacement(linear_pot_voltage);

  // Calculate velocity in mm/s
  float velocity = calculate_linear_pot_velocity();

  // Calculate dynamic force based on real-time displacement

  // Output results
  Serial.print(F("Linear_Pot_Voltage:"));
  Serial.print(linear_pot_voltage, 2);
  Serial.print(F(",   Displacement_mm:"));
  Serial.print(displacement, 2);
  Serial.print(F(",   Velocity_mm/s:"));
  Serial.print(velocity, 2);
  Serial.print(F(",    Rotary_Pot_Voltage:"));
  Serial.print(rotary_pot_voltage, 4);
  Serial.print(F(",  Load Cell Voltage:"));
  Serial.print(load_cell_voltage,6);
  Serial.print(F(",  Load Cell Weight:"));
  Serial.println(convert_to_weight(load_cell_voltage), 2);

  // Delay for the next reading
  delay(DELAY_MS);
}

// ADS1115 address (default - ADDR connected to GND)
// refer to datasheet for explanation.
constexpr uint8_t ADS1115_ADDRESS = 0b01001000;

// ADS1115 config, very specific to this project.
// Refer to datasheet for explanation.
constexpr uint16_t ADS1115_CONFIG = 0b0000111011100011;

// ADS1115 registers
constexpr uint8_t CONVERSION_REGISTER = 0b00000000;
constexpr uint8_t CONFIG_REGISTER = 0b00000001;


float last_linear_pot_voltage = 0.0;
float current_linear_pot_voltage = 0.0;

unsigned long last_linear_pot_readout_time = 0;
unsigned long current_linear_pot_readout_time = 0;

// reads from the specified ADS1115 register
int16_t read_ADS1115(uint8_t reg) {
  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(ADS1115_ADDRESS, (uint8_t) 2);

  int16_t result = 0;
  if (Wire.available() >= 2) {
    result = Wire.read() << 8;
    result |= Wire.read();
  }
  return result;
}

// writes the value to the specified ADS1115 register, one byte at a time
void write_ADS1115(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0XFF);
  Wire.endTransmission();
}

// reads the load cell voltage from the ADS1115
float read_load_cell() {
  // mask with 0s where the value should be and 1s everywhere else
  // refer to datasheet for explanation
  constexpr uint16_t mask     = ~(0b0111111000000000);
  constexpr uint16_t new_bits =   0b0000111000000000;

  // set and write the config
  uint16_t config = (ADS1115_CONFIG & mask) | new_bits;
  write_ADS1115(CONFIG_REGISTER, config);

  // give the chip time to settle and take a new reading
  wait_for_conversion();

  int16_t readouts[NUM_SAMPLES];
  for (int i = 0; i < NUM_SAMPLES; i++) {
    readouts[NUM_SAMPLES] = read_ADS1115(CONVERSION_REGISTER);
  }
  int16_t readout = take_ADS1115_samples();
  return convert_to_voltage(readout, ADS1115_250MV);
}

// reads the linear pot voltage from the ADS1115
float read_linear_pot() {
  constexpr uint16_t mask     = ~(0b0111111000000000);
  constexpr uint16_t new_bits =   0b0110000000000000;

    // set and write the config
  uint16_t config = (ADS1115_CONFIG & mask) | new_bits;
  write_ADS1115(CONFIG_REGISTER, config);

  // give the chip time to settle and take a new reading
  wait_for_conversion();

  int16_t readout = take_ADS1115_samples();
  current_linear_pot_readout_time = millis();
  current_linear_pot_voltage = convert_to_voltage(readout, ADS1115_6V);
  return current_linear_pot_voltage;
}

// reads the rotary pot voltage from the ADS1115
float read_rotary_pot() {
  constexpr uint16_t mask     = ~(0b0111111000000000);
  constexpr uint16_t new_bits =   0b0111000000000000;

    // set and write the config
  uint16_t config = (ADS1115_CONFIG & mask) | new_bits;
  write_ADS1115(CONFIG_REGISTER, config);

  // give the chip time to settle and take a new reading
  wait_for_conversion();

  int16_t readout = take_ADS1115_samples();
  return convert_to_voltage(readout, ADS1115_6V);
}

// waits for the ADS1115 to set the correct bit in the config register
void wait_for_conversion() {
  delay(5);
}

int16_t take_ADS1115_samples() {
    return read_ADS1115(CONVERSION_REGISTER);
}

// Convert the ADC value to a voltage (0V to referenceVoltage) 
float convert_to_voltage(int16_t adc_value, float reference_voltage) {
  if (adc_value >= 0) {
    return (adc_value / MAX_INT16_T_POSITIVE) * reference_voltage;
  } else {
    return (adc_value / MAX_INT16_T_NEGATIVE) * reference_voltage;
  }
}


float calculate_linear_pot_velocity() {
    // Ensure we have valid timestamps to avoid division by zero
    if (current_linear_pot_readout_time == last_linear_pot_readout_time) {
        return 0.0;  // No time difference, can't calculate velocity
    }

    // Convert voltage to displacement
    float y2 = convert_to_displacement(current_linear_pot_voltage);
    float y1 = convert_to_displacement(last_linear_pot_voltage);

    // Calculate time difference in seconds
    float deltaTime = (current_linear_pot_readout_time - last_linear_pot_readout_time) / 1000.0;

    // Calculate velocity: change in displacement over time
    float velocity = (y2 - y1) / deltaTime;  // Velocity in mm/s

    last_linear_pot_readout_time = current_linear_pot_readout_time;
    last_linear_pot_voltage = current_linear_pot_voltage;

    return velocity;
}


float convert_to_displacement(float voltage) {
  // Clamp the voltage to the valid range
  if (voltage < MIN_VOLTAGE) {
    voltage = MIN_VOLTAGE;
  } else if (voltage > MAX_VOLTAGE) {
    voltage = MAX_VOLTAGE;
  }

  return ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * MAX_TRAVEL_MM;
}

float convert_to_weight(float voltage) {
  constexpr float max_meter_voltage = (LOAD_CELL_MV_V * MAX_VOLTAGE) / 1000.0;

  return (voltage / max_meter_voltage) * MAX_LOAD_CELL_WEIGHT;
}
