#include <Wire.h>

// --- RP2040 PIN DEFINITIONS (Standard Pico I2C0) ---
#define CUSTOM_SDA      4  // Physical Pin 6
#define CUSTOM_SCL      5  // Physical Pin 7

#define specification_info 0x0010 // SBS 1.1, version 1 (no PEC)
#define serial_number 0x1234

byte register_to_call = 0;

int16_t current = -1500; // mA (negative = discharge)
uint16_t remaining_capacity = 4500;  // mAh
uint16_t temperature = 2982; // 298.2 Kelvin
uint16_t overall_voltage = 26955; // 7S pack voltage

// 7 Cell voltages (in mV) - Starting values
uint16_t cell_voltage[7] = {3850, 3855, 3848, 3852, 3849, 3851, 3850}; 

uint16_t full_charge_capacity = 5000; // mAh
uint16_t cycle_count = 12; 

/* ---------- Diagnostic Variables ---------- */
volatile bool     log_rx_flag = false;
volatile uint8_t  log_rx_cmd  = 0;
volatile bool     log_tx_flag = false;
volatile int16_t  log_tx_val  = 0; 

void requestEvent(void){
  switch (register_to_call) {
    case 0x0A: // current
      Wire.write(lowByte(current));
      Wire.write(highByte(current));
      log_tx_val = current;
      break;
    case 0x0F: // remaining capacity
      Wire.write(lowByte(remaining_capacity));
      Wire.write(highByte(remaining_capacity));
      log_tx_val = remaining_capacity;
      break;
    case 0x08: // temperature
      Wire.write(lowByte(temperature));
      Wire.write(highByte(temperature));
      log_tx_val = temperature;
      break;
    case 0x09: // overall voltage
      Wire.write(lowByte(overall_voltage));
      Wire.write(highByte(overall_voltage));
      log_tx_val = overall_voltage;
      break;
      
    // --- Cells 1 through 7 ---
    case 0x3F: Wire.write(lowByte(cell_voltage[0])); Wire.write(highByte(cell_voltage[0])); log_tx_val = cell_voltage[0]; break;
    case 0x3E: Wire.write(lowByte(cell_voltage[1])); Wire.write(highByte(cell_voltage[1])); log_tx_val = cell_voltage[1]; break;
    case 0x3D: Wire.write(lowByte(cell_voltage[2])); Wire.write(highByte(cell_voltage[2])); log_tx_val = cell_voltage[2]; break;
    case 0x3C: Wire.write(lowByte(cell_voltage[3])); Wire.write(highByte(cell_voltage[3])); log_tx_val = cell_voltage[3]; break;
    case 0x3B: Wire.write(lowByte(cell_voltage[4])); Wire.write(highByte(cell_voltage[4])); log_tx_val = cell_voltage[4]; break;
    case 0x3A: Wire.write(lowByte(cell_voltage[5])); Wire.write(highByte(cell_voltage[5])); log_tx_val = cell_voltage[5]; break;
    case 0x39: Wire.write(lowByte(cell_voltage[6])); Wire.write(highByte(cell_voltage[6])); log_tx_val = cell_voltage[6]; break;
      
    case 0x38: // THE STOP FLAG
      Wire.write(0);
      Wire.write(0);
      log_tx_val = 0;
      break;

    // --- Housekeeping ---
    case 0x1A: Wire.write(lowByte(specification_info)); Wire.write(highByte(specification_info)); log_tx_val = specification_info; break;
    case 0x10: Wire.write(lowByte(full_charge_capacity)); Wire.write(highByte(full_charge_capacity)); log_tx_val = full_charge_capacity; break;
    case 0x1C: Wire.write(lowByte(serial_number)); Wire.write(highByte(serial_number)); log_tx_val = serial_number; break;
    case 0x17: Wire.write(lowByte(cycle_count)); Wire.write(highByte(cycle_count)); log_tx_val = cycle_count; break;
    
    default: // If ArduPilot asks for ANYTHING else, send 0
      Wire.write(0);
      Wire.write(0);
      log_tx_val = 0;
      break;
  }
  
  log_tx_flag = true; // Flag that we sent a response
}

void receiveEvent(int howMany){
  while (Wire.available()){
    register_to_call = Wire.read();
  }
  log_rx_cmd = register_to_call;
  log_rx_flag = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Seed the random number generator using an unconnected analog pin
  randomSeed(analogRead(26)); 

  Wire.setSDA(CUSTOM_SDA);
  Wire.setSCL(CUSTOM_SCL);
  
  Wire.begin(0x0B);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("\n=== RP2040 SMBus Diagnostics (Dynamic) ===");
  Serial.println("Addr 0x0B | SDA Pin 6 | SCL Pin 7");
}

void loop() {
  // 1. Blink LED
  static uint32_t last_blink = 0;
  if (millis() - last_blink > 500) {
    last_blink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 2. Update battery values randomly every 2 seconds
  static uint32_t last_update = 0;
  if (millis() - last_update > 2000) {
    last_update = millis();
    
    uint16_t new_pack_voltage = 0;
    
    // Jitter each cell by +/- 15mV around 3.85V
    for (int i = 0; i < 7; i++) {
      cell_voltage[i] = 3850 + random(-15, 15);
      new_pack_voltage += cell_voltage[i];
    }
    
    // Update the pack voltage to match the sum of the cells
    overall_voltage = new_pack_voltage;
    
    // Fluctuate current draw between -5A and -20A
    current = random(-20000, -5000);
    
    // Drain capacity slowly
    if (remaining_capacity > 5) {
      remaining_capacity -= 5;
    }
    
    Serial.printf("\n[UPDATE] Pack: %u mV | C1: %u mV | Current: %d mA\n", 
                  overall_voltage, cell_voltage[0], current);
  }

  // 3. Print I2C Diagnostics safely
  if (log_rx_flag) {
    log_rx_flag = false;
    Serial.print("[RX] Command: 0x");
    if (log_rx_cmd < 0x10) Serial.print("0");
    Serial.print(log_rx_cmd, HEX);
  }

  if (log_tx_flag) {
    log_tx_flag = false;
    Serial.print(" -> [TX] Value: ");
    Serial.print(log_tx_val);
    Serial.println();
  }
}