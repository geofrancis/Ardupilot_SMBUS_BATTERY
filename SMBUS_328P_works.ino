#include <Wire.h>

#define specification_info 0x0010 // SBS 1.1, version 1 (no PEC)
#define serial_number 0x1234

byte register_to_call = 0;

int16_t current = -1500; // mA (negative = discharge)
uint16_t remaining_capacity = 4500;  // mAh
uint16_t temperature = 2982; // 298.2 Kelvin
uint16_t overall_voltage = 26955; // 7S pack voltage (26.955V)

// 7 Cell voltages (in mV)
uint16_t cell_voltage[7] = {3850, 3855, 3848, 3852, 3849, 3851, 3850}; 

uint16_t full_charge_capacity = 5000; // mAh
uint16_t cycle_count = 12; 

void requestEvent(void){
  switch (register_to_call) {
    case 0x0A: // current
      Wire.write(lowByte(current));
      Wire.write(highByte(current));
      break;
    case 0x0F: // remaining capacity
      Wire.write(lowByte(remaining_capacity));
      Wire.write(highByte(remaining_capacity));
      break;
    case 0x08: // temperature
      Wire.write(lowByte(temperature));
      Wire.write(highByte(temperature));
      break;
    case 0x09: // overall voltage
      Wire.write(lowByte(overall_voltage));
      Wire.write(highByte(overall_voltage));
      break;
      
    // --- Cells 1 through 7 ---
    case 0x3F: Wire.write(lowByte(cell_voltage[0])); Wire.write(highByte(cell_voltage[0])); break;
    case 0x3E: Wire.write(lowByte(cell_voltage[1])); Wire.write(highByte(cell_voltage[1])); break;
    case 0x3D: Wire.write(lowByte(cell_voltage[2])); Wire.write(highByte(cell_voltage[2])); break;
    case 0x3C: Wire.write(lowByte(cell_voltage[3])); Wire.write(highByte(cell_voltage[3])); break;
    case 0x3B: Wire.write(lowByte(cell_voltage[4])); Wire.write(highByte(cell_voltage[4])); break;
    case 0x3A: Wire.write(lowByte(cell_voltage[5])); Wire.write(highByte(cell_voltage[5])); break;
    case 0x39: Wire.write(lowByte(cell_voltage[6])); Wire.write(highByte(cell_voltage[6])); break;
      
    case 0x38: // THE STOP FLAG (Tells ArduPilot there are only 7 cells)
      Wire.write(0);
      Wire.write(0);
      break;

    // --- Housekeeping ---
    case 0x1A: Wire.write(lowByte(specification_info)); Wire.write(highByte(specification_info)); break;
    case 0x10: Wire.write(lowByte(full_charge_capacity)); Wire.write(highByte(full_charge_capacity)); break;
    case 0x1C: Wire.write(lowByte(serial_number)); Wire.write(highByte(serial_number)); break;
    case 0x17: Wire.write(lowByte(cycle_count)); Wire.write(highByte(cycle_count)); break;
    
    default: // If ArduPilot asks for ANYTHING else, send 0 (stops the 65.28V garbage)
      Wire.write(0);
      Wire.write(0);
      break;
  }
}

void receiveEvent(int howMany){
  while (Wire.available()){
    register_to_call = Wire.read();
  }
}

void setup() {
  Wire.begin(0x0B);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  // No changes needed here
}