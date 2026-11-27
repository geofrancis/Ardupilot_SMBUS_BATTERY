#include <Wire.h>
#include <bms2.h> // OverkillSolarBms2 library

// --- I2C PINS (I2C0 to ArduPilot) ---
#define CUSTOM_SDA      4  // Physical Pin 6
#define CUSTOM_SCL      5  // Physical Pin 7

// --- BMS UART PINS (Direct TTL to JBD / Overkill BMS) ---
#define BMS_TX          8  // Physical Pin 11 (Pico TX -> BMS RX)
#define BMS_RX          9  // Physical Pin 12 (Pico RX -> BMS TX)

#define specification_info 0x0010 // SBS 1.1, version 1 (no PEC)
#define serial_number 0x1234

OverkillSolarBms2 bms = OverkillSolarBms2();
byte register_to_call = 0;

/* ---------- Fallback / Default Values (Used until first valid BMS read) ---------- */
uint16_t cell_voltage[7]       = {1110, 2220, 3330, 4440, 5550, 6660, 7770};
uint16_t overall_voltage       = 31080; // mV
int16_t  current               = 0;     // mA (+ charge, - discharge)
uint16_t remaining_capacity    = 5000;  // mAh
uint16_t full_charge_capacity  = 5000;  // mAh
uint16_t relative_soc          = 100;   // %
uint16_t temperature           = 2982;  // 0.1 K (298.2 K = 25.0 C)
uint16_t cycle_count           = 12;
uint16_t battery_status        = 0x0002; // Discharging

/* ---------- Diagnostic Variables ---------- */
volatile bool    log_rx_flag = false;
volatile uint8_t log_rx_cmd  = 0;
volatile bool    log_tx_flag = false;
volatile int16_t log_tx_val  = 0;

void requestEvent(void) {
  switch (register_to_call) {
    case 0x0A: Wire.write(lowByte(current)); Wire.write(highByte(current)); log_tx_val = current; break;
    case 0x0F: Wire.write(lowByte(remaining_capacity)); Wire.write(highByte(remaining_capacity)); log_tx_val = remaining_capacity; break;
    case 0x08: Wire.write(lowByte(temperature)); Wire.write(highByte(temperature)); log_tx_val = temperature; break;
    case 0x09: Wire.write(lowByte(overall_voltage)); Wire.write(highByte(overall_voltage)); log_tx_val = overall_voltage; break;
    case 0x0D: Wire.write(lowByte(relative_soc)); Wire.write(highByte(relative_soc)); log_tx_val = relative_soc; break;
    case 0x16: Wire.write(lowByte(battery_status)); Wire.write(highByte(battery_status)); log_tx_val = battery_status; break;

    // --- Cells 1 through 7 (mV) ---
    case 0x3F: Wire.write(lowByte(cell_voltage[0])); Wire.write(highByte(cell_voltage[0])); log_tx_val = cell_voltage[0]; break;
    case 0x3E: Wire.write(lowByte(cell_voltage[1])); Wire.write(highByte(cell_voltage[1])); log_tx_val = cell_voltage[1]; break;
    case 0x3D: Wire.write(lowByte(cell_voltage[2])); Wire.write(highByte(cell_voltage[2])); log_tx_val = cell_voltage[2]; break;
    case 0x3C: Wire.write(lowByte(cell_voltage[3])); Wire.write(highByte(cell_voltage[3])); log_tx_val = cell_voltage[3]; break;
    case 0x3B: Wire.write(lowByte(cell_voltage[4])); Wire.write(highByte(cell_voltage[4])); log_tx_val = cell_voltage[4]; break;
    case 0x3A: Wire.write(lowByte(cell_voltage[5])); Wire.write(highByte(cell_voltage[5])); log_tx_val = cell_voltage[5]; break;
    case 0x39: Wire.write(lowByte(cell_voltage[6])); Wire.write(highByte(cell_voltage[6])); log_tx_val = cell_voltage[6]; break;

    case 0x38: // Stop flag
      Wire.write(0); Wire.write(0); log_tx_val = 0; break;

    // --- Housekeeping ---
    case 0x1A: Wire.write(lowByte(specification_info)); Wire.write(highByte(specification_info)); log_tx_val = specification_info; break;
    case 0x10: Wire.write(lowByte(full_charge_capacity)); Wire.write(highByte(full_charge_capacity)); log_tx_val = full_charge_capacity; break;
    case 0x1C: Wire.write(lowByte(serial_number)); Wire.write(highByte(serial_number)); log_tx_val = serial_number; break;
    case 0x17: Wire.write(lowByte(cycle_count)); Wire.write(highByte(cycle_count)); log_tx_val = cycle_count; break;

    default:
      Wire.write(0); Wire.write(0); log_tx_val = 0; break;
  }
  log_tx_flag = true;
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    register_to_call = Wire.read();
  }
  log_rx_cmd = register_to_call;
  log_rx_flag = true;
}

void sync_bms_to_smbus() {
  bms.main_task(true);

  float v_pack = bms.get_voltage();
  if (v_pack <= 0.1f) {
    // No valid pack reading yet; keep existing/fallback values
    return;
  }

  overall_voltage      = (uint16_t)(v_pack * 1000.0f);            // V -> mV
  current              = (int16_t)(bms.get_current() * 1000.0f);  // A -> mA
  remaining_capacity   = (uint16_t)(bms.get_balance_capacity() * 1000.0f); // Ah -> mAh
  full_charge_capacity = (uint16_t)(bms.get_rate_capacity() * 1000.0f);    // Ah -> mAh
  relative_soc         = (uint16_t)bms.get_state_of_charge();    // %
  cycle_count          = (uint16_t)bms.get_cycle_count();

  // Convert first NTC probe from °C to 0.1 K (0 °C = 273.15 K -> 2731.5 tenths of K)
  if (bms.get_num_ntcs() > 0) {
    float temp_c = bms.get_ntc_temperature(0);
    temperature = (uint16_t)((temp_c + 273.15f) * 10.0f);
  }

  // Copy up to 7 cell voltages in mV
  uint8_t num_cells = bms.get_num_cells();
  for (uint8_t i = 0; i < 7; i++) {
    if (i < num_cells) {
      cell_voltage[i] = (uint16_t)(bms.get_cell_voltage(i) * 1000.0f);
    }
  }

  // Basic SBS status bits (bit 6 = charging when current > 0)
  if (current > 50) {
    battery_status = 0x0040;
  } else if (current < -50) {
    battery_status = 0x0002;
  } else {
    battery_status = 0x0000;
  }

  Serial.printf("[BMS SYNC] V: %u mV | I: %d mA | SOC: %u%% | T: %u (0.1K) | C1: %u mV\n",
                overall_voltage, current, relative_soc, temperature, cell_voltage[0]);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup BMS UART on GP8 (TX) / GP9 (RX)
  Serial2.setTX(BMS_TX);
  Serial2.setRX(BMS_RX);
  Serial2.begin(9600);
  bms.begin(&Serial2); //

  // Setup I2C0 on GP4 (SDA) / GP5 (SCL) as SMBus slave at 0x0B
  Wire.setSDA(CUSTOM_SDA);
  Wire.setSCL(CUSTOM_SCL);
  Wire.begin(0x0B);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("\n=== OverkillSolarBms2 (Serial2) -> SMBus (0x0B) ===");
}

void loop() {
  static uint32_t last_blink = 0;
  if (millis() - last_blink > 500) {
    last_blink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  static uint32_t last_poll = 0;
  if (millis() - last_poll > 1000) {
    last_poll = millis();
    sync_bms_to_smbus();
  }
}