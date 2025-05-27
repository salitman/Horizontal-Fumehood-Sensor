// ESP32-C3 Mounted Horizontal Fume Hood Sensor Sender
// Desc: Contains functions for interfacing with LIS2DE12TR accelerometer 

uint8_t spiRead(uint8_t address) {
  // Address should begin with 10 when reading, 00 when writing
  if (address < 0b10000000)
    address = address + 0x80; 
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE));
  digitalWrite(SS, LOW);
  uint16_t DATA = SPI.transfer16( (address << 8) | 0b00000000 );
  digitalWrite(SS, HIGH);
  SPI.endTransaction();

  return uint8_t(DATA);
}

void spiWrite(uint8_t address, uint8_t info) {
  // Address should begin with 10 when reading, 00 when writing
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE));
  digitalWrite(SS, LOW);
  uint16_t DATA = SPI.transfer16( (address << 8) | info );
  //Serial.println((address << 8) | info);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();

  return;
}

void LIS2DE12TR_StartupCheck() {
  // Check if LISDE12TR defaults are correct and communication working
  Serial.print("My name is: ");
  Serial.print(spiRead( 0x0F ));
  Serial.println(", Default is 51");

  Serial.print("My status is: ");
  Serial.print(spiRead( 0x03 ));
  Serial.println(", Default is 0");

  Serial.print("CTRL_REG_1: ");
  Serial.print(spiRead( 0x20 ));
  Serial.println(", Default is 7");

  Serial.print("CTRL_REG_0: ");
  Serial.print(spiRead( 0x1E ));
  Serial.println(", Default is 16");

  Serial.print("CTRL_REG_3: ");
  Serial.print(spiRead( 0x22 ));
  Serial.println(", Default is 0");

  Serial.print("INT1_SRC: ");
  Serial.print(spiRead( 0x31 ));
  Serial.println(", Default is 0");
}

void LIS2DE12TR_Initialize(int data_resolution) {
  // Data resolution options: 1Hz, 10Hz, 25Hz, 50Hz, 100Hz, 200Hz, 400Hz
  switch(data_resolution) {
    case 1: spiWrite( 0x20, 0b00010111); break;
    case 10: spiWrite( 0x20, 0b00100111); break;
    case 25: spiWrite( 0x20, 0b00110111); break;
    case 50: spiWrite( 0x20, 0b01000111); break;
    case 100: spiWrite( 0x20, 0b01010111); break;
    case 200: spiWrite( 0x20, 0b01100111); break;
    case 400: spiWrite( 0x20, 0b01110111); break;
    default: spiWrite( 0x20, 0b00110111); break;
  }
  Serial.print("CTRL_REG_1 set to: ");
  Serial.println(spiRead( 0x20 ));
}

void LIS2DE12TR_Int1_Setup(float threshold, float duration) {
  // Threshold of interrupt in g-force, minimum duration to detect in seconds
  spiWrite( 0x22, 0b01000000); // CTRL_REG_3, Enable Interupt 1 on pin Int1
  spiWrite( 0x30, 0b00001000); // INT1_CNFG, 6LSB are X-low, X-high, Y-low, Y-high, Z-low, Z-high

  // Maximum threshold for interrupt is 2g (at default scale)
  uint8_t threshold_bits = (uint8_t)(threshold / 0.016);
  if (threshold_bits > 0b01111111)
    threshold_bits = 0b01111111;
  spiWrite( 0x32, threshold_bits); // INT1_THRESHOLD

  int resolution_vec[] = {0, 1, 10, 25, 50, 100, 200, 400};
  uint8_t ODR_bits = spiRead( 0x20 ) >> 4;
  uint8_t N = (uint8_t)(duration * resolution_vec[ODR_bits]);
  if (N > 0b01111111)
    N = 0b01111111;
  spiWrite( 0x33, N); // INT1_DURATION

  Serial.println();
  Serial.print("CTRL_REG_3: ");
  Serial.println(spiRead( 0x22 )); 
  Serial.print("INT1_CNFG: ");
  Serial.println(spiRead( 0x30 )); 
  Serial.print("INT1_THRESHOLD: ");
  Serial.println(spiRead( 0x32 )); 
  Serial.print("INT1_DURATION: ");
  Serial.println(spiRead( 0x33 ));
  Serial.print("INT1_SRC: ");
  Serial.println(spiRead( 0x31 ));
}

uint8_t LIS2DE12TR_Int1_Clear() {
  uint8_t reg = spiRead( 0x31 );
  Serial.print("INT1_SRC: "); Serial.println(reg);
  return reg;
}
