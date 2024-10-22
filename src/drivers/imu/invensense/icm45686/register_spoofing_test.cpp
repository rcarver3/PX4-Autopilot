#include <SPI.h>

// Define SPI settings
const int SPI_CS_PIN = 10;
const int SPI_CLOCK_SPEED = 1000000; // 1 MHz
const int SPI_MODE = SPI_MODE0;

// Define ICM45686 registers and constants
#define WHO_AM_I 0x72
#define WHOAMI_RESPONSE 0xE9
#define FIFO_DATA 0x14
#define FIFO_SIZE 8192

// Define FIFO data structure
struct FIFOData {
  uint8_t FIFO_Header;
  uint8_t ACCEL_DATA_XH;
  uint8_t ACCEL_DATA_XL;
  uint8_t ACCEL_DATA_YH;
  uint8_t ACCEL_DATA_YL;
  uint8_t ACCEL_DATA_ZH;
  uint8_t ACCEL_DATA_ZL;
  uint8_t GYRO_DATA_XH;
  uint8_t GYRO_DATA_XL;
  uint8_t GYRO_DATA_YH;
  uint8_t GYRO_DATA_YL;
  uint8_t GYRO_DATA_ZH;
  uint8_t GYRO_DATA_ZL;
  uint8_t TEMP_DATA_H;
  uint8_t TEMP_DATA_L;
  uint8_t Timestamp_H;
  uint8_t Timestamp_L;
  uint8_t HIGHRES_X_LSB;
  uint8_t HIGHRES_Y_LSB;
  uint8_t HIGHRES_Z_LSB;
};

// Register map
uint8_t register_map[256] = {0};

// FIFO buffer
FIFOData fifo_buffer[FIFO_SIZE];
int fifo_index = 0;

// Function to generate random IMU data
void generateRandomIMUData() {
  for (int i = 0; i < FIFO_SIZE; i++) {
    fifo_buffer[i].FIFO_Header = 0x6A; // Example header
    fifo_buffer[i].ACCEL_DATA_XH = random(0, 256);
    fifo_buffer[i].ACCEL_DATA_XL = random(0, 256);
    fifo_buffer[i].ACCEL_DATA_YH = random(0, 256);
    fifo_buffer[i].ACCEL_DATA_YL = random(0, 256);
    fifo_buffer[i].ACCEL_DATA_ZH = random(0, 256);
    fifo_buffer[i].ACCEL_DATA_ZL = random(0, 256);
    fifo_buffer[i].GYRO_DATA_XH = random(0, 256);
    fifo_buffer[i].GYRO_DATA_XL = random(0, 256);
    fifo_buffer[i].GYRO_DATA_YH = random(0, 256);
    fifo_buffer[i].GYRO_DATA_YL = random(0, 256);
    fifo_buffer[i].GYRO_DATA_ZH = random(0, 256);
    fifo_buffer[i].GYRO_DATA_ZL = random(0, 256);
    fifo_buffer[i].TEMP_DATA_H = random(0, 256);
    fifo_buffer[i].TEMP_DATA_L = random(0, 256);
    fifo_buffer[i].Timestamp_H = random(0, 256);
    fifo_buffer[i].Timestamp_L = random(0, 256);
    fifo_buffer[i].HIGHRES_X_LSB = random(0, 256);
    fifo_buffer[i].HIGHRES_Y_LSB = random(0, 256);
    fifo_buffer[i].HIGHRES_Z_LSB = random(0, 256);
  }
}

// SPI transaction handler
void handleSPITransaction() {
  static uint8_t address = 0;
  static bool reading = false;

  if (digitalRead(SPI_CS_PIN) == LOW) {
    if (SPI.transfer(0x00) & 0x80) {
      // Read operation
      reading = true;
      address = SPI.transfer(0x00) & 0x7F;
    } else {
      // Write operation
      reading = false;
      address = SPI.transfer(0x00);
      register_map[address] = SPI.transfer(0x00);
    }

    if (reading) {
      if (address == WHO_AM_I) {
        SPI.transfer(WHOAMI_RESPONSE);
      } else if (address == FIFO_DATA) {
        SPI.transfer(reinterpret_cast<uint8_t*>(&fifo_buffer[fifo_index]), sizeof(FIFOData));
        fifo_index = (fifo_index + 1) % FIFO_SIZE;
      } else {
        SPI.transfer(register_map[address]);
      }
    }
  }
}

void setup() {
  // Initialize SPI
  pinMode(SPI_CS_PIN, INPUT_PULLUP);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE));

  // Generate random IMU data
  generateRandomIMUData();
}

void loop() {
  // Handle SPI transactions
  handleSPITransaction();
}
