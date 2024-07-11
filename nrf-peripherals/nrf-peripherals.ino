#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <stdio.h>
      
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define GYRO_UUID "31d31ed5-aa9b-4325-b011-25caa3765c2a"
#define ACCEL_UUID "bcd6dfbe-0c7b-4530-a5b3-ecd2ed69ff4f"

#define DISCOVERY_INTERVAL 1000 // Ms
#define SEND_INTERVAL 10 // Hz

BLEService dataService(SERVICE_UUID); 
BLECharacteristic gyroCharacteristic(GYRO_UUID, BLERead | BLEWrite, 100);
BLECharacteristic accelCharacterictic(ACCEL_UUID, BLERead | BLEWrite, 100);

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

void setup() {
  Serial.begin(9600);
  while (!Serial);  

  if (myIMU.begin() != 0) {
      Serial.println("IMU error");
  } else {
      Serial.println("IMU OK!");
  }
  
  if (!BLE.begin()) {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  BLE.setLocalName("XIAO BLE Sense (Peripheral)");
  BLE.setAdvertisedService(dataService);
  dataService.addCharacteristic(gyroCharacteristic);
  dataService.addCharacteristic(accelCharacterictic);
  BLE.addService(dataService);

  BLE.advertise();

  BLE.poll();

  Serial.println("XIAO BLE Sense (Peripheral)");
  Serial.println(" ");
}

void loop() {
  BLEDevice central = BLE.central();
  Serial.println("- Discovering central device...");
  delay(500);

  if (central) {
    Serial.println("* Connected to central device!");
    Serial.print("* Device MAC address: ");
    Serial.println(central.address());
    Serial.println(" ");

    while (central.connected()) {

      char gyro[100];
      sprintf(gyro, "%.2f;%.2f;%.2f", myIMU.readFloatGyroX(), myIMU.readFloatGyroY(),  myIMU.readFloatGyroZ());
      gyroCharacteristic.writeValue(gyro);
      Serial.println(gyro);

      char accelerometer[100];
      sprintf(accelerometer, "%.2f;%.2f;%.2f", myIMU.readFloatAccelX(), myIMU.readFloatAccelY(),  myIMU.readFloatAccelZ());
      accelCharacterictic.writeValue(accelerometer);
      Serial.println(accelerometer);

      delay(1000/SEND_INTERVAL);

    }
    
    Serial.println("* Disconnected to central device!");
  }
}
