/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

// todo : find a way to target BLEDevice.h from esp32 lib without absolute path
#include "/Users/fgutmann/Library/Arduino15/packages/esp32/hardware/esp32/3.0.2/libraries/BLE/src/BLEDevice.h"
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>


#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define GYRO_UUID "31d31ed5-aa9b-4325-b011-25caa3765c2a"
#define ACCEL_UUID "bcd6dfbe-0c7b-4530-a5b3-ecd2ed69ff4f"

#define SENSOR_DEVICE_COUNT 4
#define SENSOR_DEVICE_MEASURE_HZ 10

#define ANEMOMETER_MEASURE_HZ 1

#define SCAN_TIME 5  //In seconds


BLEScan *pBLEScan;

// The remote service we wish to connect to.
static BLEUUID serviceUUID(SERVICE_UUID);
// The characteristic of the remote service we are interested in.
static BLEUUID gyroUUID(GYRO_UUID);
static BLEUUID accelUUID(ACCEL_UUID);

struct Device {
  BLEClient *client;
  BLERemoteCharacteristic *gyro;
  BLERemoteCharacteristic *accel;
};

Device sensor_devices[SENSOR_DEVICE_COUNT];
int deviceCount = 0;

/// Callbacks on device connection / deconnection
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pClient) {
    Serial.print("onConnect : ");
    Serial.println(pClient->getPeerAddress().toString().c_str());

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    BLERemoteCharacteristic *pRemoteGyro = pRemoteService->getCharacteristic(gyroUUID);
    if (pRemoteGyro == nullptr) {
      Serial.print("Failed to find our gyro characterictic UUID: ");
      Serial.println(gyroUUID.toString().c_str());
      pClient->disconnect();
      return;
    }

    BLERemoteCharacteristic *pRemoteAccel = pRemoteService->getCharacteristic(accelUUID);
    if (pRemoteAccel == nullptr) {
      Serial.print("Failed to find our accel characteristic UUID: ");
      Serial.println(accelUUID.toString().c_str());
      pClient->disconnect();
      return;
    }

    Serial.println(" - Found our characteristics");

    Device device;
    device.client = pClient;
    device.gyro = pRemoteGyro;
    device.accel = pRemoteAccel;

    sensor_devices[deviceCount++] = device;

    Serial.println("- Connected to device");

  }

  void onDisconnect(BLEClient *pclient) {
    Serial.print("onDisconnect");
    Serial.println(pclient->getPeerAddress().toString().c_str());

    // todo : handle disconnecting properly
    for (int i = 0; i < deviceCount; i++) {
      if (sensor_devices[i].client == pclient) {
        sensor_devices[i].client = nullptr;
        deviceCount--;
        Serial.println("- Disconnected from device");
        break;
      }
    }

  }
};


// Callback on scan found a device
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
      Serial.printf("Found Device: %s \n", advertisedDevice.toString().c_str());
      
      // pBLEScan->stop();
      BLEAdvertisedDevice *myDevice = new BLEAdvertisedDevice(advertisedDevice);

      Serial.print("Forming a connection to ");
      Serial.println(myDevice->getAddress().toString().c_str());

      BLEClient *pClient = BLEDevice::createClient();
      Serial.println(" - Created client");

      pClient->setClientCallbacks(new MyClientCallback());

      // Connect to the remove BLE Server.
      pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
      Serial.println(" - Connected to server");
      pClient->setMTU(517);  //set client to request maximum MTU from server (default is 23 otherwise)
    }

  }
};


void readData(Device *device) {

  if (!device->gyro->canRead()) {
    Serial.print("Gyro value not available - device : ");
    Serial.println(device->client->getPeerAddress().toString().c_str());
  }

  if (!device->accel->canRead()) {
    Serial.print("Accel value not available - device : ");
    Serial.println(device->client->getPeerAddress().toString().c_str());
  }

  String gyroValue = device->gyro->readValue();
  String accelValue = device->accel->readValue();

  Serial.print("--- Device : ");
  Serial.print(device->client->getPeerAddress().toString().c_str());
  Serial.println(" ---");

  Serial.print("Gyro value : ");
  Serial.println(gyroValue.c_str());

  Serial.print("Accel value : ");
  Serial.println(accelValue.c_str());

}

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();  //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

  pBLEScan->start(SCAN_TIME, false);
}

void loop() {

  // fetch sensors data
  for (int i = 0; i < SENSOR_DEVICE_COUNT; i++) {

    Device device = sensor_devices[i];

    if (device.client == nullptr) {
      continue;
    }

    readData(&sensor_devices[i]);
  }

  // todo : better interval handling
  delay(1000/ANEMOMETER_MEASURE_HZ);

}
