/********************************************************************************/
/* ****.c                                                                       */
/* Auth-date: yyyy/mm/dd                                                        */
/*------------------------------------------------------------------------------*/
/* licence:   MIT                                                               */
/* Copyright: Ksamochi                                                          */
/* Depends on:                                                                  */
/********************************************************************************/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

/* ---------------------------------------------------------------- Definitions */
/* Hardware Definition */
#define GPS_RX_PIN (20)  /* ESP32-C3 RX (Connect to GPS TX) */
#define GPS_TX_PIN (21)  /* ESP32-C3 TX (Connect to GPS RX) */
#define LED_PIN    (8)   /* Onboard LED (Active LOW) */

/* Peer Device Name */
#define PAIR_DEV_NAME "AprTFT"

/* Nordic UART Service (NUS) UUID */
#define SVC_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/* Send packet format */
struct __attribute__((packed)) GpsPacket {
    uint32_t time_sec;  /* Elapsed seconds from 00:00:00 (0-86399) / Invalid: 0xFFFFFFFF */
    uint8_t  fix;       /* 0: No Fix, 1: Fix */
    uint8_t  sats;      /* Number of satellites (0-255) */
    int32_t  lat;       /* Latitude x 1,000,000 (e.g., 35.681234 -> 35681234) */
    int32_t  lng;       /* Longitude x 1,000,000 (e.g., 139.767123 -> 139767123) */
    uint16_t course;    /* Course x 10 (e.g., 120.5 deg -> 1205) */
    uint16_t speed;     /* Speed km/h x 10 (e.g., 45.2 km/h -> 452) */
};

/* ------------------------------------------------------------------ Variables */
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

BLEServer *pServer = NULL;
BLECharacteristic *pTxChar = NULL;
bool nowDevConn = false;
bool oldDevConn = false;
bool tagDevFound = false;


/* ----------------------------------------------- class / Function Proto-Types */
class MyAdvDevCb: public BLEAdvertisedDeviceCallbacks {
	void onResult(BLEAdvertisedDevice advertisedDevice) override;
};

class MySvrCb: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) override;
	void onDisconnect(BLEServer* pServer) override;
};


/* *************************************************************** Main Process */
/* ---------------------------------------------------------------------- setup */
void setup()
{
    pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	Serial.begin(115200);
	delay(1000);
	Serial.println("\n=== ESP32-C3 GPS BLE NUS ===");

	gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

	BLEDevice::init("ApeGPS");

	/* GAP Scan */
	BLEScan* pScan = BLEDevice::getScan();
	pScan->setAdvertisedDeviceCallbacks((BLEAdvertisedDeviceCallbacks*)new MyAdvDevCb());
	pScan->setActiveScan(true);
	pScan->setInterval(100);
	pScan->setWindow(99);
	pScan->start(10, false);

	/* GATT Server Setup */
	pServer = BLEDevice::createServer();
	pServer->setCallbacks((BLEServerCallbacks*)new MySvrCb());

	BLEService *pSvc = pServer->createService(SVC_UUID);

	pTxChar = pSvc->createCharacteristic(
		CHAR_UUID_TX,
		BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
	);
	pTxChar->addDescriptor(new BLE2902());      /* Subscription Endpoint Ready */
	pTxChar->setValue("Waiting for GPS...");

	pSvc->start();

    /* Advertise */
	BLEAdvertising *pAdv = BLEDevice::getAdvertising();
	pAdv->addServiceUUID(SVC_UUID);
	pAdv->setScanResponse(true);
	pAdv->setMinPreferred(0x06);
	pAdv->setMinPreferred(0x12);
	BLEDevice::startAdvertising();
    Serial.println("[BLE] Advertise");
}

/* ----------------------------------------------------------------------- loop */
void loop()
{
    bool parsed = false;
    while (gpsSerial.available() > 0) {
		if (gps.encode(gpsSerial.read())) {     /* parse */
			parsed = true;                      /* done */
		}
	}

    if (parsed && gps.date.isUpdated()) {
        GpsPacket pkt;

		/* 1. Convert time (elapsed seconds) */
		if (gps.time.isValid()) {
			pkt.time_sec = (uint32_t)gps.time.hour() * 3600 +
			               (uint32_t)gps.time.minute() * 60 +
			               (uint32_t)gps.time.second();
		} else {
			pkt.time_sec = 0xFFFFFFFF;          /* Invalid value */
		}

		/* 2. Convert flags and numeric values (integer conversion) */
		pkt.fix    = gps.location.isValid() ? 1 : 0;
		pkt.sats   = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
		pkt.lat    = gps.location.isValid() ? (int32_t)(gps.location.lat() * 1000000.0) : 0;
		pkt.lng    = gps.location.isValid() ? (int32_t)(gps.location.lng() * 1000000.0) : 0;
		pkt.course = gps.course.isValid()   ? (uint16_t)(gps.course.deg() * 10.0) : 0;
		pkt.speed  = gps.speed.isValid()    ? (uint16_t)(gps.speed.kmph() * 10.0) : 0;

		/* 3. Set as binary data and send notification */
		pTxChar->setValue((uint8_t*)&pkt, sizeof(pkt));

		if (newDevConn) {
			pTxChar->notify();
		}
	}

	/* 4. Automatic disconnection and reconnection control */
	if (!nowDevConn && oldDevConn) {
		delay(500);
		pServer->startAdvertising();
		Serial.println("[BLE] Re:Advertise");
	}
	oldDevConn = nowDevConn;
}

/* **************************************************************** Sub Process */
/* ---------------------------------------------------- MyAdvDevCb class method */
void MyAdvDevCb::onResult(BLEAdvertisedDevice advertisedDevice)
{
	if (advertisedDevice.haveName()) {
		String devName = advertisedDevice.getName().c_str();
		if (devName == PAIR_DEV_NAME) {
			tagDevFound = true;
			advertisedDevice.getScan()->stop();
		}
	}
}

/* ------------------------------------------------------- MySvrCb class method */
void MySvrCb::onConnect(BLEServer* pServer)
{
    nowDevConn = true;
	digitalWrite(LED_PIN, LOW);
}

void MySvrCb::onDisconnect(BLEServer* pServer)
{
	nowDevConn = false;
	digitalWrite(LED_PIN, HIGH);
}
