#include <WioLTEforArduino.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <msgpack.h>
#include <Client.h>
#include <stdio.h>
#include <time.h>


#include <WioLTEforArduino.h>
#include <WioLTEClient.h>
#include <PubSubClient.h>    // https://github.com/knolleary/pubsubclient

#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



#define APN               "soracom.io"
#define USERNAME          "sora"
#define PASSWORD          "sora"


//----------------declare variables on  wio area -------------------------------
WioLTE Wio;
WioLTEClient WioClient(&Wio);

//----------------declare variables on  wio area -------------------------------



//----------------declare variables on  motion_data area -------------------------------
const int flagPin = 20;

// declare the sensor 
Adafruit_BNO055 bno = Adafruit_BNO055();

//uint16_t seqNum = 1;
//uint16_t deltaT = 1;

typedef struct {
  uint16_t sN;
  uint16_t dT;
  int16_t accx; int16_t accy; int16_t accz;
  int16_t gyrox; int16_t gyroy; int16_t gyroz;
  int16_t magx; int16_t magy; int16_t magz;
} motion_data_t;

//----------------declare variables on  motion_data area -------------------------------





//----------------declare variables on mqtt publish  area -------------------------------
PubSubClient MqttClient;
const char apiKey[] = "v1.ZHwyNDk3.1519720484.ad90647f636cac481eae98a2f3d0efadcd59bddd909f726294c371286b23bd33e0d84103fe1173e783e23a6fda8cf2b372cffac12c889fc3746dd9dd4f059a21022cf0bc392a1ef3";

#define MQTT_SERVER_HOST  "windsurfinglab.southeastasia.cloudapp.azure.com"
#define MQTT_SERVER_PORT  (10091)
#define MQTT_USER         ""
#define MQTT_PASS         ""
#define ID                "test"


#define START_TOPIC      "/2497/start"
#define STOP_TOPIC       "/2497/stop"
#define DATA_TOPIC       "/2497/data"
#define TEXT_TOPIC       "/2497/text"
#define IN_TOPIC         "inTopic"
//----------------declare variables on mqtt publish  area -------------------------------




//----------------declare variables on mqtt publish  area -------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  SerialUSB.print("Subscribe:");
  for (int i = 0; i < length; i++) SerialUSB.print((char)payload[i]);
  SerialUSB.println("");
}
//----------------declare variables on mqtt publish  area -------------------------------





//----------------declare functions on  motion_data area -------------------------------

void readBno055Int16Cycle(motion_data_t *motion_data) {

  // declare a buffer holding all the values of speed metric
  int len = 18;
  uint8_t buff[len];

  // declare each speed  metric
//  uint8_t accBuf[6];
//  uint8_t gyroBuf[6];
//  uint8_t magBuf[6];

  // get data and put it in the buffer
  Wire.beginTransmission(0x28); // BNO055アドレス
  Wire.write((uint8_t)0x08);  // ACC_DATA_X_LSBレジスタ
  Wire.endTransmission();

  Wire.requestFrom(0x28, (byte)len);

  for (uint8_t i = 0; i < len; i++)
  {
    buff[i] = Wire.read();
    SerialUSB.print("buff");
    SerialUSB.print(i);
    SerialUSB.print(":");
    SerialUSB.println(buff[i]);
    delay(1000);
  }


  // input the data of buff to its area
//  accBuf[0] = buff[1]; accBuf[1] = buff[0];
//  accBuf[2] = buff[3]; accBuf[3] = buff[2];
//  accBuf[4] = buff[5]; accBuf[5] = buff[4];
//  magBuf[0] = buff[7]; magBuf[1] = buff[6];
//  magBuf[2] = buff[9]; magBuf[3] = buff[8];
//  magBuf[4] = buff[11]; magBuf[5] = buff[10];
//  gyroBuf[0] = buff[13]; gyroBuf[1] = buff[12];
//  gyroBuf[2] = buff[15]; gyroBuf[3] = buff[14];
//  gyroBuf[4] = buff[17]; gyroBuf[5] = buff[16];
//
//
//  // construct a motion data
//  motion_data->sN = seqNum;
//
//  motion_data->dT = deltaT;
//
//  motion_data->accx = (short)((accBuf[0] << 8 & 0xFF00) | (accBuf[1] & 0x00FF));
//  motion_data->accy = (short)((accBuf[2] << 8 & 0xFF00) | (accBuf[3] & 0x00FF));
//  motion_data->accz = (short)((accBuf[4] << 8 & 0xFF00) | (accBuf[5] & 0x00FF));
//
//  motion_data->gyrox = (short)((gyroBuf[0] << 8 & 0xFF00) | (gyroBuf[1] & 0x00FF));
//  motion_data->gyroy = (short)((gyroBuf[2] << 8 & 0xFF00) | (gyroBuf[3] & 0x00FF));
//  motion_data->gyroz = (short)((gyroBuf[4] << 8 & 0xFF00) | (gyroBuf[5] & 0x00FF));
//
//  motion_data->magx = (short)((magBuf[0] << 8 & 0xFF00) | (magBuf[1] & 0x00FF));
//  motion_data->magy = (short)((magBuf[2] << 8 & 0xFF00) | (magBuf[3] & 0x00FF));
//  motion_data->magz = (short)((magBuf[4] << 8 & 0xFF00) | (magBuf[5] & 0x00FF));

}

//----------------declare functions on  motion_data area -------------------------------





//----------------declare functions on Wio-------------------------------
void startWio() {

  SerialUSB.println("### I/O Initialize.");
  Wio.Init();

  SerialUSB.println("### Power supply ON.");
  Wio.PowerSupplyLTE(true);
  Wio.PowerSupplyGrove(true);
  delay(200);

  // Initialize the sensor
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(flagPin, HIGH);
    while (1);
  }
  delay(200);
  bno.setExtCrystalUse(true);

  SerialUSB.println("### Turn on or reset.");
  if (!Wio.TurnOnOrReset()) {
    SerialUSB.println("### ERROR! ###");
    return;
  }

  SerialUSB.println("### Connecting to \""APN"\".");
  if (!Wio.Activate(APN, USERNAME, PASSWORD)) {
    SerialUSB.println("### ERROR! ###");
    return;
  }

}
//----------------declare functions on Wio-------------------------------





//----------------declare functions on MQTT-------------------------------
void startMqtt() {

  SerialUSB.println("### Connecting to MQTT server \""MQTT_SERVER_HOST"\"");
  MqttClient.setServer(MQTT_SERVER_HOST, MQTT_SERVER_PORT);
  MqttClient.setCallback(callback);
  MqttClient.setClient(WioClient);

  if (!MqttClient.connect(ID, MQTT_USER, MQTT_PASS)) {
    SerialUSB.println("### ERROR! ###");
    return;
  }

  MqttClient.subscribe(IN_TOPIC);

}

//----------------declare functions on MQTT-------------------------------






//--------------------declare setup function --------------------------------

void setup() {

  SerialUSB.println("");
  SerialUSB.println("### Setup start.");

  // start wio
  startWio();
  delay(200);

  // start mqtt
  startMqtt();
  delay(200);

  SerialUSB.println("### Setup completed.");


}
//--------------------declare setup function --------------------------------



//------------declare loop function---------------------

void loop() {


     
  SerialUSB.println("begin looping");


// ----- motion data ------------------------- 
  // declare motion_data
//  motion_data_t * motion_data;

  // set motion_data  
//  readBno055Int16Cycle(motion_data);

  // print out the motion data
//  SerialUSB.println("the motion_data.sN:");
//  SerialUSB.println(motion_data->sN);
//
//  SerialUSB.print("motion_data.dT:");
//  SerialUSB.println(motion_data->dT);
//
//  SerialUSB.print("motion_data.accx : ");
//  SerialUSB.println(motion_data->accx);
//
//  SerialUSB.print("motion_data.accy : ");
//  SerialUSB.println(motion_data->accy);
//
//  SerialUSB.print("motion_data.accz : ");
//  SerialUSB.println(motion_data->accz);
//
//  SerialUSB.print("motion_data.accx : ");
//  SerialUSB.println(motion_data->accx);
//
//  SerialUSB.print("motion_data.gyrox: ");
//  SerialUSB.println(motion_data->gyrox);
//
//  SerialUSB.print("motion_data.gyroy: ");
//  SerialUSB.println(motion_data->gyroy);
//
//  SerialUSB.print("motion_data.gyroz: ");
//  SerialUSB.println(motion_data->gyroz);
//
//  SerialUSB.print("motion_data.magx: ");
//  SerialUSB.println(motion_data->magx);
//
//  SerialUSB.print("motion_data.magy: ");
//  SerialUSB.println(motion_data->magy);
//
//  SerialUSB.print("motion_data.magz: ");
//  SerialUSB.println(motion_data->magz);


// ------------Mqtt ----------------

  char data[100];

  sprintf(data, "{\"uptime\":%lu}", millis() / 1000);
  SerialUSB.print("Publish:");
  SerialUSB.print(data);
  SerialUSB.println("");
  MqttClient.publish(START_TOPIC, apiKey);
  MqttClient.publish(TEXT_TOPIC, data);
  MqttClient.publish(STOP_TOPIC, "ok");

  delay(2000);

}

//------------declare loop function---------------------
