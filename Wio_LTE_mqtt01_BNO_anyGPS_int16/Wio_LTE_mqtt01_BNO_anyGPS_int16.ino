#include <WioLTEforArduino.h>

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

// cloudMQTT TestServer2
//#define MQTT_SERVER_HOST  "m14.cloudmqtt.com"
//#define MQTT_SERVER_PORT  (17052)
//#define MQTT_USER         "ufvvbjdp"
//#define MQTT_PASS         "9JjCaD9GbBna"
// #define ID                "test"

// cloudMQTT TestServer2
// #define MQTT_SERVER_HOST  "m11.cloudmqtt.com"
// #define MQTT_SERVER_PORT  (12272)
// #define MQTT_USER         "vlqxkpun"
// #define MQTT_PASS         "rSKYUmzMT5yW"
// #define ID                "test"

// mode Device Server
#define MQTT_SERVER_HOST  "windsurfinglab.southeastasia.cloudapp.azure.com"
#define MQTT_SERVER_PORT  (10090)
#define MQTT_USER         ""
#define MQTT_PASS         ""
#define ID                "test"

#define OUT_TOPIC         "outTopic"
#define IN_TOPIC          "inTopic"

WioLTE Wio;
WioLTEClient WioClient(&Wio);
PubSubClient MqttClient;
bool isMqttConnected = false;

const int flagPin = 20;  // 処理時間監視用など

const int schema_gps = 1;
const int schema_motion = 2;

const char endpoint[] = "api.tinkermode.com";
int port = 80;
//const int deviceId = 2478;
// const int deviceId = 2477;
const int deviceId = 2497;

//char apiKey[] = "v1.ZHwyNDc4.1519349822.a114a3b71e0315ea685415310478efcf10639ae003dfcdfb83ba87677de01c1115e7844279d84811af3457e40c036d6fb508556923d213a2b998768f58e0205f9a02c6064f5a5c62";
// const char apiKey[] = "v1.ZHwyNDc3.1519348403.3118b761523acad95caa43626b5c9e2ac74794dbcc259da4f69a71434b62890d088220bfdd7c690522cdb0e5ac80f3d7dfb3fa17442724601555bac5ac1fb33e5538229928b94891";
// const char apiKey[] = "v1.ZHwyNDQz.1518048374.624196eebe2023f02c9741fa03ffa7f94c0720197839af8902b2af4f12e9977da8422b8f400585b67ebef2770e03f42abdb030c226d93adca5c4a594b167539308f0ef5880b4a75a";
const char apiKey[] = "v1.ZHwyNDk3.1519720484.ad90647f636cac481eae98a2f3d0efadcd59bddd909f726294c371286b23bd33e0d84103fe1173e783e23a6fda8cf2b372cffac12c889fc3746dd9dd4f059a21022cf0bc392a1ef3";
bool isClaimed = false;
const int duration = 380;


//const char path[] = "/devices/2478";
const char path[] = "/devices/2497";
const char path2[] = "/deviceRegistration";
char header[500];
String response;
String response2;
int statusCode = 0;

HttpClient client = HttpClient( WioClient, endpoint, port);
// ----------------------- miyazaki-end --------------------------------------------------------------------------------

// GPS関連
Adafruit_GPS anyGPS(&Serial);  // Connect to the GPS on the hardware port
char * gpsData;
byte * gpsSbuffData;
int gpsSbufSize = 0;
int nmeaSize;
const int baudrate = 9600;

// 9軸関連
Adafruit_BNO055 bno = Adafruit_BNO055();
// uint16_t seqNum = 0;
uint16_t seqNum = 1;
int cycleCount = 0;
const int bufCycle = 1; // MAXでも2？
bool toSendMqtt;
unsigned long deltaT, motion_ts_40, motion_ts_1000;
const int interval_40 = 40;  // millisecondで指定 (40msec = 25Hz)
// const int interval_40 = 200;  // millisecondで指定 (40msec = 25Hz)
bool isGpsData = false;

const int interval_1000 = 1000;  // millisecondで指定 (1000msec)

typedef struct {
  uint16_t sN;
  uint16_t dT;
  int16_t accx; int16_t accy; int16_t accz;
  int16_t gyrox; int16_t gyroy; int16_t gyroz;
  int16_t magx; int16_t magy; int16_t magz;
} motion_data_t;

// motion_data_t motion_data[bufCycle];

// msgpack関連
msgpack_sbuffer sbuf;
msgpack_packer pk;


// acclerator metric
int before_acc;

void readBno055Int16Cycle(motion_data_t *motion_data) {
  int len = 18;
  uint8_t buff[len];
  uint8_t accBuf[6];
  uint8_t gyroBuf[6];
  uint8_t magBuf[6];

  digitalWrite(flagPin, HIGH);
  Wire.beginTransmission(0x28); // BNO055アドレス
  Wire.write((uint8_t)0x08);  // ACC_DATA_X_LSBレジスタ
  Wire.endTransmission();
  Wire.requestFrom(0x28, (byte)len);
  for (uint8_t i = 0; i < len; i++)
  {
    buff[i] = Wire.read();
  }
  digitalWrite(flagPin, LOW);
  // BNO055 Output LSB->MSB, Acc->Mag->Gyro なのでデータ順の入れ替え
  accBuf[0] = buff[1]; accBuf[1] = buff[0];
  accBuf[2] = buff[3]; accBuf[3] = buff[2];
  accBuf[4] = buff[5]; accBuf[5] = buff[4];
  magBuf[0] = buff[7]; magBuf[1] = buff[6];
  magBuf[2] = buff[9]; magBuf[3] = buff[8];
  magBuf[4] = buff[11]; magBuf[5] = buff[10];
  gyroBuf[0] = buff[13]; gyroBuf[1] = buff[12];
  gyroBuf[2] = buff[15]; gyroBuf[3] = buff[14];
  gyroBuf[4] = buff[17]; gyroBuf[5] = buff[16];

  motion_data[cycleCount].sN = seqNum;
  motion_data[cycleCount].dT = deltaT;
  motion_data[cycleCount].accx = (short)((accBuf[0] << 8 & 0xFF00) | (accBuf[1] & 0x00FF));
  motion_data[cycleCount].accy = (short)((accBuf[2] << 8 & 0xFF00) | (accBuf[3] & 0x00FF));
  motion_data[cycleCount].accz = (short)((accBuf[4] << 8 & 0xFF00) | (accBuf[5] & 0x00FF));
  motion_data[cycleCount].gyrox = (short)((gyroBuf[0] << 8 & 0xFF00) | (gyroBuf[1] & 0x00FF));
  motion_data[cycleCount].gyroy = (short)((gyroBuf[2] << 8 & 0xFF00) | (gyroBuf[3] & 0x00FF));
  motion_data[cycleCount].gyroz = (short)((gyroBuf[4] << 8 & 0xFF00) | (gyroBuf[5] & 0x00FF));
  motion_data[cycleCount].magx = (short)((magBuf[0] << 8 & 0xFF00) | (magBuf[1] & 0x00FF));
  motion_data[cycleCount].magy = (short)((magBuf[2] << 8 & 0xFF00) | (magBuf[3] & 0x00FF));
  motion_data[cycleCount].magz = (short)((magBuf[4] << 8 & 0xFF00) | (magBuf[5] & 0x00FF));

  seqNum++;
  cycleCount++;
  if (cycleCount >= bufCycle) {
    toSendMqtt = true;
    cycleCount = 0;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  SerialUSB.print("Subscribe:");
  for (int i = 0; i < length; i++) SerialUSB.print((char)payload[i]);
  SerialUSB.println("");
}

int split(String data, char delimiter, String *dst) {
  //  SerialUSB.print("data: ");
  //  SerialUSB.println(data);
  int index = 0;
  //  int arraySize = (sizeof(data) / sizeof((data)[0]));
  int arraySize = (sizeof(data) / sizeof((data)[0])) + 1;
  //  SerialUSB.print("arraySize: ");
  //  SerialUSB.println(arraySize);

  int datalength = data.length();
  //  SerialUSB.print("datalength: ");
  //  SerialUSB.println(datalength);

  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    //    SerialUSB.print("tmp: ");
    //    SerialUSB.println(tmp);
    if ( tmp == delimiter ) {
      //      SerialUSB.println("==");
      index++;
      if ( index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}
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
void claimingAndCreateSession() {
  // claim初期要求発行
  client.beginRequest();
  client.get( path );
  client.sendHeader("Content-Type", "application/json");
  sprintf(header, "ModeCloud %s", apiKey);
  client.sendHeader("Authorization", header);
  client.endRequest();

  // read the status code and body of the response
  statusCode = client.responseStatusCode();
  response = client.responseBody();

  SerialUSB.print("Status code: ");
  SerialUSB.println(statusCode);
  SerialUSB.print("Response: ");
  SerialUSB.println(response);
  SerialUSB.println("Wait five seconds");

  // 端末が登録されているか確認(claimTime と homeId があるかないか)
  StaticJsonBuffer<200> jsonBuffer;

  char res[500];
  response.toCharArray(res, sizeof(res) );

  SerialUSB.println(res);
  JsonObject& root = jsonBuffer.parseObject(res);
  const char* homeId = root["homeId"];
  const char* claimTime = root["claimTime"];
  if (!root.success()) {
    SerialUSB.println("parseObject() failed");
    return;
  }
  if (homeId != NULL && claimTime != NULL) {
    isClaimed = true;
  }
  client.stop();

  if (!isClaimed) {
    SerialUSB.println("Let's claim.");

    // post発行
    char body[] = "";

    sprintf(body , "deviceId=%d&claimable=%d&duration=%d" , deviceId , true, duration);
    SerialUSB.print("body : ");
    SerialUSB.println(body);

    client.beginRequest();
    client.post(path2);

    client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
    client.sendHeader("Content-Length", strlen(body));

    SerialUSB.print("Content-Length : ");
    SerialUSB.println(strlen(body));

    sprintf(header, "ModeCloud %s", apiKey);
    client.sendHeader("Authorization", header);
    SerialUSB.print("header : ");
    SerialUSB.println(header);

    client.beginBody();
    client.print(body);
    client.endRequest();

    // postの戻りを解析、判断
    statusCode = client.responseStatusCode();
    response2 = client.responseBody();

    SerialUSB.print("Status code: ");
    SerialUSB.println(statusCode);

    if (statusCode == 200 ) {
      SerialUSB.println("httpPOST OK");
      SerialUSB.print("Response2: ");
      SerialUSB.println(response2);
    } else {
      SerialUSB.println("Can't post HTTP");
      SerialUSB.print("Response2: ");
      SerialUSB.println(response2);
    }
  }
  client.stop();
  // TODO claimされていなかった場合、携帯操作の間（380secの間）待って、再度claimされていることを確認する処理を入れる。
}

void beginBulkuploader() {
  // バルクアップローダー向けに MQTT を送る初期処理
  SerialUSB.println("### Connecting to MQTT server \""MQTT_SERVER_HOST"\"");
  MqttClient.setServer(MQTT_SERVER_HOST, MQTT_SERVER_PORT);
  MqttClient.setCallback(callback);
  MqttClient.setClient(WioClient);
  if (!MqttClient.connect(ID, MQTT_USER, MQTT_PASS)) {
    SerialUSB.println("### ERROR! ###");
    return;
  } else {
    isMqttConnected = true;
  }
  MqttClient.subscribe(IN_TOPIC);

  // バルクアップローダー向けの開始処理
  MqttClient.publish("/2497/start", apiKey);
}




// the setup function --------------------------------

void setup() {
  pinMode(flagPin, OUTPUT);
  delay(200);
  SerialUSB.println("");
  SerialUSB.println("### Setup start.");

  // WioやGPSを起動しておく
  startWio();
  anyGPS.begin(baudrate);

  // 現在の加速度合算を退避, initializing acc metric
  before_acc = getAccSum();

  SerialUSB.println("### Setup completed.");
}

// the setup function --------------------------------


// String bufToString(char const* buf, unsigned int len)
// {
//   SerialUSB.println("--- bufToString-start ---");
//   size_t i = 0;
//   String tmp = "";
//   for (; i < len ; ++i) {
//     char buffer[50];
//     sprintf(buffer, "%02x", 0xff & buf[i]);
//     tmp += (String)buffer;
//   }
//   SerialUSB.println("--- bufToString-end ---");
//   return tmp;
// }

// String msgpacking(int flag) {
//   SerialUSB.println("--- msgpacking_start ---");
//
//   // msgpack化処理
//   // 初期化
//   msgpack_sbuffer_init(&sbuf);
//
//   // とりあえずMOTION (1) を入れる
//   msgpack_pack_int(&pk, schema_motion );
//   msgpack_pack_int(&pk, motion_data[0].sN );
//
//   // 10の配列作成
//   msgpack_pack_array(&pk, 10);
//
//   //時間と9軸の値を入れる
//   msgpack_pack_int(&pk, motion_data[0].dT );
//   msgpack_pack_int(&pk, motion_data[0].accx );
//   msgpack_pack_int(&pk, motion_data[0].accy );
//   msgpack_pack_int(&pk, motion_data[0].accz );
//   msgpack_pack_int(&pk, motion_data[0].gyrox );
//   msgpack_pack_int(&pk, motion_data[0].gyroy );
//   msgpack_pack_int(&pk, motion_data[0].gyroz );
//   msgpack_pack_int(&pk, motion_data[0].magx );
//   msgpack_pack_int(&pk, motion_data[0].magy );
//   msgpack_pack_int(&pk, motion_data[0].magz );
//
//   SerialUSB.println("--- bufToString call ---");
//
//   // バッファにためた内容をString化する
//   String ret_str = bufToString(sbuf.data, sbuf.size);
//   SerialUSB.println(ret_str);
//
//   //後処理
//   msgpack_sbuffer_destroy(&sbuf);
//
//   return ret_str;
//   SerialUSB.println("--- msgpacking_end ---");
// }

// the moving function

void moving() {

  // BNO055 Sensing(9軸の値をstruct motion_data に編集)
  motion_data_t motion_data[bufCycle];
  readBno055Int16Cycle( motion_data);

  // 9軸データをpayloadに編集
  if (isMqttConnected && toSendMqtt) {
    // if ( false ) {
    toSendMqtt = false;
    char payload[256];
    int payloadSize = 0;
    for (int i = 0; i < bufCycle; i++) {
      payloadSize += snprintf(&payload[payloadSize], 150, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                              motion_data[i].dT,
                              motion_data[i].accx, motion_data[i].accy, motion_data[i].accz,
                              motion_data[i].gyrox, motion_data[i].gyroy, motion_data[i].gyroz,
                              motion_data[i].magx, motion_data[i].magy, motion_data[i].magz);
    }
    SerialUSB.print("motion_data[i].sN(シーケンス番号)(データ送信個数) : ");
    SerialUSB.println(motion_data[0].sN);
    SerialUSB.print("motion_data[i].dT : ");
    SerialUSB.println(motion_data[0].dT);

    SerialUSB.print("payload : ");
    SerialUSB.println(payload);
    SerialUSB.print("strlen(payload)");
    SerialUSB.println(strlen(payload));

    // msgpack化してmqtt publishしている間LED点灯 start
    digitalWrite(flagPin, HIGH);
    SerialUSB.println("--- msgpacking_start ---");

    // msgpack化処理
    // 初期化
    msgpack_sbuffer_init(&sbuf);
    msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);// この処理は不要？

    msgpack_pack_int(&pk, schema_motion );
    msgpack_pack_int(&pk, motion_data[0].sN );

    //時間と9軸の値を入れる
    msgpack_pack_bin(&pk, strlen(payload));
    msgpack_pack_bin_body(&pk, payload , strlen(payload) );

    // TODO Stringで一括で入れてるところをmessagepackにして送る（1箇所目)

    // 試行錯誤
    //    msgpack_pack_bin_body(&pk, (void*)motion_data[0].dT ,4 );

    // 10の配列作成
    //    msgpack_pack_array(&pk, 10);
    //    msgpack_pack_int(&pk, motion_data[0].dT );
    //    msgpack_pack_int(&pk, motion_data[0].accx );
    //    msgpack_pack_int(&pk, motion_data[0].accy );
    //    msgpack_pack_int(&pk, motion_data[0].accz );
    //    msgpack_pack_int(&pk, motion_data[0].gyrox );
    //    msgpack_pack_int(&pk, motion_data[0].gyroy );
    //    msgpack_pack_int(&pk, motion_data[0].gyroz );
    //    msgpack_pack_int(&pk, motion_data[0].magx );
    //    msgpack_pack_int(&pk, motion_data[0].magy );
    //    msgpack_pack_int(&pk, motion_data[0].magz );

    // バッファにためた内容をString化する
    //    String ret_str = bufToString(sbuf.data, sbuf.size);
    //    SerialUSB.println(ret_str);

    //後処理
    //    char buffTmp[127];
    //    snprintf(buffTmp, 127, "%02x", sbuf.data);
    //    SerialUSB.print("buffTmp : ");
    //    SerialUSB.println(buffTmp);
    SerialUSB.print("sbuf.data[i]の中身 : ");
    size_t i = 0;
    String tmp = "";
    for (; i < sbuf.size ; ++i) {
      char buffer[50];
      sprintf(buffer, "%02x", 0xff & sbuf.data[i]);
      tmp += (String)buffer;

      SerialUSB.print(" [");
      SerialUSB.print((String)buffer);
      SerialUSB.print("] ");
    }
    SerialUSB.println(" /n");


    if (isGpsData) {
      // GPSデータが「ある」場合のMQTT publish
      isGpsData = false;
      // byte * payload;
      // memcpy(payload , sbuf.data, sbuf.size);
      // memcpy(payload , gpsSbuffData, gpsSbufSize);
      SerialUSB.print("a : ");
      SerialUSB.println(sizeof(byte) * sbuf.size);

      SerialUSB.print("b : ");
      SerialUSB.println(sbuf.size);

      SerialUSB.print("strlen(sbuf.data) : ");
      SerialUSB.println(strlen(sbuf.data));

      SerialUSB.print("c : ");
      SerialUSB.println(sizeof(byte) * gpsSbufSize);

      SerialUSB.print("d : ");
      SerialUSB.println(gpsSbufSize);


      // sizeof(byte) * sbuf.size
     // byte * payload = (byte*)malloc( sbuf.size + gpsSbufSize);
     byte * payload = (byte*)malloc( sizeof(byte) * sbuf.size);

     memcpy(payload             , sbuf.data , sbuf.size);

     memcpy(payload + sbuf.size , gpsSbuffData , gpsSbufSize);

     SerialUSB.print("strlen((char *)payload) : ");
     SerialUSB.println(strlen((char *)payload));

     char yokoi[1];
     for(int i = 0; i < sizeof(payload); i++){
       //SerialUSB.print("%c", (unsigned char)payload[i]);
       sprintf(yokoi, "%c", payload[i]);
       SerialUSB.print(yokoi);
       if(payload[i] == '¥0'){
         SerialUSB.print("+++++++++++++++++++++++");
       }
     }
     SerialUSB.println("");

     SerialUSB.println(String((char *)payload));
     // SerialUSB.println(payload.c_str);



     SerialUSB.print("gpsSbuffData : (表示の仕方不明)");
     // SerialUSB.println(gpsSbuffData);
     SerialUSB.print("payloadSize : ");
     SerialUSB.println(sbuf.size + gpsSbufSize);
     SerialUSB.println("--- msgpacking_end ---");

     MqttClient.publish("/2497/data", payload , sbuf.size + gpsSbufSize );

     free(payload);
    } else {
      // GPSデータが「ない」場合のMQTT publish
      SerialUSB.print("sbuf.size : ");
      SerialUSB.println(sbuf.size);
      SerialUSB.println("--- msgpacking_end ---");

      MqttClient.publish("/2497/data", (byte*)sbuf.data, sbuf.size );
    }
    //    msgpackを自前でやったデータを遅れるかどうかテスト
    //    byte mac[]    = { 0x01, 0x01 , 0xC4, 0x02,0x03, 0x04 };
    //    MqttClient.publish("/2443/data", mac , 6);

    //    char mac[]    = { 0x01, 0x01 , 0xC4, 0x05, 0x02 };
    //    char mac[] = {1,motion_data[0].sN,0xc4,4};
    //    MqttClient.publish("/2443/data", (byte*)mac , 5);

    msgpack_sbuffer_destroy(&sbuf);

    // msgpack化してmqtt publishしている間LED点灯 end
    digitalWrite(flagPin, LOW);
  }
err:
  unsigned long next = millis();
  /*  while (millis() < next + interval_40)
    {
      MqttClient.loop();
    } */
}

bool firstFlag = true;
int moveCount = 0;
int notMoveCount = 0;
bool isMoving = false;

int getAccSum() {
  // 現在の加速度の値を取得
  motion_data_t motion_data[bufCycle];
  readBno055Int16Cycle( motion_data);

  // 加速度のxyzの合算
  int tmp;
  tmp = abs(motion_data[0].accx) + abs(motion_data[0].accy) + abs(motion_data[0].accz);
  SerialUSB.print(" (x : ");
  SerialUSB.print(motion_data[0].accx);
  SerialUSB.print(" y : ");
  SerialUSB.print(motion_data[0].accy);
  SerialUSB.print(" z : ");
  SerialUSB.print(motion_data[0].accz);
  SerialUSB.println(")");

  // TODO 構造体の初期化とかいらないのかな？？？freeとか？

  return tmp;
}

void gpsDataBuff() {
  /* GPS */
  anyGPS.read(); // read data from the GPS in the 'main loop'
  // if a sentence is received, we can check the checksum, parse it...
  if (anyGPS.newNMEAreceived()) {
    gpsData = anyGPS.lastNMEA();
    if (strncmp(gpsData + 1, "$GPRMC", 6) == 0 && anyGPS.parse(gpsData)) {

      seqNum++;
      isGpsData = true;
      char nmeaBuff_tmp[127];
      char nmeaBuff_tmp2[127];
      // nmeaSizeは使わない
      nmeaSize = snprintf(nmeaBuff_tmp, 127, "%s\n", gpsData + 1);

      SerialUSB.print("nmeaBuff_tmp(anyGPS.lastNMEA) : ");
      SerialUSB.println(nmeaBuff_tmp);
      // ----------------------- miyazaki-start --------------------------------------------------------------------------------
      String gprmcDatalist[13] = {"\0"}; // 分割された文字列を格納する配列
      // 文字列 (テスト用データ)
      // String gprmcData = "$GPRMC,000010.800,V,,,,,0.00,0.00,060180,,,N*4B";
      // 0  $GPRMC,     -
      // 1  000010.800, %02u%02u%02u.%003u  UTC TIME            str
      // 2  V,          %s                  Status              char
      // 3  ,           %10.4f              Latitude            float
      // 4  ,           %s                  Lat                 char
      // 5  ,           %10.4f              Longitude           float
      // 6  ,           %s                  Lon                 char
      // 7  0.00,       %5.1f               Speed               float
      // 8  0.00,       %5.1f               angle               float
      // 9  060180,     %02u%02u%02u        date                str
      // 10 ,           %5.1f               Magnetic Variation  float
      // 11 ,           %s                  mag                 char
      // 12 M*4B        %s                  CheckSum            str

      // 分割数 = 分割処理(文字列, 区切り文字, 配列)
      // int arraySize = (sizeof(gprmcDatalist) / sizeof((gprmcDatalist)[0])); //上の方に書くかもしれないコード
      int index = split(nmeaBuff_tmp, ',', gprmcDatalist);
      String strtmp = "";

      for (int i = 1; i < index; i++) {
        // for (int i = 0; i < index; i++) {
        //   if( gprmcDatalist[i] == "$GPRMC" ){
        //     continue;
        //   }
        SerialUSB.print("len :");
        SerialUSB.println(gprmcDatalist[i].length());
        SerialUSB.print("data :");
        SerialUSB.println(gprmcDatalist[i]);
        // strtmp = strtmp + gprmcDatalist[i];
      }

      //       char moto[4];
      //       strcpy(moto,gprmcDatalist[12].c_str());
      // //      moto = gprmcDatalist[12].c_str();
      //       char strChecksum[3];
      //       for (int i = 1; i < 4; i ++){
      //         strChecksum[i-1] += moto[i];
      //       }

      sprintf(nmeaBuff_tmp2 , "%02u%02u%02u.%003u ,%s ,%10.4f ,%s ,%10.4f ,%s ,%5.1f ,%5.1f ,%02u%02u%02u ,%5.1f ,%s , %s" ,
              anyGPS.hour , anyGPS.minute, anyGPS.seconds, anyGPS.milliseconds , gprmcDatalist[2].c_str() ,
              anyGPS.latitude , anyGPS.lat , anyGPS.longitude , anyGPS.lon ,
              anyGPS.speed , anyGPS.angle ,
              anyGPS.day , anyGPS.month , anyGPS.year ,
              // デバッグ予定。magがおかしい？（あと全体的にnullが入ってる気が。表示上の問題か？
              anyGPS.magvariation , anyGPS.mag ,
              gprmcDatalist[12].c_str() );
      // MqttClient.publish("2443/data", strtmp.c_str());

      // msgpack化処理
      // 初期化
      msgpack_sbuffer_init(&sbuf);

      // 多分この処理は不要？
      msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);

      // MOTION (2) を入れる
      msgpack_pack_int(&pk, schema_gps );

      msgpack_pack_int(&pk, seqNum );

      //GPSの値を入れる
      msgpack_pack_bin(&pk, strlen(nmeaBuff_tmp2));
      msgpack_pack_bin_body(&pk, nmeaBuff_tmp2 , strlen(nmeaBuff_tmp2) );

      // TODO Stringで一括で入れてるところをmessagepackにして送る（9軸に続き2箇所目)
      // // 12の配列作成
      // msgpack_pack_array(&pk, 12);
      // //GPSの値を入れる
      // char utctime[10];
      // sprintf(utctime,  "%02u%02u%02u.%003u", anyGPS.hour , anyGPS.minute, anyGPS.seconds, anyGPS.milliseconds);
      // msgpack_pack_str(&pk, 10);
      // msgpack_pack_str_body(&pk, utctime, 10);
      //
      // // msgpack_pack_char(&pk, gprmcDatalist[2].c_str());
      // msgpack_pack_str(&pk, 1);
      // msgpack_pack_str_body(&pk, gprmcDatalist[2].c_str() , 1);
      //
      // msgpack_pack_float(&pk, anyGPS.latitude);
      // msgpack_pack_char(&pk, anyGPS.lat);
      //
      // msgpack_pack_float(&pk, anyGPS.longitude);
      // msgpack_pack_char(&pk, anyGPS.lon);
      //
      // msgpack_pack_float(&pk, anyGPS.speed);
      // msgpack_pack_float(&pk, anyGPS.angle);
      //
      // char date[6];
      // sprintf(date,  "%02u%02u%02u", anyGPS.day , anyGPS.month, anyGPS.year);
      // msgpack_pack_str(&pk, 6);
      // msgpack_pack_str_body(&pk, date, 6);
      //
      // msgpack_pack_float(&pk, anyGPS.magvariation);
      // msgpack_pack_char(&pk, anyGPS.mag);
      //
      // msgpack_pack_str(&pk, 3);
      // msgpack_pack_str_body(&pk, gprmcDatalist[12].c_str() , 3);
      //
      // // バッファにためた内容をString化する
      // String ret_str = bufToString(sbuf.data, sbuf.size);
      // SerialUSB.println(ret_str);
      //
      // msgpack_payload = ret_str;
      // SerialUSB.print(msgpack_payload);

      // MqttClient.publish("test", "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
      // MqttClient.publish("/2443/data", msgpack_payload.c_str() );
      // MqttClient.publish("GPRMC", nmeaBuff_tmp2);
      // MqttClient.publish("GPRMC", nmeaBuff_tmp);

      // ここでは送信しない！！
      // digitalWrite(flagPin, HIGH);
      // MqttClient.publish("/2497/data", (byte*)sbuf.data, sbuf.size );
      // memcpy(gpsSbuffData, (byte*)sbuf.data , sbuf.size);

      gpsSbuffData = (byte*)sbuf.data;
      gpsSbufSize = sbuf.size;
      //後処理
      msgpack_sbuffer_destroy(&sbuf);
      // digitalWrite(flagPin, LOW);
    }
  }
}
// int sendTime_tmp = 0;






// the loop function

void loop() {
  if (firstFlag) {
    // deltaTの値が初回だけおかしいのでスキップ。
    firstFlag = false;
    return;
  }

  // GPSのデータはいつ読み込めるか不明なので、とりあえず常に読んでためておく
  gpsDataBuff();

  int elapsedTimeSinceLastTime_40 = millis() - motion_ts_40;
  int elapsedTimeSinceLastTime_1000 = millis() - motion_ts_1000;

  // interval_40 の時間が経過したかどうか。経過してない場合はスキップ。
  if (elapsedTimeSinceLastTime_40 < interval_40) {
    return;
  }

  // 間隔(deltaTを保持、大体40msecはハズ）
  deltaT = elapsedTimeSinceLastTime_40;
  // 前回時間を退避
  motion_ts_40 = millis();
  SerialUSB.print("■motion_ts_40 : ");
  SerialUSB.println(motion_ts_40);

  // interval_1000 の時間が経過したかどうか。1秒毎に動きのチェックをする
  if (elapsedTimeSinceLastTime_1000 >= interval_1000) {

    // 前回時間を退避
    motion_ts_1000 = millis();
    SerialUSB.print("■motion_ts_1000 : ");
    SerialUSB.println(motion_ts_1000);

    //check
    // 加速度のxyzの合算
    int tmp_acc;
    tmp_acc = getAccSum();

    SerialUSB.print("tmp_acc");
    SerialUSB.println(tmp_acc);

    // 差分算出
    int diff;
    diff = tmp_acc - before_acc;

    SerialUSB.print("before_acc : ");
    SerialUSB.print(before_acc);
    SerialUSB.print(" after_acc : ");
    SerialUSB.print(tmp_acc);
    SerialUSB.print(" diff(abs) : ");
    SerialUSB.println(abs(diff));

    // 動いているか動いていないか
    if (  abs(diff)  >= 10 ) {
      moveCount ++;
      SerialUSB.print( "move : ");
      SerialUSB.println(moveCount);
      notMoveCount = 0;
    } else {
      notMoveCount ++;
      SerialUSB.print( "notMove : ");
      SerialUSB.println(notMoveCount);
      moveCount = 0;
    }

    // 動き出したことの検知
    if ( !isMoving && (moveCount >= 3)) {
      isMoving = true;
      SerialUSB.println( "moveing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      SerialUSB.println( "開始処理");

      SerialUSB.println( "" );
      SerialUSB.println( "call - claimingAndCreateSession" );
      claimingAndCreateSession();

      SerialUSB.println( "" );
      SerialUSB.println( "call - beginBulkuploader" );
      beginBulkuploader();

      // /* msgpack::sbuffer is a simple buffer implementation. */
      // msgpack_sbuffer_init(&sbuf);
      // /* serialize values into the buffer using msgpack_sbuffer_write callback function. */
      // msgpack_packer_init(&pk, &sbuf, msgpack_sbuffer_write);
    }
    // 止まっていることの検知
    if ( isMoving && (notMoveCount >= 5 ) ) {
      isMoving = false;
      SerialUSB.println( "あれれ？終わりましたか？");
      SerialUSB.println( "終了処理");

      SerialUSB.println( "" );
      SerialUSB.println( "終了のmqttデータをpublish");
      MqttClient.publish("/2497/stop", apiKey );

      // なんとなく1分くらい停止
      delay( 60 * 1000);
    }
    before_acc = tmp_acc;
  }

  if ( isMoving ) {
    SerialUSB.println( "動いている時の処理。データを送るとか。");

    // int hoge = millis();
    // SerialUSB.print( "■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■");
    // SerialUSB.print( "現在時間1 : ");
    // SerialUSB.println(hoge);
    moving();

    // int foo = millis();
    // SerialUSB.print( "■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■");
    // SerialUSB.print( "現在時間2 : ");
    // SerialUSB.println(foo);
    //
    // int bar =  foo - hoge;
    // SerialUSB.print( "■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■");
    // SerialUSB.print( "送信にかかる時間(msec) : ");
    // SerialUSB.println(bar);
    //
    // int aaa = foo - sendTime_tmp;
    // SerialUSB.print( "■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■");
    // SerialUSB.print( "前回送信してからの時間(msec) : ");
    // SerialUSB.println(aaa);
    // sendTime_tmp = foo;

    /* test code 複数件登録確認　OK
      byte mac[]    = { 0x01, 0x01 , 0xC4, 0x02,0x05, 0x04, 0x01, 0x02 , 0xC4, 0x02,0x05, 0x06 };
      MqttClient.publish("/2497/data", mac , 12);
      isMoving = false;
      delay(3000);
      MqttClient.publish("/2497/stop", apiKey );
      delay(3000);*/
  }
}
