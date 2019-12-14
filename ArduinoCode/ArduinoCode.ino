#include <ESP8266WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Watson IoT connection details
#define MQTT_HOST "whyeub.messaging.internetofthings.ibmcloud.com"
#define MQTT_PORT 1883
#define MQTT_DEVICEID "d:whyeub:ESP8266:dev01"
#define MQTT_USER "use-token-auth"
#define MQTT_TOKEN "TatianaSaraJose12345"
#define MQTT_TOPIC "iot-2/evt/status/fmt/json"
#define MQTT_TOPIC_DISPLAY "iot-2/cmd/display/fmt/json"
#define MQTT_TOPIC_RESET_W "iot-2/cmd/resetW/fmt/json"
#define MQTT_TOPIC_INTERVAL "iot-2/cmd/interval/fmt/json"

// Add GPIO pins used to connect devices
#define RGB_PIN D1        // GPIO pin the data line of RGB LED is connected to
#define RFID_RX_PIN D7    // GPIO pin RFID Board RX
#define RFID_TX_PIN D8    // GPIO pin RFID Board TX
#define FORCE1_ACT_PIN D3  // GPIO pin Force Sensor 1 Activation
#define FORCE2_ACT_PIN D4 // GPIO pin Force Sensor 2 Activation
#define WET_RESET_PIN  D2  // GPIO pin Wet Reset Button

// Specify DHT11 (Blue) or DHT22 (White) sensor
#define NEOPIXEL_TYPE NEO_RGB + NEO_KHZ800


// Add WiFi connection information
char ssid[] = "SIGH";     //  your network SSID (name)
char pass[] = "aguapanela";  // your network password

//Variables for the Activity
#define hSizeForce 5
#define REC_DAT_TRIGGER 2
#define ACTIVE_THRESH 1000
#define OUTOFRANGE_THRESH 1500
short lastValueForce1, lastValueForce2;
short hDiffForce1[hSizeForce], hDiffForce2[hSizeForce];

//Variables for Force Sensor
bool outOfRange = false;
#define MIN_FORCE_VALUE 20

//RFID Board Variables
SoftwareSerial RFID_Read(RFID_RX_PIN, RFID_TX_PIN); //RFID_Read(Rx, Tx)

//NeoPixel Configuration
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, RGB_PIN, NEOPIXEL_TYPE);
//LED State
#define S_NORMAL 0
#define S_ACTIVE 1
#define S_OUTOFRANGE 2
#define S_ALARMED 3

// MQTT objects
void callback(char* topic, byte* payload, unsigned int length);
WiFiClient wifiClient;
PubSubClient mqtt(MQTT_HOST, MQTT_PORT, callback, wifiClient);

//Adafruit MQTT
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   
#define AIO_USERNAME    "jmontoya_mx"  
#define AIO_KEY         "b9ad15b1bcce4554afa0c088435ea2d3" 
Adafruit_MQTT_Client mqttAdaf(&wifiClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe nappyReset = Adafruit_MQTT_Subscribe(&mqttAdaf, AIO_USERNAME "/feed/nappyreset");
void MQTT_connect();

// variables to hold data
StaticJsonDocument<100> jsonDoc;
JsonObject payload = jsonDoc.to<JsonObject>();
JsonObject status = payload.createNestedObject("d");
static char msg[50];
StaticJsonDocument<100> jsonReceiveDoc;
int32_t ReportingInterval = 10;                       // Reporting Interval seconds
bool Wetness=false;                                   //The holding value of wetness in the system

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");
  
  payload[length] = 0; // ensure valid content is zero terminated so can treat as c-string
  Serial.println((char *)payload);
  
  DeserializationError err = deserializeJson(jsonReceiveDoc, (char *)payload);
  if (err) {
    Serial.print(F("deserializeJson() failed with code ")); 
    Serial.println(err.c_str());
  } else {
    JsonObject cmdData = jsonReceiveDoc.as<JsonObject>();
    if (0 == strcmp(topic, MQTT_TOPIC_RESET_W)) {
      Wetness = false;
      Serial.println("The wetness was reset");
    } else if (0 == strcmp(topic, MQTT_TOPIC_INTERVAL)) {
      //Valid message received
      ReportingInterval = cmdData["Interval"].as<int32_t>(); // this form allows you specify the type of the data you want from the JSON object
      Serial.print("Reporting Interval has been changed:");
      Serial.println(ReportingInterval);
      jsonReceiveDoc.clear();
    } else {
      Serial.println("Unknown command received");
    }
  }
}

void setup() {  
  //Set PinMode
  pinMode(FORCE1_ACT_PIN, OUTPUT);
  pinMode(FORCE2_ACT_PIN, OUTPUT);
  pinMode(WET_RESET_PIN,INPUT);
  
 // Start serial console
  Serial.begin(115200);
  Serial.setTimeout(2000);
  while (!Serial) { }
  Serial.println();
  Serial.println("ESP8266 Sensor Application");

  //Configure RFID Module
  RFID_Read.listen();
  RFID_Read.begin(9600); // set the data rate for the SoftwareSerial port
  delay(10);
  RFID_Read.write(0x02); //Send the command to read RFID tag, please refer to the manual for more detail.
  Serial.println();
  Serial.println("RFID Module Activated");

  // Start WiFi connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");

  // Start connected devices
  pixel.begin();

  while(! mqtt.connected()){
    if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) { // Token Authentication
      Serial.println("MQTT Connected");
      mqtt.subscribe(MQTT_TOPIC_DISPLAY);
      mqtt.subscribe(MQTT_TOPIC_RESET_W);
    } else {
      Serial.println("MQTT Failed to connect! ... retrying");
      delay(500);
    }
  }

  //Initialize the Pressure Sensor memory value
  lastValueForce1=getForceSensorValue(FORCE1_ACT_PIN);
  lastValueForce2=getForceSensorValue(FORCE2_ACT_PIN);
  for(byte i=0;i<hSizeForce;i++){
    hDiffForce1[i]=0;
    hDiffForce2[i]=0;
  }
  updateDiffPressureArray();
  Serial.println("Force sensor initialized, initial values: "+String(lastValueForce1)+","+String(lastValueForce2));

  //Adafruit MQTT Start
  mqttAdaf.subscribe(&nappyReset);
}

void loop() {
  MQTTAda_connect();
  mqtt.loop();
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) {
      Serial.println("MQTT Connected");
      mqtt.subscribe(MQTT_TOPIC_DISPLAY);
      mqtt.loop();
    } else {
      Serial.println("MQTT Failed to connect!");
      delay(5000);
    }
  }

  //Read RFID Module
  bool IsWet = getWetValue();
  Serial.println("Wetness: "+ String(IsWet));
  
  //Set the LED Color
  byte stateValue = getStatusNNModel();
  if (outOfRange){
    stateValue = S_OUTOFRANGE;
  }
  setLEDState(stateValue);
  Serial.println("The baby movement status is: "+ String(stateValue));

  // Send data to Watson IoT Platform
  status["wet"] = IsWet;
  status["stat"] = stateValue;
  serializeJson(jsonDoc, msg, 50);
  Serial.println(msg);
  if (!mqtt.publish(MQTT_TOPIC, msg)) {
    Serial.println("MQTT Publish failed");
  }
  else{
    Serial.println("MQTT Publish succeed");
  }

  // Pause - but keep polling MQTT for incoming messages
  for (byte i = 0; i < ReportingInterval; i++) {
    mqtt.loop();
    updateDiffPressureArray();
    if(Wetness){
      if(digitalRead(WET_RESET_PIN)==HIGH){
        Serial.println("Wetness Reseted");
        Wetness = false;
      }
    }
    if(!Wetness && getWetValue()){
      setLEDState(stateValue);
      break;
    }

    if(mqttAdaf.connected()){
      Adafruit_MQTT_Subscribe *subscription;
      while ((subscription = mqttAdaf.readSubscription(5000))) {
            // to test the tx line data
        Serial.print(F("Received Something: "));
        if (subscription == &nappyReset) {
          Serial.print(F("Got Reset Received from Assistant: "));
          Serial.println((char *)nappyReset.lastread);
          uint16_t state = atoi((char *)nappyReset.lastread);
          Serial.println(state);
          Serial.println("Wetness Reseted");
          Wetness = false;
        }
      }
    }
    else{
      MQTTAda_connect();
    }

    delay(1000);
  }
}

//Functions
short getForceSensorValue(unsigned short pin)
{
  digitalWrite(FORCE1_ACT_PIN,LOW);
  digitalWrite(FORCE2_ACT_PIN, LOW);
  digitalWrite(pin, HIGH);
  delay(30);
  short analogValue = analogRead(A0);
  digitalWrite(FORCE2_ACT_PIN, LOW);
  delay(30);
  return analogValue;
}

void pushFILOArray(short FiloArray[], short newValue){
  for(byte a =(hSizeForce-1);a>0;a--){
    FiloArray[a]=FiloArray[a-1];
  }
  FiloArray[0] = newValue;
}

void printArray(short Array[]){
  for(byte j =0;j<hSizeForce;j++){
    Serial.print(Array[j]);
    Serial.print(",");
  }
}

short updateDiffPressureArray(){
  //Read Pressure Sensor  
  short ForceSensor1=getForceSensorValue(FORCE1_ACT_PIN);
  short ForceSensor2=getForceSensorValue(FORCE2_ACT_PIN);

  //Check if the baby is in reading position
  int sumValues = SumArray(hDiffForce1)+SumArray(hDiffForce2);
  outOfRange = ForceSensor1<=MIN_FORCE_VALUE&&ForceSensor2<=MIN_FORCE_VALUE && sumValues<OUTOFRANGE_THRESH;
  
  //Get the difference
  short difSensor1 = abs(ForceSensor1-lastValueForce1);
  short difSensor2 = abs(ForceSensor2-lastValueForce2);

  //Push and prshort the arrays
  pushFILOArray(hDiffForce1,difSensor1);
  pushFILOArray(hDiffForce2,difSensor2);
  printArray(hDiffForce1);
  printArray(hDiffForce2);
  
  //Save the force value
  lastValueForce1=ForceSensor1;
  lastValueForce2=ForceSensor2;

  //Print out of range
  if(outOfRange){
    Serial.println("true");
  }
  else{
    Serial.println("false");
  }
}

byte getStatusNNModel(){
  int totalSum = SumArray(hDiffForce1)+SumArray(hDiffForce2);
  Serial.println("Total Dif Sum: "+ String(totalSum));
  if(totalSum>=ACTIVE_THRESH){
    return S_ACTIVE;
  }
  return S_NORMAL;
}

int SumArray(short Array[]){
  int sum =0;
  for(byte j =0;j<hSizeForce;j++){
    sum+=Array[j];
  }
  return sum;
}

bool getWetValue()
{
  if(digitalRead(WET_RESET_PIN)==HIGH){
    Serial.println("Wetness Reseted");
    Wetness = false;
  }
  if(Wetness){
    return Wetness;
  }
 bool writeInSerial = false;        //Axilizar variable to organize the Serial Printing
 byte receivedDataCount =0;         //Counts the number of data received

 while (RFID_Read.available()) {  //The sensor detected something
  byte C = RFID_Read.read();
  if(!writeInSerial){             //It is the first time that will write in the serial
    Serial.print("RFID sensor receive: ");
  }
  writeInSerial = true;
  if (C<10) {
    Serial.print("0");
  }
  else{
    Serial.print(C ,HEX);         //Display in HEX
    receivedDataCount++;
  }
  Serial.print(" ");
 }

 if(writeInSerial){
   Serial.println();
}
  
 if (receivedDataCount>=REC_DAT_TRIGGER) {
  Serial.println("RFID Tag Recognized Data Received: " + String(receivedDataCount));
  Wetness = true;
  return true;
 }
 return false;
}

void setLEDState(byte state)
{
  unsigned char r = 0,g = 0,b = 0;
  if(Wetness){
    state=S_ALARMED;
  }
  switch(state)
  {
  case S_NORMAL:
    g=255;
    break;
  case S_OUTOFRANGE:
    r=255;
    g=255;
    break;
  case S_ACTIVE:
    b=255;
    break;
  case S_ALARMED:
    r=255;
    break;
  default:
    r=255;
    g=255;
    b=255;
    break;
  }
  pixel.setPixelColor(0, r, g, b);
  pixel.show();
}

void MQTTAda_connect() {
  int8_t ret;
  
  // Stop if already connected.
  if (mqttAdaf.connected()) {
    Serial.println("Conection with Assistant Stable");
    return;
  }

  Serial.print("Connecting to MQTT Assistant... ");

  uint8_t retries = 3;
  if((ret = mqttAdaf.connect()) != 0) {
       Serial.println(mqttAdaf.connectErrorString(ret));
       Serial.println("Retrying MQTT Assistant connection next cycle  seconds...");
       mqttAdaf.disconnect();
  }
  else{
    Serial.println("MQTT Assistant Connected!");
    mqttAdaf.subscribe(&nappyReset);
  }
}
