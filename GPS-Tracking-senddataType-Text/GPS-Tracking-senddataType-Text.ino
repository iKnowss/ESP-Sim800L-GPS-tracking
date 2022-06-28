//* import Sim lobrary
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include <string.h>

#define RXPin (22)
#define TXPin (21)
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

HardwareSerial ss(2);
//? Select your modem sim:
#define TINY_GSM_MODEM_SIM800 

//* Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
// set GSM PIN, if any
#define GSM_PIN ""

//TODO: Set sim card 
const char apn[] = "internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "true"; //? sim true
const char gprsPass[] = "true"; //? sim true

const char simPIN[]   = ""; // SIM card PIN (leave empty, if not defined)

//TODO: Set detail MQTT broker
const char* broker = "mqtt.rfidpatient.com";//! Public IP address or domain name
const char* mqttUsername = "gpsclient";  //! MQTT username
const char* mqttPassword = "1q2w3e4r5t";  //! MQTT password

//TODO: set port MQTT
#define MQTT_Port 1883

//TODO: Set Topic MQTT
const char* GPStopic = "gpstopic";

//TODO: time send data
int time_send =  3000; //? send data every 3s

//TODO: Set board ID
int boardID = 1;


#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>

//! set client
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

uint32_t lastReconnectAttempt = 0;
TwoWire I2CPower = TwoWire(0);  


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < len; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  //? If a message is received on the topic gpstopic, you check if the message is either "true" or "false". 
  if (String(topic) == "gpstopic") {
    Serial.print("Changing output to ");
    if(messageTemp == "true"){
      Serial.println("true");
    }
    else if(messageTemp == "false"){
      Serial.println("false");
    }
  }
  else {
    Serial.println("no topic");
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  //? Connect to MQTT Broker without username and password
  //* boolean status = mqtt.connect("GsmClientN");

  //? if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(GPStopic);

  return mqtt.connected();
}


void setup() {

  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  Serial.println(TinyGPSPlus::libraryVersion());
  
  SerialMon.begin(115200);
  
  I2CPower.begin(IP5306_REG_SYS_CTL0 ,I2C_SDA, I2C_SCL, 400000);
  
  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  
  SerialMon.println("Wait...");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }
  
  //MQTT Broker setup
  mqtt.setServer(broker, MQTT_Port);
  mqtt.setCallback(mqttCallback);
}

void loop() {

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 3 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 3000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { //! reconnect
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  else{

  float lat = 0; //! long
  float lon = 0; //! lat

  long lastMsg = 0;
  long now = millis();

   SerialMon.println("Requesting current GSM location");
   if (modem.getGsmLocation(&lat, &lon))
    {
      // send data evert 3s
      if (now - lastMsg > time_send) {
        lastMsg = now;
        
        SerialMon.println("=== MQTT CONNECTED ===");
        // Convert the value to a char array
        String latString = String(lat, 20);
        String longString = String(lon, 20);
        SerialMon.println(latString);
        SerialMon.println(longString);

        String pipeline = "|";

        //! fomat data for send
        String data_local = boardID + pipeline + longString + pipeline + latString; //! example DATA: 1|100.13|13.33
        SerialMon.println(data_local);

        char local[50];
        data_local.toCharArray(local, 50);

        //! send data to MQTT topic GPStopic
        mqtt.publish(GPStopic, local);
        mqtt.loop();
        }
      else{
        ;
        }
      }
    }
}