#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <credentials.h>

#include <BME280I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <LiquidCrystal_PCF8574.h>

#include <ArduinoJson.h>

#include <TimeLib.h>
#include <EasyNTPClient.h>
#include <WiFiUdp.h>


/*-----------------------------------------------------------------------------------*/
// Sensoren

/* Setting Sttring for BME280 read out */
BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x77
);

BME280I2C bme(settings);


// Data wire is connected to GPIO32
#define ONE_WIRE_BUS 32
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

DeviceAddress sensor2 = { 0x28, 0x74, 0xD3, 0x45, 0x92, 0x19, 0x02, 0xAE};
DeviceAddress sensor1 = { 0x28, 0xB8, 0x82, 0x45, 0x92, 0x02, 0x02, 0xDB};


/*-----------------------------------------------------------------------------------*/
// WIFI

/* initialize WiFIMulti */
WiFiMulti wifiMulti;

/* create an instance of PubSubClient client */
WiFiClient WlanClient;


/*-----------------------------------------------------------------------------------*/
// MQTT

/* initialize PubSubClient */
PubSubClient MQTTClient(WlanClient);
const char* mqtt_server = "broker.hivemq.com";

/* MQTT device name*/
const String device_name = "TempLogger_001";

/* topics */
String STAT_TOPIC  = "/devices/" + device_name + "/status";
//String TEMP_TOPIC  = "/ambient/" + device_name + "/temperature";
//String RH_TOPIC    = "/ambient/" + device_name + "/humidity";
//String PRES_TOPIC  = "/ambient/" + device_name + "/pressure";
//String RPRES_TOPIC = "/ambient/" + device_name + "/reduced_pressure";

String LOG_AMBIENT_TOPIC = "/logger/" + device_name + "/ambient";
String LOG_TEMP_TOPIC    = "/logger/" + device_name + "/temp_sensor";


/*-----------------------------------------------------------------------------------*/
// initialize JSON

const size_t capacity = JSON_OBJECT_SIZE(5) + 100;
DynamicJsonDocument JSON_ambient(capacity);
DynamicJsonDocument JSON_temp(capacity);


/*-----------------------------------------------------------------------------------*/
// LCD

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display
int degree[8] = {0x7,0x5,0x7,0x0,0x0,0x0,0x0,0x0};


/*-----------------------------------------------------------------------------------*/
// NTP time synconisation

WiFiUDP udp;
const int timeZone = 2*60*60;
const char *ntpServer = "pool.ntp.org";
EasyNTPClient ntpClient(udp, ntpServer, timeZone);


/*-----------------------------------------------------------------------------------*/
// zusätzliche Variabeln

/* barometrische Höhenformel */
const float g = 9.80665;
const float R = 287.05;
const float alpha = 0.0065;
const float C_h = 0.12;
const float h = 465;           // Change to your height above seelevel

/* Antonie Parameter */
const float A = 5.20389;
const float B = 1733.926;
const float C = 39.485;


long lastMsgSense = 0;
long lastMsgTime = 0;

/* Pin Definitions */
const int selectPin = 25;     // this pin is for selecting the data dispalyed by the LCD


// +++++++++++++++++++++++++ Time Conversion Functions +++++++++++++++++++++++++
String printDigits(int digits){
  // utility function for digital clock display
  if(digits < 10){
    return ":0" + String(digits);
  }
  else{
    return ":" + String(digits);
  }
}

String digitalClockDisplay(time_t t){
  String tString;

  // digital clock display of the time and date
  
  tString = String(day(t));
  tString += "." + String(month(t));
  tString += "." + String(year(t));
    
  tString += " " + String(hour(t));
  tString += printDigits(minute(t));
  tString += printDigits(second(t));

  return tString;
}

String timeDisplay(time_t t){
  String tString;

  // digital clock display of the time
  
  tString += String(hour(t));
  tString += printDigits(minute(t));
  tString += printDigits(second(t));

  return tString;
}

// +++++++++++++++++++++++++ WiFi Setup +++++++++++++++++++++++++
void setup_wifi() {
    delay(10);

    Serial.println();
    Serial.print("Wait for WiFi... ");

    /* Wait for the connection to establisch */
    while(wifiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}


// +++++++++++++++++++++++++ MQTT Callback +++++++++++++++++++++++++
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
  Serial.print((char)payload[i]);
  }
  Serial.println();
}


// +++++++++++++++++++++++++ MQTT Reconnect +++++++++++++++++++++++++
void reconnect() {
  // Loop until we're reconnected
  while (!MQTTClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MQTTClient.connect(device_name.c_str(),STAT_TOPIC.c_str(),1,1,"disconnected")) {
      Serial.println("connected");
      Serial.println();
      // Once connected, publish an announcement...
      MQTTClient.publish(STAT_TOPIC.c_str(), "connected",true);
      // ... and resubscribe
      // MQTTClient.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// ++++++++++++++++++++++ NTP function wrapper ++++++++++++++++++++++
time_t getNTPtime(){
  return(ntpClient.getUnixTime());
}

// +++++++++++++++++++++++++ BME280 Read Data +++++++++++++++++++++++++
int readBME280Data(float &pres, float &temp, float &hum) {

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  while (!((pres > 80000 && pres < 110000) && (temp > -30 && temp < 60) && (hum > 0 && hum <100))) {
    Serial.println("Sensor read faild");
    delay(500);
    bme.read(pres, temp, hum);
  }
}



// +++++++++++++++++++++++++ Program Start +++++++++++++++++++++++++
void setup() {

  // ----------------------------- Basic -----------------------------
  Serial.begin(115200);
  Serial.println();

  // wait on Serial to be available on Leonardo
  while (!Serial);

  Wire.begin();


  // --------------------------- Pin Setup ---------------------------
  pinMode(selectPin, INPUT_PULLDOWN);


  // ------------------------------ LCD ------------------------------
  int error;

  // check if LCD is connected
  Serial.println("Check for LCD");
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(" LCD found.");
    lcd.begin(20, 4); // initialize the lcd

  } else {
    Serial.println(" LCD not found.");
  }

  // start LCD
  lcd.setBacklight(255);
  lcd.setCursor(0, 1);
  lcd.print("Starting");
  lcd.setCursor(0, 2);
  lcd.print("esp32 Data-Logger");

  // add custom character
  lcd.createChar(0, degree);


  // ------------------------------ BME280 ------------------------------
  while(!bme.begin())
  {
    Serial.println("Could not find BME280I2C sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }

  Serial.println();


// ------------------------------ DS18B20 ------------------------------
sensors.setResolution(sensor1, 12);
sensors.setResolution(sensor2, 12);

Serial.println("Sensor 1: " + String(sensors.getResolution(sensor1), DEC) + " bit");
Serial.println("Sensor 2: " + String(sensors.getResolution(sensor2), DEC) + " bit");

// ------------------------------ WiFi ------------------------------

  /* Defining the AP's to connect to */
    wifiMulti.addAP(ssid, password);
  // more AP can be added
  
  setup_wifi();
  Serial.println();


// ------------------------------ MQTT ------------------------------
  MQTTClient.setServer(mqtt_server, 1883);
  MQTTClient.setCallback(callback);
  // MQTTClient is now ready for use


  // ------------------------- Syncronize Tine -------------------------
  Serial.println("Syncing system time to NTP");
  setSyncProvider(getNTPtime);

  if (timeStatus() != timeSet){
    Serial.println("Syncronisatrion FAILED!");
    // lcd.print("FAILED");
    while(1);
  }

  Serial.println("Syncronisatrion SUCCESSFUL!");
  Serial.println(digitalClockDisplay(now()));
  Serial.println();


  //Prepare LCD
  lcd.clear();
  lcd.home();
}

void loop() {

// ------------------------------ WiFi Reconnect ------------------------------
  if(wifiMulti.run() != WL_CONNECTED) {
        Serial.println("WiFi not connected!");
        setup_wifi();
  }
// ------------------------------ MQTT Reconnect ------------------------------
  if (!MQTTClient.connected()) {
    reconnect();
  }

// ------------------------------ MQTT Handler ------------------------------
  MQTTClient.loop();

// ------------------------------ Non Blocking BME & OneWire Read ------------------------------
  if ((millis() - lastMsgSense > 10000) || (millis()-lastMsgSense < 0)) {     // intervall 10 seconds
    lastMsgSense = millis();
    
    float temp(NAN), hum(NAN), pres(NAN);
    readBME280Data(pres, temp, hum);

    float p_s(NAN), E(NAN), p_r(NAN);
    p_s = pow(10, A-B/(C+temp));
    E = hum/100* p_s;

    p_r = pres/100 * exp(g*h/(R*(temp+273.15+C_h*E+alpha*h/2)));

    Serial.println();
    Serial.println("temperature in °C = " + String(temp));
    Serial.println("relatve humidity in % = " + String(hum));
    Serial.println("pressure in mbar = " + String(pres/100));
    Serial.println("reduced pressure in mbar = " + String(p_r));


    // Display on LCD
    if (digitalRead(!selectPin)){
      lcd.setCursor(0,3);
      lcd.print("redPres: "+ String(p_r)+"mbar   ");
      lcd.setCursor(0,2);
      lcd.print("Hum:     " + String(hum) + "%   ");
      lcd.setCursor(0,1);
      lcd.print("Temp:    " + String(temp));
      lcd.write(uint8_t(0));
      lcd.print("C   ");
      lcd.setCursor(0,0);
      lcd.print("                    ");
      lcd.setCursor(0,0);
    }


    //Concstuct JSON
    JSON_ambient["time"] = now();
    JSON_ambient["temperature"] = temp;
    JSON_ambient["humidity"] = hum;
    JSON_ambient["pressure"] = pres/100;
    JSON_ambient["reduced pressure"] = p_r;

    char buffer[512];
    size_t n = serializeJson(JSON_ambient, buffer);

    MQTTClient.publish(LOG_AMBIENT_TOPIC.c_str(), buffer, n);

    Serial.println();


    //OneWire
    sensors.requestTemperatures(); // Send the command to get temperatures

    float tempSens_1, tempSens_2;

    sensors.requestTemperatures();
    tempSens_1 = sensors.getTempC(sensor1);
    tempSens_2 = sensors.getTempC(sensor2);

    Serial.println("Sensor 1 in °C: " + String(tempSens_1));
    Serial.println("Sensor 2 in °C: " + String(tempSens_2));

    if (digitalRead(selectPin)){
      lcd.setCursor(0,3);
      lcd.print("Sensor 2: " + String(tempSens_2));
      lcd.write(uint8_t(0));
      lcd.print("C   ");
      lcd.setCursor(0,2);
      lcd.print("Sensor 1: " + String(tempSens_1));
      lcd.write(uint8_t(0));
      lcd.print("C   ");
      lcd.setCursor(0,1);
      lcd.print(digitalClockDisplay(now()) + "  ");
      lcd.setCursor(0,0);
      lcd.print("                    ");
      lcd.setCursor(0,0);
    }

    JSON_temp["time"] = now();
    JSON_temp["sensor 1"] = tempSens_1;
    JSON_temp["sensor 2"] = tempSens_2;

    n = serializeJson(JSON_temp, buffer);
    
    MQTTClient.publish(LOG_TEMP_TOPIC.c_str(), buffer);
    Serial.println();
  }


// ------------------------------ Non Blocking Time ------------------------------
  if ((millis() - lastMsgTime > 1000) || (millis()-lastMsgTime < 0)) {     // intervall 1 seconds
    lastMsgTime = millis();
    Serial.println(digitalClockDisplay(now()));
    lcd.print(">>");
  }
}