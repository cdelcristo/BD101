/*
Fermentor autoMation Project. 

Automation will provided control for:
    → Heater
    → Indoor Fan

Collect the following data:
    → Indoor temperature
    → Indoor humidity
    → Outdoor temperature
    → Outdoor humidity

Develop by: → Carlos Delcristo ← 
Date: 08/05/2020

Board: ESP32 DevKit

Connections:

*/

// Libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "InfluxArduino.hpp"
#include <SPI.h>
#include <Adafruit_BME280.h>


#define Sprintln(a) (Serial.println(a))

// Define analog inputs
#define INDOORLIGHTSENSOR 32 //LDR to measure indoor light value


// Define digital outputs
#define LEDBLUE 2         //WIFI indicator
#define HEATER_1 25       // Heater #1 connected to GPIO25
#define COOLER_1 26       // Cooler #1 connected to GPIOXX


// Object Creation
InfluxArduino influx;
Adafruit_BME280 bme; // I2C pin used are 22, 21
WiFiClient espClient;
PubSubClient client(espClient);


//Constant declaration
//Constants
unsigned long CHECKWIFI = 300;
unsigned long CHECKINPUTSINTERVAL = 1000;
unsigned long checkInputsPrevious = 0;
unsigned long CHECKOUTPUTSINTERVAL = 2000;
unsigned long checkOutputsPrevious = 0;
unsigned long UPDATEDBINTERVAL = 6000;
unsigned long updateDBPrevious = 0;
unsigned long coolerBlockTime = 300000;

int heater_1_status = 0;
int cooler_1_status = 0;

float indoorTemp = 0;
float indoorHumdity = 0;
float indoorPressure = 0;
float indoorLight = 0;

float outdoorTemp = 0;
float outdoorHumdity = 0;
float outdoorPressure = 0;

float fermentationSetPoint_1 = 19;
float fermentationPV = 0;
float fermentationThreshold_1 = 0.5;
int fermentationPVSource = 0; // source of SP for fermentor

bool coolerBlocked = true;


char tags[64];
char fields[512];


//WIFI
const char *WIFI_NAME = WIFI_SSID;
const char *WIFI_PASS = WIFI_PASSWORD;

//InfluxDB
const char *INFLUX_DATABASE = INFLUX_DB;
const char *INFLUX_IP = INFLUX_SERVER;
const char INFLUX_USER[] = "";                       //username if authorization is enabled.
const char INFLUX_PASS[] = "";                       //password for if authorization is enabled.
const char *INFLUX_MEASUREMENT_1 = INFLUX_MEASURE_1; //measurement name for the database. (in practice, you can use several, this example just uses the one)

//Mosquitto
const char mosquittoServer[] = MOSQUITTO_SERVER;
const int mosquittoServerPort = MOSQUITTO_SERVER_PORT;
const char outTopic[] = "/garden/sensor1";
const char inTopic[] = "/garden/#";

// Prototype Functions
void connectWifi();
void reconnectWifi();
void reconnectBroker();
void callback(char *topic, byte *payload, unsigned int length);
void influxConf();
void bmeConf();
void mosquittoConf();
void readInputs();
void updateOutputs();
void publish2DB();

// SETUP
void setup()
{
  pinMode(LEDBLUE, OUTPUT);
  pinMode(HEATER_1, OUTPUT);
  pinMode(COOLER_1, OUTPUT);
  digitalWrite(HEATER_1, HIGH); // apaga heater 1
  digitalWrite(COOLER_1, HIGH); // apaga heater 1
  Serial.begin(115200);
  Sprintln("Setup Begins...");
  delay(500); //delay for better read at console
  // ESP Configurations
  connectWifi();
  influxConf();
  bmeConf();
  mosquittoConf();
}

//MAIN LOOP
void loop()
{
  // main code:
  // Check inputs status
  unsigned long startTime = millis();
  if (startTime >= checkInputsPrevious)
  {
    readInputs();
    checkInputsPrevious = startTime + CHECKINPUTSINTERVAL;
  }
  // Check outputs status
  if (startTime >= checkOutputsPrevious)
  {
    updateOutputs();
    checkOutputsPrevious = startTime + CHECKOUTPUTSINTERVAL;
  }
  // Saving data into InfluxDB
if (startTime >= updateDBPrevious)
  {
    publish2DB();
    updateDBPrevious = startTime + UPDATEDBINTERVAL;
  }


 
//   sprintf(tags, "Location=indoorGarden,What=Tray-1"); //write a tag called
//   sprintf(fields, "temp_probe1=%f,temp_probe2=%f,temp_probe3=%f,temp_probeAVG=%f,indoorTemp=%f,indoorHumdity=%f,indoorPressure=%f,VPD=%f,moisture_1=%i,heater1_stat=%i,fermentationSetPoint_1=%f,fermentationPV=%f",
//           temp_probe1, temp_probe2, temp_probe3, temp_probeAVG, indoorTemp, indoorHumdity, indoorPressure, indoorVPD, moisture_1, heater_1_status, fermentationSetPoint_1, fermentationPV);
//   bool writeSuccessful = influx.write(INFLUX_MEASUREMENT_1, tags, fields);
//   if (!writeSuccessful)
//   {
//     Sprintln("error: ");
//     Sprintln(influx.getResponse());
//   }

  Sprintln("Temperature = ");
  Sprintln(indoorTemp);
  Sprintln(" °C");

  Sprintln("Humidity = ");
  Sprintln(indoorHumdity);
  Sprintln(" %");

  Sprintln("Pressure = ");
  Sprintln(indoorPressure);
  Sprintln(" hPa");

  Sprintln("Light Value: ");
  Sprintln(indoorLight);
  Sprintln(" lux");

  if (!client.connected())
  {
    reconnectBroker();
  }
  client.loop();
  //delay(2000);
}

























































// Functions

// Read and inputs function

void readInputs()
{

  Sprintln("Reading inputs....");

  sensors.requestTemperatures();           // Read oneWire sensors
  moisture_1 = analogRead(SOILMOISTURE_1); // Reads Moisture sensor 1
  //int moisture_2 = analogRead(SOILMOISTURE_2); // Reads Moisture sensor 2
  temp_probe1 = sensors.getTempC(sensor1);
  temp_probe2 = sensors.getTempC(sensor2);
  temp_probe3 = sensors.getTempC(sensor3);
  // read indoor BME280 data
  indoorTemp = bme.readTemperature();
  indoorHumdity = bme.readHumidity();
  indoorPressure = bme.readPressure() / 100;
  leafTemp = indoorTemp - tempDiff;

  // Calculate VPD
  indoorVPD = ((610.7 * pow(10, ((7.5 * leafTemp) / (237.3 + leafTemp)))) / 1000) - ((610.7 * pow(10, ((7.5 * indoorTemp) / (237.3 + indoorTemp))) / 1000)) * (indoorHumdity / 100);

  // Calculate Probe AVG
  temp_probeAVG = (temp_probe1 + temp_probe2 + temp_probe3) / 3;
}

void updateOutputs()
{

    // HEATER SET POINT SELECTION
    if (fermentationPVSource == 0)
    {
        fermentationPV = indoorTemp;
    }
    else if (fermentationPVSource == 1 && temp_probeAVG > 19)
    {
        fermentationPV = temp_probeAVG;    
    }

    // HEATER OUT UPDATE
    if (fermentationPV < (fermentationSetPoint_1 - fermentationThreshold_1))
    {

        digitalWrite(HEATER_1, LOW); // Turn on HEATER_1
        heater_1_status = 50;
    }
    else if (fermentationPV > fermentationSetPoint_1)
    {

        digitalWrite(HEATER_1, HIGH); // Turn off HEATER_1
        heater_1_status = 0;
    }
        
    // COOLER OUT UPDATE

    if (startTime >= coolerBlockStartTime )
  {
    coolerBlocked = false;
    coolerBlockStartTime = 0;
  }

    if (fermentationPV > (fermentationSetPoint_1 + fermentationThreshold_1) && coolerBlocked) // if hotter than SP and cooling not blocked cooling ON
    {
        digitalWrite(COOLER_1, LOW); // Turn on COOLER_1
        cooler_1_status = 50;
    }
    else if (fermentationPV < fermentationSetPoint_1) // if cooler than SP and cooling not blocked cooling ON
    {
        unsigned long coolerBlockStartTime = millis() + coolerBlockTime;
        coolerBlocked = true;
        digitalWrite(COOLER_1, HIGH); // Turn off COOLER_1
        
        cooler_1_status = 0;
    }

}

// Connect to WIFI
void connectWifi()
{
  Sprintln("Connecting to Wireless Network...");
  WiFi.begin(WIFI_NAME, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Sprintln("WiFi connected!");
  digitalWrite(LEDBLUE, HIGH);
  Sprintln(WiFi.localIP());
}

// Reconnect to WIFI
void reconnectWifi()
{
  // check wifi status and reconnect if disconnected
  if ((WiFi.status() != WL_CONNECTED) && (millis() > CHECKWIFI))
  {
    Sprintln("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(WIFI_NAME, WIFI_PASS);
    CHECKWIFI = millis() + 30000;
  }
}

// Connect to MQTT Broker
void reconnectBroker()
{
  while (!client.connected())
  {
    Sprintln("Connecting to Mosquitto at ");
    Sprintln(mosquittoServer);
    if (client.connect("ESP8266CLIENT"))
    {
      Sprintln("Connected");
      Sprintln("Subscribing to: ");
      Sprintln(inTopic);
      if (client.subscribe(inTopic))
      {

        Sprintln("Conectado!");
      }
      else
      {
        Sprintln("fallo Suscripciión");
      }
      client.subscribe("/garden/Out2");
    }
    else
    {
      Sprintln(client.state());
    }
  }
}

// Callback MQTT Functions
void callback(char *topic, byte *payload, unsigned int length)
{
  Sprintln("Message arrived [");
  Sprintln(topic);
  Sprintln("] ");
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);

  String strTopic = String((char *)topic);

  if (strTopic == "/garden/Out2")
  {

    if (message == "0")
    {
      digitalWrite(HEATER_1, HIGH);
    }
    else
    {
      digitalWrite(HEATER_1, LOW);
    }
  }

  if (strTopic == "/garden/HeaterSP1")
  {

    fermentationSetPoint_1 = message.toFloat();
  }
  if (strTopic == "/garden/HeaterSP2")
  {

    heaterSetPoint_2 = message.toFloat();
  }

  if (strTopic == "/garden/Heater1Source")
  {

    fermentationPVSource = message.toFloat();
  }

  Sprintln(fermentationSetPoint_1);
  //Serial.write(payload, length);
  Sprintln();
}

// InfluxDB Configuration
void influxConf()
{

  Sprintln("Influx Configuration...");
  influx.configure(INFLUX_DATABASE, INFLUX_IP); //third argument (port number) defaults to 8086
}

// BME Configuration
void bmeConf()
{

  if (!bme.begin(0x76))
  { //other potential address for BME280 could be 0x77
    Sprintln("Could not find BME280, check wiring!");
    while (1)
      ;
  }
}

// Mosquitto Configuration
void mosquittoConf()
{
  client.setServer(mosquittoServer, mosquittoServerPort);
  client.setCallback(callback);
}




// Update influxDB

void publish2DB();
{
    sprintf(tags, "Location=indoorGarden,What=Tray-1"); //write a tag called
    sprintf(fields, "temp_probe1=%f,temp_probe2=%f,temp_probe3=%f,temp_probeAVG=%f,indoorTemp=%f,indoorHumdity=%f,indoorPressure=%f,VPD=%f,moisture_1=%i,heater1_stat=%i,fermentationSetPoint_1=%f,fermentationPV=%f",
          temp_probe1, temp_probe2, temp_probe3, temp_probeAVG, indoorTemp, indoorHumdity, indoorPressure, indoorVPD, moisture_1, heater_1_status, fermentationSetPoint_1, fermentationPV);
    bool writeSuccessful = influx.write(INFLUX_MEASUREMENT_1, tags, fields);
    if (!writeSuccessful)
    {
    Sprintln("error: ");
    Sprintln(influx.getResponse());
    }
}