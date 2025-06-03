



// Select your modem:
#define TINY_GSM_MODEM_SIM900
#include <DHT.h>      // подключаем библиотеку для датчика
DHT dht(2, DHT11);  // сообщаем на каком порту будет датчик
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

int measurePin = A5;                            
//вывода А0, к которому подключен вывод датчика для передачи значений 
int ledPower = 5;                               
// задаём переменную для 
// задаём переменную для 
//вывода 2, к которому подкючен управляющий вывод датчика 
int samplingTime = 280;                         
// задаём переменную для 
//времени (в микросекундах), в течении которого датчик производит подсчёт 
//значений 
int deltaTime = 40;                             
// задаём переменную для 
//времени, которое необходимо датчику для передачи данных в буфер 
int sleepTime = 9680;                           
// задаём переменную для 
//времени, в течении которого к датчику не будет обращения 
float voMeasured;                               
// задаём переменную для 
//"сырых" значений с датчика 
float calcVoltage;                              
//значений, переведённых из "сырых" в вольты 
float dustDensity;                              
// задаём переменную для 
// задаём переменную для 
//значений, переведённых из вольт в плотность микрочастиц пыли в воздухе 

int sensorPin = A0;
int SensorValue = 0;
int sensorPinSm = A1;
int smSensor = 0;
// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
//#ifndef __AVR_ATmega328P__
//#define SerialAT Serial1

// or Software Serial on Uno, Nano
//#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(7, 8);  // RX, TX
//#endif

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
#define TINY_GSM_YIELD() { delay(4); }
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
// Your GPRS credentials, if any
const char apn[]      = "internet";
const char gprsUser[] = "gdata";
const char gprsPass[] = "gdata";

// MQTT details
const char* broker = "dev.rightech.io";

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define PUB_DELAY (5 * 1000) /* 5 seconds */

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

uint32_t lastReconnectAttempt = 0;
bool r1 = false;
bool r2 = false;
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.println(topic);
  SerialMon.write(payload, len);
  SerialMon.println();

  if(topic == "rele1"){
    if(payload==1){
      r1 = true;
    }
    else r1 = false;
    
  }
  else if(topic == "rele2"){
    if(payload==1){
      r2 = true;
    }
    else r2 = false;
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
 boolean status = mqtt.connect("mqtt-ghost_blood34732004-n2n31i", "1234", "1234");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");

  mqtt.subscribe("base/state/temperature");
  mqtt.subscribe("base/state/fire");
  mqtt.subscribe("base/state/trueFire");
  mqtt.subscribe("base/state/smoke");
  return mqtt.connected();
}


void setup() {
  pinMode(ledPower, OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(6, OUTPUT);
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
 dht.begin();                // запускаем датчик DHT11
   Serial.begin(9600);   // подключаем монитор порта
  SerialMon.println("Wait...");

  // Set GSM module baud rate
  SerialAT.begin(9600);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback([] (char* topic, byte * payload, unsigned int len)  {
    SerialMon.write(payload, len);
    SerialMon.println();
  });
}


   
long last = 0;
void publishTemperature() {
  digitalWrite(ledPower, LOW);                  
//выводе датчика значение LOW 
delayMicroseconds(samplingTime);              
//произведёт подсчет 
voMeasured = analogRead(measurePin);          
//датчика 
delayMicroseconds(deltaTime);                 
//передаст данные в буфер 
digitalWrite(ledPower, HIGH);                 
//выводе датчика значение HIGH 
delayMicroseconds(sleepTime);                 
//(минимальное время между опросами датчика) 
calcVoltage = voMeasured * (5.0 / 1024.0);    
//значения в вольты 
dustDensity = 0.17 * calcVoltage - 0.1;       
//плотность микрочастиц пыли в воздухе 
Serial.print("Raw Signal Value: ");           
//монитор порта "Сырые значения" 
Serial.print(voMeasured);                     
//значения с датчика в монитор порта 
Serial.print(" --- Voltage: ");               
//монитор порта "Вольтаж" 
Serial.print(calcVoltage);                    
//датчика, переведённые в вольты 
Serial.print(" --- Dust Density: ");          

Serial.println(dustDensity);                  
// выводим значение 
//плотности микрочастиц пыли в воздухе в мкг/м3 
  long now = millis();
  // считываем температуру (t) и влажность (h)
   float h = dht.readHumidity();
   float t = dht.readTemperature();
   int tt = round(t);
   smSensor = analogRead(sensorPinSm);
   SensorValue = analogRead(sensorPin);
  if (now - last > PUB_DELAY) {
    //mqtt.publish("base/state/temperature", String(h));
    mqtt.publish("base/state/temperature",  String(tt).c_str());
    mqtt.publish("base/state/fire", String(SensorValue).c_str());
    mqtt.publish("base/state/trueFire",  String(voMeasured).c_str());
    mqtt.publish("base/state/smoke", String(dustDensity).c_str());
    Serial.print("Temperature: ");
   Serial.println(tt);
    last = now;
  }
  if(tt > 31){
    digitalWrite(3,LOW);
   
  }
  if(tt <= 30){
    digitalWrite(3,HIGH);
    }
    if(SensorValue <=  200){
      digitalWrite(4,LOW);
      Serial.print("Fire: ");
   Serial.println(SensorValue);
    }
    if(SensorValue > 400){
      digitalWrite(4,HIGH);
    }
    if(dustDensity>=0.4){
      digitalWrite(6,LOW);
    }
    if(dustDensity<0.2){
      digitalWrite(6,HIGH);
    }
   
}

void loop() {
  publishTemperature();
  
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) {
        SerialMon.println("GPRS reconnected");
      }
    }
  }

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  mqtt.loop();
  publishTemperature();
}
