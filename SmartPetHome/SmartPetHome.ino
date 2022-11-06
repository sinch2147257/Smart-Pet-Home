 #include <stdlib.h>
 #include <DHT.h>

 #include "WizFi360.h"
 const int trig = 12; //Trigger pin of ultrasonic Sesnor
 const int echo = 11; //Echo pin of ultrasonic Sesnor
 long time_taken;
 int dist,distance;

 const int analogInPin = A0;
 int sensorValue = 0;
 int a;
 const int buzzer = 15;

 // setup according to the device you use
 #define WIZFI360_EVB_PICO

 // Emulate Serial1 on pins 6/7 if not present
 // remember that the wiznet is connected to 4 and 5 not 6 and 7
 #ifndef HAVE_HWSERIAL1
 #include "SoftwareSerial.h"
 #if defined(ARDUINO_MEGA_2560)
 SoftwareSerial Serial1(4, 5); // RX, TX
 #elif defined(WIZFI360_EVB_PICO)
 SoftwareSerial Serial2(4, 5); // RX, TX
 #endif
 #endif

 /* Baudrate */
 #define SERIAL_BAUDRATE 115200
 #if defined(ARDUINO_MEGA_2560)
 #define SERIAL1_BAUDRATE 115200
 #elif defined(WIZFI360_EVB_PICO)
 #define SERIAL2_BAUDRATE 115200
 #endif

 /* Sensor */
 #define DHTTYPE DHT11
 #define DHTPIN SDA
 #define CDSPIN A0

 /* Wi-Fi info */
 char ssid[] = "LAPTOP-GRDK2B0B 4358"; // your network SSID
(name)
 char pass[] = "R223u51*"; // your network password

 int status = WL_IDLE_STATUS; // the Wifi radio's status

 char server[] = "api.thingspeak.com"; // server address
 String apiKey ="9HM2EATBZJ1UPFGD"; // apki
key

 // sensor buffer
 char temp_buf[10];
 char humi_buf[10];
 char cds_buf[10];

 unsigned long lastConnectionTime = 0; // last time you
connected to the server, in milliseconds
 const unsigned long postingInterval = 30000L; // delay between
updates, in milliseconds

 // Initialize the Ethernet client object
 WiFiClient client;
 // Initialize the DHT object
 DHT dht(DHTPIN, DHTTYPE);

 void setup() {
 //initialize sensor
 pinMode(CDSPIN, INPUT);

 pinMode(13, OUTPUT);//tells the arduino to get ready to
exchange messages with the serial monitor at a data rate of 9600
bits/sec.
 pinMode(trig, OUTPUT); //configures the specified pin to
behave as output/input.
 pinMode(echo, INPUT);
 pinMode(buzzer, OUTPUT);
 dht.begin();

 // initialize serial for debugging
 Serial.begin(SERIAL_BAUDRATE);
 // initialize serial for WizFi360 module
 #if defined(ARDUINO_MEGA_2560)
 Serial1.begin(SERIAL1_BAUDRATE);
 #elif defined(WIZFI360_EVB_PICO)
 Serial2.begin(SERIAL2_BAUDRATE);
 #endif
 // initialize WizFi360 module
 #if defined(ARDUINO_MEGA_2560)
 WiFi.init(&Serial1);
 #elif defined(WIZFI360_EVB_PICO)
 WiFi.init(&Serial2);
 #endif

 // check for the presence of the shield
 if (WiFi.status() == WL_NO_SHIELD) {
 Serial.println("WiFi shield not present");
 // don't continue
 while (true);
 }

 // attempt to connect to WiFi network
 while ( status != WL_CONNECTED) {
 Serial.print("Attempting to connect to WPA SSID: ");
 Serial.println(ssid);
 // Connect to WPA/WPA2 network
 status = WiFi.begin(ssid, pass);
 }
 Serial.println("You're connected to the network");
 printWifiStatus();
 thingspeakTrans();
 }

 void calculate_distance(int trigger, int echo)
 {
 digitalWrite(trigger, LOW);
 delayMicroseconds(2);
 digitalWrite(trigger, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigger, LOW);

 time_taken = pulseIn(echo, HIGH);
 dist= time_taken*0.034/2;

 }

 void loop() {

 calculate_distance(trig, echo);
 distance=dist; //getting distance of ultrasonic sensor
 if(distance < 15 )
 {
 digitalWrite(13, HIGH); // turn the LED on (HIGH is the
voltage level)
 delay(1000);
 }
 else
 {
 digitalWrite(13, LOW); // turn the LED off by making the
voltage LOW
 delay(1000);
 }
 Serial.println(distance);
 delay(1000);
 water();
 // if 30 seconds have passed since your last connection,
 // then connect again and send data
 if (millis() - lastConnectionTime > postingInterval) {
 //sensorRead();
 thingspeakTrans();
 }
 }


 //Transmitting sensor value to thingspeak
 void thingspeakTrans() {

 // close any connection before send a new request
 // this will free the socket on the WiFi shield
 client.stop();
 calculate_distance(trig, echo);

 distance=dist;

 // if there's a successful connection
 if (client.connect(server, 80)) {
 Serial.println("Connecting...");
 Serial.println(distance);
 client.print(F("GET /update?api_key="));
 client.print(apiKey);
 client.print(F("&field1="));
 client.print(distance);
 client.print(F("&field2="));
 client.print(water());
 client.println();
 lastConnectionTime = millis();
 }
 else {
 // if you couldn't make a connection
 Serial.println("Connection failed");
 }
 }

 void printWifiStatus() {
 // print the SSID of the network you're attached to
 Serial.print("SSID: ");
 //IPAddress ip = WiFi.localIP();
 Serial.println(WiFi.SSID());
 Serial.println("STOPED HERE .... ");
 // print your WiFi shield's IP address
 //IPAddress ip = WiFi.localIP();
 Serial.print("IP Address: ");
 //Serial.println(ip);

 // print the received signal strength
 //long rssi = WiFi.RSSI();
 Serial.print("Signal strength (RSSI):");
 //Serial.print(rssi);
 Serial.println(" dBm");
 }

 int water(){
 sensorValue = analogRead(analogInPin);
 Serial.print("Sensor = " );
 a=sensorValue*100/1024;
 Serial.print(a);
 Serial.println("%");
 if(a < 20){
 digitalWrite(buzzer,HIGH);
 }
 else
 {
 digitalWrite(buzzer,LOW);
 }
 return a;
 }