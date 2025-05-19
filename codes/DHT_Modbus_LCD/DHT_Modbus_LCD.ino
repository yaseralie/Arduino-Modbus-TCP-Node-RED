//Device Arduino UNO with ethernet shield
//This device as modbus TCP Client, send command to TCP Server
//using library DHT sensor Library by Adafruit Version 1.4.3
//library I2C LED 20x4: https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
//For UNO, use SDA = A4, SCL = A5
//For MEGA, use SDA = 20, SCL = 21
//As memory reason, please use Arduino Mega instead Arduino Uno

#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 88);

//Set the IP address node-red as modbus Server
IPAddress serverAddress(192, 168, 0, 101);
const int serverPort = 4058;

EthernetClient TCPclient;

//Configuration relay/button====================
int BUTTON_PIN = 7;
int OUTPUT_PIN = 8;
int OUTPUT_LED = 9;
//==============================================

//Configuration DHT-11====================
#define DHTPIN 2
// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT11     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);

// current temperature & humidity, updated in loop()
float t = 0.0;
float h = 0.0;
//=======================================

//millis================================
//Set every  sec send data to server
unsigned long previousMillis = 0; // variable to store the last time the task was run
const long interval = 5000; // time interval in milliseconds (eg 1000ms = 1 second)
//======================================

String msg;
String fan = "OFF";
bool status_button;

void setup()
{
  //Setup pin============================
  pinMode(BUTTON_PIN, INPUT_PULLUP); //button to turn ON relay fan
  pinMode(OUTPUT_PIN, OUTPUT); //relay
  pinMode(OUTPUT_LED, OUTPUT); //LED for connection signal
  //=====================================
  dht.begin();

  //LCD
  lcd.init();                      // initialize the lcd
  //Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Arduino Monitoring");
  lcd.setCursor(0, 1);
  lcd.print("Temperature: 0C");
  lcd.setCursor(0, 2);
  lcd.print("Humidity: 0%");
  lcd.setCursor(0, 3);
  lcd.print("Fan Status: OFF");

  //turn off relay fan
  digitalWrite(OUTPUT_PIN, LOW);

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Modbus TCP Monitoring");

  Ethernet.begin(mac, ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet: No Hardware");
    while (1);
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet connection error");
  }

  // connect to TCP server (Node-Red)
  if (TCPclient.connect(serverAddress, serverPort))
  {
    Serial.println("Connected to TCP server");
    //turn on LED, connection established
    digitalWrite(OUTPUT_LED, HIGH);
  }
  else
  {
    Serial.println("Failed to connect to TCP server");
    //turn off LED because no connection
    digitalWrite(OUTPUT_LED, LOW);
  }
}

void loop()
{
  unsigned long currentMillis = millis(); // mendapatkan waktu sekarang
  if (!TCPclient.connected()) {
    digitalWrite(OUTPUT_LED, LOW);
    Serial.println("Connection is disconnected");
    TCPclient.stop();

    // reconnect to TCP server (Node-Red)
    if (TCPclient.connect(serverAddress, serverPort))
    {
      digitalWrite(OUTPUT_LED, HIGH);
      Serial.println("Reconnected to TCP server");
    }
    else
    {
      digitalWrite(OUTPUT_LED, LOW);
      Serial.println("Failed to reconnect to TCP server");
    }
  }

  //check button=================================
  int buttonValue = digitalRead(BUTTON_PIN);
  if (buttonValue == LOW )
  {
    if (status_button == false)
    {
      fan = "ON";
      //turn on relay fan
      digitalWrite(OUTPUT_PIN, HIGH);
      status_button = true;
    }
  }
  else if (buttonValue == HIGH)
  {
    if (status_button == true)
    {
      fan = "OFF";
      //turn off relay fan
      digitalWrite(OUTPUT_PIN, LOW);
      status_button = false;
    }
  }
  //============================================

  //LCD=========================================
  lcd.setCursor(0, 1);
  lcd.print("Temperature: " + String(t) + "C");
  lcd.setCursor(0, 2);
  lcd.print("Humidity: " + String(h) + "%");
  lcd.setCursor(0, 3);
  lcd.print("Fan Status: " + fan + " ");

  // Checks whether it is time to run the task
  if (currentMillis - previousMillis >= interval) {
    // Save the last time the task was run
    previousMillis = currentMillis;

    // Perform tasks that need to be executed at intervals
    t = dht.readTemperature();
    h = dht.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(h);
    Serial.print("% ");
    Serial.print("Temperature = ");
    Serial.print(t);
    Serial.println(" C ");
    msg = String(t) + "," + String(h) + "," + fan;
    TCPclient.write(msg.c_str());
    TCPclient.flush();
  }

}
