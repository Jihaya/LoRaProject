#include <SPI.h>
#include <LoRa.h>
#include <NTPClient.h>
#include "SSD1306.h"
#include "WiFi.h"
#include <IOXhop_FirebaseESP32.h>

///#define FIREBASE_HOST "tracksystem6.firebaseio.com"
//#define FIREBASE_AUTH "Z9rfkOQIbaTlQ67CvUJ0xRTiQB1J44eHFToQLfYq"
#define FIREBASE_HOST "logistics-car.firebaseio.com/"
#define FIREBASE_AUTH "raASc2JGJM9kGYcY1GJVU5QtYfTraVW0olvhoFeS"
#define WIFI_SSID "SaKuRai"
#define WIFI_PASSWORD "930893089308"

const long utcOffsetInSeconds = 25200; //กำหนดเวลาให้เป็นเวลาประเทศไทย 7*3600
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
SSD1306  display(0x3c, 4, 15);
 
//OLED pins to ESP32 GPIOs via this connection:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16
// WIFI_LoRa_32 ports
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)
 
#define SS      18
#define RST     14
#define DI0     26
#define BAND    915E6 //กำหนดคลื่นความถี่

String state = "";

void setup() {
  Serial.begin(115200);

  //กำหนดขาต่างๆ เป็นสัญญาณ Digital ขาเข้า-ขาออก
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
   
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  
  while (!Serial); //if just the the basic function, must connect to a computer
  delay(1000);
  Serial.println("LoRa Receiver"); 
  display.drawString(5,5,"LoRa Receiver"); 
  display.display();
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
   
  if (!LoRa.begin(BAND)) {
    display.drawString(5,25,"Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initial OK!");
  display.drawString(5,25,"LoRa Initializing OK!");
  display.display();
}

void loop() {
  //อ่านค่าเวลา
  timeClient.update();
  int H = timeClient.getHours();
  int M = timeClient.getMinutes();
  //int S = timeClient.getSeconds();

  String strs = "'Time' ";
  String R = ": ";
  String space = " ";
  
  // ตรวจสอบว่ามีข้อมูลเข้ามาใหม่หรือเปล่า
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    
    // Read Packet และเก็บไว้ในตัวแปร data
    while (LoRa.available()) {
    String data = LoRa.readString();
    Serial.print(data);
    state = "Receiving Data";
    
    //หน้าจอ OLED
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(1,2, state);
    display.display();

    //Send Data to Firebase
    String Datar;
    Datar = Firebase.pushString("Cars3",data + strs + H + space + R + M);
    if (Firebase.failed()) {
      Serial.print("pushing /Data failed:");
      Serial.println(Firebase.error());  
      return;
    }
    }
    //Serial.print(daysOfTheWeek[timeClient.getDay()]);
    //Serial.print(", ");
    Serial.print("Last ");
    Serial.print(H);
    Serial.print(":");
    Serial.println(M);
    
    // print RSSI of packet
    //Serial.print(" with RSSI ");
    //Serial.println(LoRa.packetRssi());
    //display.drawString(20, 45, "RSSI:  ");
    //display.drawString(70, 45, (String)LoRa.packetRssi());
    //display.display();
  }
  else{
    state = "Waiting to receive Data";
    //หน้าจอ OLED
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(1,2, state);
    display.display();
  }
}
