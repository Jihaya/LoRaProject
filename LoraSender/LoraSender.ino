//https://www.alictronix.com/archives/860
#include <SPI.h>
#include <LoRa.h>
#include "DHT.h"
#include "SSD1306.h"
#include <Arduino.h>
#include <TinyGPS++.h>  
#include <HardwareSerial.h>

TinyGPSPlus gps;  //กำหนด object ชื่อ gps                          
HardwareSerial mySerial1(1);  //กำหนด Hardware Serial ชื่อ mySerial1 
   
SSD1306  display(0x3c, 4, 15);
//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16    

//pins used 4, 5, 12(Switch OFF), 14, 15(RX), 16, 17(Switch ON), 18, 19, 23(DHT22), 26, 27, 33(TX),  

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
#define BAND    915E6  //915E6 กำหนดคลื่นความถี่

//กำหนดขา I/O ของปุ่ม 
#define BTN_PIN 17
#define BTN_PIN2 12

#define DHTPIN 23
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
 
int counter = 0; 
String ID = "03"; 
String state = "";
 
void setup() {
  Serial.begin(115200);
  SPI.begin(5,19,27,18);
  dht.begin();
  
  // กำหนดขา pin ของ GPS
  mySerial1.begin(9600, SERIAL_8N1, 33, 2);   //33-TX 15-RX

  //กำหนดขาต่างๆ เป็นสัญญาณ Digital ขาเข้า-ขาออก
  pinMode(BTN_PIN, INPUT);
  pinMode(25,OUTPUT); //Send success, LED will bright 1 second
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH);
 
  
  while (!Serial); //If just the the basic function, must connect to a computer
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Sender");
  display.display();


  // ตรวจเช็คสถานะ Lora
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initial OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(2000);
}

void loop() {
  //Serial.println(state);
  
  //อ่านค่าอุณหภูมิและความชื้น
  int c = dht.readTemperature();
  int h = dht.readHumidity();

  //กำหนดดีเลย์ในการทำงานลูป
  delay(1000);
  
  //เช็คปุ่มกด 
  if (digitalRead(BTN_PIN) == 1) {
      state = "Start";
  } 
  if (digitalRead(BTN_PIN2) == 1) {
      state = "Stop";
      Serial.println("Ready!");
      LoRa.print("'");
      LoRa.print("STATUS");
      LoRa.print("': ");
      LoRa.print("'");
      LoRa.print(state);
      LoRa.print("' ");
      LoRa.endPacket();
      
      // แสดงบนหน้าจอ OLED
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.drawString(1, 2, state);
      display.display();
  }

  //เช็คสถานะ
  if (state == "Start") { 
    delay(2000);

    //แสดงจำนวน Package ที่ส่งใน Serial 
    Serial.print("Sending packet: ");
    Serial.println(counter);
  
    //แสดงค่าอุณหภูมิ และความชื้นใน Serial 
    Serial.print("Temperature: ");
    Serial.print(c);
    Serial.print("*C ");
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print("%\n");
    
    // แสดงค่า GPS ใน Serial
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 5); //แสดงค่าละติจูดเป็นองศาทศนิยม 5ตำแหน่ง
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 4); //แสดงค่าลองจิจูดเป็นองศาทศนิยม 4ตำแหน่ง
    
    smartDelay(2000);
     //smartDelay(1000);                                      
     //กรณี pin ไม่ตรงหรือต่อสายผิด
    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring\n"));

    // แสดงหน้าจอ LED
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(1, 2, state);
    display.display();

    // เริ่มต้นการส่งข้อมูล Lora
    LoRa.beginPacket();
    
    //LoRa.print(counter);
    counter++;
  
    // Send Packet
    LoRa.print("'");
    LoRa.print("FROM_DEVICE");
    LoRa.print("': ");
    LoRa.print("'");
    LoRa.print(ID);
    LoRa.print("' ");

    LoRa.print("'");
    LoRa.print("TEMP");
    LoRa.print("': ");
    LoRa.print(c);
    LoRa.print(" ");
    
    LoRa.print("'");
    LoRa.print("HUMID");
    LoRa.print("': ");
    LoRa.print(h);
    LoRa.print(" ");
    
    LoRa.print("'");
    LoRa.print("LAT");
    LoRa.print("': ");
    LoRa.print(gps.location.lat(), 5);
    LoRa.print(" ");
  
    LoRa.print("'");
    LoRa.print("LON");
    LoRa.print("': ");
    LoRa.print(gps.location.lng(), 4);  
    LoRa.print(" ");

    LoRa.print("'");
    LoRa.print("STATUS");
    LoRa.print("': ");
    LoRa.print("'");
    LoRa.print(state);
    LoRa.print("' ");
    LoRa.endPacket();
    
    
    digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                       // wait for a second
  }

}
//ฟังก์ชั่น SmartDelay ของ GPS
static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (mySerial1.available())
      gps.encode(mySerial1.read());
 } while (millis() - start < ms);
}
