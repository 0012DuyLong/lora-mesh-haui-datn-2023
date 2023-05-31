#include <avr/wdt.h>
#include <SPI.h>
#include <LoRa.h>
#include "DHT.h"

// #define USE_NODE_1    //Temp-Humi    
// #define USE_NODE_2   //Gas       
// #define USE_NODE_3  //Light
//#define USE_NODE_4  //Light2
#define USE_NODE_5  //Light3

#define GATEWAY_ADDRESS           0x00
#define BROADCAST_ADDRESS         0xFF
#define HEARDER                   0xA5
#define CMD_TEMP_HUMI             0xA0
#define CMD_GAS                   0xC0
//#define CMD_LIGHT                 0xB0
//#define CMD_LIGHT_2               0xD0 //new
#define CMD_LIGHT_3               0xE0 //new



#define NODE_1_ADDR               0x1 // temp-humi
#define NODE_2_ADDR               0x2 // Gas
//#define NODE_3_ADDR               0x3 // Light
//#define NODE_4_ADDR               0x4 // Light2
#define NODE_5_ADDR               0x5 // Light3

#define TIME_UPDATE_TEMP_HUMI     1700
//#define TIME_UPDATE_LIGHT         1200
//#define TIME_UPDATE_LIGHT_2       1250  //new
#define TIME_UPDATE_LIGHT_3       1300  //new
#define TIME_UPDATE_GAS           2000
#define TIME_SHORT_POLL           15000 

#define GAS_PIN       A0
#define LIGHT_PIN     A1
//#define LIGHT_PIN_2   A1
#define DHT_PIN       3
#define BUTTON_PIN    4
#define LED_PIN       8

#if defined(NODE_1_ADDR)     
const int DHTTYPE = DHT11;
 
DHT dht(DHT_PIN, DHTTYPE);
#endif

const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // Hardware interrupt pin

byte seq = 0;                               // count of outgoing messages
byte souceAddress, localAddress;     // address of this device
byte destination = GATEWAY_ADDRESS;       // destination to send to
long lastSendTime = 0;                   // last send time
byte lastSeq = 0;
byte payloadForward[7] = {0, };
bool isForward = false;
bool sendOnlineFlag = false;
void setup() {
  wdt_enable(WDTO_2S);    // Set watdog 2s

  Serial.begin(9600);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

#if defined(NODE_1_ADDR)   
  dht.begin();         // Khởi động cảm biến
#endif


#if defined(USE_NODE_1)
  souceAddress = NODE_1_ADDR;
  localAddress = NODE_1_ADDR;
#elif defined(USE_NODE_2)
  souceAddress = NODE_2_ADDR;
  localAddress = NODE_2_ADDR;
// #elif defined(USE_NODE_3)
//   souceAddress = NODE_3_ADDR;
//   localAddress = NODE_3_ADDR;
// #elif defined(USE_NODE_4)
//   souceAddress = NODE_4_ADDR;
//   localAddress = NODE_4_ADDR;
#elif defined(USE_NODE_5)
  souceAddress = NODE_5_ADDR;
  localAddress = NODE_5_ADDR;
#endif

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  // LoRa.onReceive(onReceive);
  // LoRa.receive();
}

void loop() {
  static unsigned long ticks = 0;
  static unsigned long tickShortPoll = 0;
  byte payload[2]; 
  int analogValue;

  if(digitalRead(BUTTON_PIN) == 0){
    while(digitalRead(BUTTON_PIN) == 0);
    sendOnlineFlag = true;
  }

  if(millis() - tickShortPoll > TIME_SHORT_POLL){
    sendOnlineStatus();
    tickShortPoll = millis();
  }
#if defined(USE_NODE_1)
  static float h_lastValue = 0;
  static float t_lastValue = 0;
  
  if(runEvery(TIME_UPDATE_TEMP_HUMI) || sendOnlineFlag == true){
    float h = dht.readHumidity();    //Đọc độ ẩm
    float t = dht.readTemperature(); //Đọc nhiệt độ
    
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
    }
    else{
      Serial.print("Nhiet do: ");
      Serial.println(t);               //Xuất nhiệt độ
      Serial.print("Do am: ");
      Serial.println(h);               //Xuất độ ẩm

      payload[0] = t;
      payload[1] = h;
      sendNodeInfortoGateway(CMD_TEMP_HUMI, payload, 2); // gửi đến gateway
    }
    
    sendOnlineFlag = false;
  }

#elif defined(USE_NODE_2)

  static int gas_lastValue = 0;
  if( (TIME_UPDATE_GAS) || sendOnlineFlag == true){
    analogValue = analogRead(GAS_PIN);

    Serial.print("Sending gas packet: ");
    Serial.println(analogValue);

    payload[0] = (analogValue >> 8) & 0xFF;
    payload[1] = (analogValue) & 0xFF;

    sendNodeInfortoGateway(CMD_GAS, payload, 2);

    sendOnlineFlag = false;
  }

#elif defined(USE_NODE_5)
  static int light_lastValue = 0;

  if(runEvery(TIME_UPDATE_LIGHT_3) || sendOnlineFlag == 1){
    analogValue = 1024 - analogRead(LIGHT_PIN);
    Serial.print("Sending light3 packet: ");
    Serial.println(analogValue);

    payload[0] = (analogValue >> 8) & 0xFF;
    payload[1] = (analogValue) & 0xFF;

    sendNodeInfortoGateway(CMD_LIGHT_3, payload, 2);

    sendOnlineFlag = false;
  }
// #elif defined(USE_NODE_3)
//   static int light_lastValue = 0;

//   if(runEvery(TIME_UPDATE_LIGHT) || sendOnlineFlag == 1){
//     analogValue = 1024 - analogRead(LIGHT_PIN);
//     Serial.print("Sending light packet: ");
//     Serial.println(analogValue);

//     payload[0] = (analogValue >> 8) & 0xFF;
//     payload[1] = (analogValue) & 0xFF;

//     sendNodeInfortoGateway(CMD_LIGHT, payload, 2);

//     sendOnlineFlag = false;
//   }

// #elif defined(USE_NODE_4)
//   static int light_lastValue = 0;

//   if(runEvery(TIME_UPDATE_LIGHT) || sendOnlineFlag == 1){
//     analogValue = 1024 - analogRead(LIGHT_PIN);
//     Serial.print("Sending light2 packet: ");
//     Serial.println(analogValue);

//     payload[0] = (analogValue >> 8) & 0xFF;
//     payload[1] = (analogValue) & 0xFF;

//     sendNodeInfortoGateway(CMD_LIGHT_2, payload, 2);

//     sendOnlineFlag = false;
//   }

#endif

  if(millis() - ticks > 100){
    ticks = millis();
    onReceive(LoRa.parsePacket());
    if(isForward == true){
      if(payloadForward[0] == HEARDER){
        forwardToGateway(payloadForward, sizeof(payloadForward));
        memset(payloadForward, 0, sizeof(payloadForward));
      }
    }
  }
  wdt_reset();  // reset counter wdg
}
// HEARDER_ScrAddr_DesAddr_Length_Sequence_Cmd_Data

void sendNodeInfortoGateway(byte cmd, byte* payload, byte lengthOfData){
  digitalWrite(LED_PIN, 1);
  LoRa.beginPacket();                   // start packet
  LoRa.write(HEARDER);
  LoRa.write(souceAddress);             // add destination address
  LoRa.write(destination);              // add sender address
  LoRa.write(seq);                      // add message ID
  LoRa.write(cmd);   
  LoRa.write(lengthOfData);        // add payload length
  for(byte i = 0; i < lengthOfData; i++){
    LoRa.write(payload[i]);
  }
  LoRa.endPacket(true);                     // finish packet and send it
  seq++;  
  Serial.println("Sending Done ");       
  digitalWrite(LED_PIN, 0); 
}

void forwardToGateway(byte* payload, byte lengthOfData){
 digitalWrite(LED_PIN, 1);
  for(byte i = 0; i < lengthOfData; i++){
    LoRa.write(payload[i]);
  }
  LoRa.endPacket(true);                     // finish packet and send it
  Serial.println("forward Done ");        
  digitalWrite(LED_PIN, 0);
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

void onReceive(int packetSize) {
  byte data[2];
  if (packetSize == 0) return;

  byte byHeader = LoRa.read();
  if(byHeader == HEARDER){
  
    byte byScrAddress = LoRa.read();
    Serial.print("byScrAddress: ");
    Serial.println(byScrAddress);

    byte byDesAddress = LoRa.read();

    if(byScrAddress == localAddress){
      Serial.println("It's mine.");
      // Process this message
    }
    else{
      Serial.print("This message is not for me.");
      byte byIncommingSequence = LoRa.read();
      byte byIncommingCmd = LoRa.read();
      byte byIncomingLength = LoRa.read();
      byte byData1 = LoRa.read();
      byte byData2 = LoRa.read();
      if(lastSeq != byIncommingSequence){
        payloadForward[0] = byHeader;
        payloadForward[1] = byScrAddress;
        payloadForward[2] = byDesAddress;
        payloadForward[3] = byIncommingSequence;
        payloadForward[4] = LoRa.read(); // length
        payloadForward[5] = LoRa.read(); // byData1
        payloadForward[6] = LoRa.read(); // byData2
        lastSeq = byIncommingSequence;
        isForward = true;
        Serial.println("-> Forward !");
      }
    }
  }
  // print RSSI of packet
  Serial.print(" _ with RSSI ");
  Serial.println(LoRa.packetRssi());
}

void sendOnlineStatus(void){
  digitalWrite(LED_PIN, 1);
  LoRa.beginPacket();                   // start packet
  LoRa.write(HEARDER);
  LoRa.write(souceAddress);             // add destination address
  LoRa.write(destination);              // add sender address
  LoRa.write(seq);                      // add message ID 
  LoRa.endPacket(true);                     // finish packet and send it
  seq++;  
  Serial.println("Sending online status done ");       
  digitalWrite(LED_PIN, 0);  
}
