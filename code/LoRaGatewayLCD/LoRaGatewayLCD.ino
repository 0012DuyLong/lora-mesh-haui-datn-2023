#include <avr/wdt.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define GATEWAY_ADDRESS       0x00
#define BROADCAST_ADDRESS     0xFF

#define HEARDER               0xA5 // byte header

#define CMD_TEMP_HUMI         0xA0
#define CMD_LIGHT             0xB0
#define CMD_GAS               0xC0
#define CMD_LIGHT_2           0xD0 //new
#define CMD_LIGHT_3           0xE0 //new

#define NODE_1_ADDR           0x1 // temp-humi
#define NODE_2_ADDR           0x2 // Gas
#define NODE_3_ADDR           0x3 // Light
#define NODE_4_ADDR           0x4 // Light2
#define NODE_5_ADDR           0x5 // Light3

#define BUTTON_PIN    8       // D8
#define LED_PIN       3       // D3
#define BUZZER_PIN    7       // D7

#define TIME_OFFLINE          20000 // ms
//LiquidCrystal_I2C lcd(0X27,16,2); //SCL A5 SDA A4
LiquidCrystal_I2C lcd(0X27,20,4);
const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

bool recvFlag = 0;

int temperature = 0;
int humidity = 0;
int light = 0;
int light2 = 0; //new
int light3 = 0; //new
int gas = 0;

unsigned long timeLastRecvNode1 = 0;
unsigned long timeLastRecvNode2 = 0;
unsigned long timeLastRecvNode3 = 0;
unsigned long timeLastRecvNode4 = 0;  //new
unsigned long timeLastRecvNode5 = 0;  //new

bool node1IsOnline = false;
bool node2IsOnline = false;
bool node3IsOnline = false;
bool node4IsOnline = false;  //new
bool node5IsOnline = false;  //new

bool node1_OfflinePrintFlag = false;
bool node2_OfflinePrintFlag = false;
bool node3_OfflinePrintFlag = false;
bool node4_OfflinePrintFlag = false;  //new
bool node5_OfflinePrintFlag = false;  //new
bool gas_warning = false;

bool checkClearLcd = false;

byte degree[8] = { // kí tự độ
  0B01110,
  0B01010,
  0B01110,
  0B00000,
  0B00000,
  0B00000,
  0B00000,
  0B00000
};

void setup() {
  wdt_enable(WDTO_2S);    // Set watdog timer 2s. Tránh treo chương trình

  Serial.begin(9600); // Khởi tạo debug uart
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);       // khởi tạo chân điều khiển led
  digitalWrite(LED_PIN, 0);       // tắt led

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 0);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Khởi tạo chân nút nhấn

  lcd.init();                     
  lcd.backlight(); 
  lcd.createChar(1, degree);
  //lcd.setCursor(0, 2);
  //lcd.print("Hello!"); 

  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {       // tần số 433Mhz
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  lcd.clear();
  LoRa.onReceive(onReceive);      // dăng kí calback cho hàm xử lý ngắt nhận
  LoRa.receive();
}

void loop() {
  if(digitalRead(BUTTON_PIN) == 0){
    gas_warning = false;
  }

  if(recvFlag){

    //bool displayNode1 = false;
    //bool displayNode2 = false;
    //bool displayNode3 = false;
     
    if(node1IsOnline == true){
      //displayNode1 = true;
      lcd.setCursor(0, 0);
      lcd.print("N1:T=");
      lcd.print(temperature);
      lcd.write(1);
      lcd.print("C  ");

      lcd.setCursor(11, 0);
      lcd.print("H=");
      lcd.print(humidity);
      lcd.print("%");
      node1_OfflinePrintFlag = false;
    }
    else{
      if(node1_OfflinePrintFlag == false){
        lcd.setCursor(0, 0);  
        //lcd.print("Node1: Offline      ");
        lcd.print("                    ");
      }
    }
      
    if(node3IsOnline == true){      
      //displayNode3 = true;
      lcd.setCursor(0, 2);
      lcd.print("N3:Lux=");
      lcd.print(light);
      node3_OfflinePrintFlag = false;
    }    
    else{
      if(node3_OfflinePrintFlag == false){       
        lcd.setCursor(0, 2);
        //lcd.print("Node3: Offline      ");
        lcd.print("                    ");
        //node3_OfflinePrintFlag = true;
      }
    }    
    
    if(node4IsOnline == true){      
      //displayNode4 = true;
      lcd.setCursor(0, 3);
      lcd.print("N4:Lux=");
      lcd.print(light2);
      node4_OfflinePrintFlag = false;
    }    
    else{
      if(node4_OfflinePrintFlag == false){       
        lcd.setCursor(0, 3);
        //lcd.print("Node4: Offline      ");
        lcd.print("          ");
        //node4_OfflinePrintFlag = true;
      }
    } 
    
    if(node5IsOnline == true){      
      //displayNode5 = true;
      lcd.setCursor(10, 3);
      lcd.print("N5:Lux=");
      lcd.print(light3);
      node5_OfflinePrintFlag = false;
    }    
    else{
      if(node5_OfflinePrintFlag == false){       
        lcd.setCursor(10, 3);
        //lcd.print("Node5: Offline      ");
        lcd.print("          ");
        //node5_OfflinePrintFlag = true;
      }
    }

    if(node2IsOnline == true){
      //displayNode2 = true;
      lcd.setCursor(0, 1);
      //lcd.setCursor(11, 2);
      lcd.print("N2:Gas="); 
      lcd.print(gas);
      node2_OfflinePrintFlag = false;
    }    
    else{
      if(node2_OfflinePrintFlag == false){        
        lcd.setCursor(0, 1);
        //lcd.print("Node2: Offline      ");
        lcd.print("                    ");
        //node2_OfflinePrintFlag = true;
      }
    }

    //if (!displayNode1 && !displayNode2 && !displayNode3) {
      //lcd.clear(); // Clear the LCD screen
    //}
    recvFlag = 0;
  }
  else{
    if(node1IsOnline == false && node2IsOnline == false && node3IsOnline == false && node4IsOnline == false && node5IsOnline == false){
      lcd.clear();
      recvFlag = 1;      
    }
  }

  // Check onl-off
  if(millis() - timeLastRecvNode1 > TIME_OFFLINE){
    node1IsOnline = false;
  }

  if(millis() - timeLastRecvNode2 > TIME_OFFLINE){
    node2IsOnline = false;
  }

  if(millis() - timeLastRecvNode3 > TIME_OFFLINE){
    node3IsOnline = false;
  }

  if(millis() - timeLastRecvNode4 > TIME_OFFLINE){
    node4IsOnline = false;
  }

  if(millis() - timeLastRecvNode5 > TIME_OFFLINE){
    node5IsOnline = false;
  }
 // End check


  if(gas_warning == true){
    buzzerToggle();
  }
  else{
    digitalWrite(BUZZER_PIN, 0);
  }

  wdt_reset();  // reset counter wdg
}

void onReceive(int packetSize) {

  byte data[2];
  if (packetSize == 0) return;

  byte byHeader = LoRa.read();
  if(byHeader == HEARDER){
    digitalWrite(LED_PIN, 1);
    byte byScrAddress = LoRa.read();
    Serial.print("byScrAddress: ");
    Serial.println(byScrAddress);
    
    switch(byScrAddress){
      case NODE_1_ADDR:
        node1IsOnline = true;
        timeLastRecvNode1 = millis(); // Lưu lại thời gian nhận gói tin cuối cùng 
        break;
      case NODE_2_ADDR:
        node2IsOnline = true;
        timeLastRecvNode2 = millis();
        break;
      case NODE_3_ADDR:
        node3IsOnline = true;
        timeLastRecvNode3 = millis();
        break;
      case NODE_4_ADDR:
        node4IsOnline = true;
        timeLastRecvNode4 = millis();
        break;
      case NODE_5_ADDR:
        node5IsOnline = true;
        timeLastRecvNode5 = millis();
        break;
      default:
        break;
    }
    byte byDesAddress = LoRa.read();
    if(byDesAddress == GATEWAY_ADDRESS){
      byte byIncommingSequence = LoRa.read();
      byte byIncommingCmd = LoRa.read();
      byte byIncomingLength = LoRa.read();
      switch(byIncommingCmd){
        case CMD_TEMP_HUMI:
          if(byIncomingLength == 2){
            data[0] = LoRa.read();
            data[1] = LoRa.read();
            temperature = data[0];
            humidity = data[1];
            recvFlag = 1;
            Serial.print("temperature: ");
            Serial.println(temperature);
            Serial.print("humidity");
            Serial.println(humidity);
            
          }
          else{
            Serial.println("error: message length does not match length");
          }          
          break;
        case CMD_GAS:
          if(byIncomingLength == 2){
            data[0] = LoRa.read();
            data[1] = LoRa.read();
            
            gas = (data[0] << 8) | data[1];
            recvFlag = 1;
            if(gas >600){
              gas_warning = true;
            }
            else{
              gas_warning = false;
            }
            Serial.print("Gas: ");
            Serial.println(gas);
          }
          else{
            Serial.println("error: message length does not match length");
          }
          break;
        case CMD_LIGHT:
          if(byIncomingLength == 2){
            data[0] = LoRa.read();
            data[1] = LoRa.read();
            light = (data[0] << 8) | data[1];
            recvFlag = 1;
            Serial.print("Light: ");
            Serial.println(light);
          }
          else{
            Serial.println("error: message length does not match length");
          }
          break;
        case CMD_LIGHT_2:
          if(byIncomingLength == 2){
            data[0] = LoRa.read();
            data[1] = LoRa.read();
            light2 = (data[0] << 8) | data[1];
            recvFlag = 1;
            Serial.print("Light2: ");
            Serial.println(light2);
          }
          else{
            Serial.println("error: message length does not match length");
          }
          break;
          case CMD_LIGHT_3:
          if(byIncomingLength == 2){
            data[0] = LoRa.read();
            data[1] = LoRa.read();
            light3 = (data[0] << 8) | data[1];
            recvFlag = 1;
            Serial.print("Light3: ");
            Serial.println(light3);
          }
          else{
            Serial.println("error: message length does not match length");
          }
          break;
        default:
          Serial.println("CMD err!");
          break;
      }
    }
    else{
      Serial.println("This message is not for me.");
    }
    digitalWrite(LED_PIN, 0); // tắt led
  }

  // print RSSI of packet
  Serial.print(" _ with RSSI ");
  Serial.println(LoRa.packetRssi());
}

void buzzerToggle(void){
  static unsigned long timeToggle = 0;
  static bool buzzerState = 0;

  if(millis() - timeToggle > 200){
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);
    timeToggle = millis();
  }
}