//=========================================== version 9 cung dùng được rồi đây là bản nâng cấp
//========================================= van tay ======================
#include <Adafruit_Fingerprint.h>
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (GREEN wire)
// pin #3 is OUT from arduino  (WHITE wire)
// Set up the serial port to use softwareserial..
SoftwareSerial mySerial(2, 3);
#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is green wire, #1 is white
#define mySerial Serial2
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
//============================================================
#include <EEPROM.h>
#define buzzer 2
#define rl1 13
#define rl2 15
#define shld 33
#define inh 32
#define clk 25
#define qh 35
#define led1 26
#define led2 27
#define led3 14
#define led4 12
#define DO_PIN 39
#define button 36
//=================== BLE SMART blutube ==================
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
BLECharacteristic *pCharacteristic;
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value == "bat") {
        digitalWrite(rl1, HIGH);
        digitalWrite(buzzer, 1);
        delay(1000);
        digitalWrite(buzzer, 0);
      }
      else if (value == "tat") {
        digitalWrite(rl1, LOW);
      }
    }
};
//=================== BLE SMART TAG ==================
#include "BLEDevice.h"
int t_ble = 0;
int khoang_chach = -60;
int Contador = 0;
static BLEAddress *pServerAddress;
BLEScan* pBLEScan;
BLEClient*  pClient;
bool deviceFound = false;
bool Encendida = false;
bool BotonOff = false;
String knownAddresses[] = {"f4:8b:3b:e4:85:21", "1c:cc:d6:39:87:5c"}; /// THEM THE TAG TAI DAY
unsigned long entry;
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
}
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice Device) {
      //Serial.print("BLE Advertised Device found: ");
      //Serial.println(Device.toString().c_str());
      pServerAddress = new BLEAddress(Device.getAddress());
      bool known = false;
      bool Master = false;
      for (int i = 0; i < (sizeof(knownAddresses) / sizeof(knownAddresses[0])); i++) {
        if (strcmp(pServerAddress->toString().c_str(), knownAddresses[i].c_str()) == 0)
          known = true;
      }
      if (known) {
        Serial.print("Device found: ");
        Serial.println(Device.getRSSI());
        if (Device.getRSSI() > khoang_chach) {
          deviceFound = true;
          digitalWrite(rl1, HIGH); // bật relay khi có thẻ tag
          digitalWrite(buzzer, 1);
          delay(1000);
          digitalWrite(buzzer, 0);
        }
        else {
          deviceFound = false;
        }
        Device.getScan()->stop();
        delay(10);
      }

    }
};
//====================================================
// NTC
#include"ntc.h"
//pin
float adc_pin = 0;
float pin = 0, gt_filter = 0;;
int  t_pin = 0;
//===================== Wakeup esp ============================
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
RTC_DATA_ATTR int bootCount = 0;
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}
//===================== watchdogtimer =========================
#define WATCHDOG_TIMEOUT_S 5 // thoi gian reset 
hw_timer_t * watchDogTimer = NULL;
void IRAM_ATTR watchDogInterrupt()
{
  Serial.println("reboot");
  ESP.restart(); ////////////////////// lệnh reset esp
}
void watchDogRefresh()
{
  timerWrite(watchDogTimer, 0);                    //reset thời gian của timer
}
//=============================================================
//=========== rf id ==========
#include <SPI.h>
#include <MFRC522.h>
#define RST_PIN 22
#define SS_PIN 5
MFRC522 mfrc522(SS_PIN, RST_PIN);
//=============== khai bao bien ============
//=============================================================
int t1_che_do = 0, t2_che_do = 0, t3_che_do = 0, dem_che_do = 0, dem_ngu = 0;
bool a_che_do = 0, b_che_do = 0, chon_che_do = 0;

unsigned long UID[4];
unsigned long i;
int diachi = 1; int diachi2 = 0; int a = 0;
int gtmas = 0; int tong = 0; int gttong = 0;
int bandau;
int menu = 7; int gtmenu = 0;
int len = 5; int gtlen = 0;
int xuong = 6; int gtxuong = 0;
int macdinh = 1;
int dem_tong = 0; int dem_menu = 0; int dem = 0;
int led = 8;
int id_moi[4]; int id_er[4];
int diachi_zero;
int m = 5; int moi = 0;
int gt_zero;
int gt_er = 0;
int n = 4; int o = 0;
int demco = 1; int demchua = 1; int demmas = 1;
//=============================== các chương trình con cho thẻ rf id =========================
void in()
{
  for (int g = 0; g < 30; g++)
  {
    Serial.print(g); Serial.print("= "); Serial.print(EEPROM.read(g)); Serial.print(" ");
  }
  Serial.println(" ");
}
//==============
void tim_zero()
{
  while ( n < a) //Tìm ô nhớ có GT 0 đầu tiên
  {
    gt_zero = EEPROM.read(n);
    if (gt_zero == 0)
    {
      diachi_zero = n;
      //Serial.print("Zero: "); Serial.print(diachi_zero);
      break;
    }
    //Serial.print("   n: "); Serial.println(n);
    n++;
  }
  //Serial.println(".....................");
  if (diachi_zero == 0)
  {
    diachi_zero = a; //Nếu trong đoạn từ 4 đến số ô đã sử dụng không có ô nào có GT 0
    // thì diachi_zero = a là ô cuối cùng lưu thẻ
    n = 0;
  }
  // Serial.print("Zero: "); Serial.print(diachi_zero);
  // Serial.print("   n: "); Serial.print(n);
  // Serial.print("   a: "); Serial.println(a);
}
//======
void ss_epprom()
{
  //Serial.print("Zero: "); Serial.println(diachi_zero);
  //m = 5 ; a lưu số ô cuối được dùng
  while (m < a) //chạy từ ô 5 đến ô sử dụng cuối ... 5 < 20
  {
    moi = m + 4; //moi = 9
    for (m; m < moi; m++)
    {
      gt_er = EEPROM.read(m);
      if (o <= 3) //Lưu giá trị đọc từ eeprom vào id_er
      {
        id_er[o] = gt_er;
      }
      o++;
    }
    if (id_moi[0] == id_er[0] && id_moi[1] == id_er[1] && id_moi[2] == id_er[2] && id_moi[3] == id_er[3]) //Nếu thẻ có trong EEPR
    {
      demco = 1; demchua = 0; demmas = 0;
      //Serial.print("   demco: "); Serial.println(demco);
      break;
    }
    else if (id_moi[0] != id_er[0] or id_moi[1] != id_er[1] or id_moi[2] != id_er[2] or id_moi[3] != id_er[3]) //Nếu thẻ KO có trong EEPR
    { // Dùng or vì chỉ cần  1 byte khác với 1 byte lưu trong eeprom thì thẻ đó là thẻ khác
      // Nếu dùng && có thể có thẻ sẽ trùng 1 đến 2 byte
      demchua += 1; demco = 0; demmas = 0;
      //Serial.print("   demchua: "); Serial.println(demchua);
    }
    o = 0;
    m = moi;
  }
}
//============================================================================================
//========================== biến cho mở rộng ngõ vào ==========================
unsigned char  bt1 = 0xe7, bt2 = 0xd7, bt3 = 0xb7, bt4 = 0x77;

unsigned char hc165() {
  unsigned char incoming = 0, dataa = 0, dataas = 0;
  digitalWrite(shld, LOW);
  digitalWrite(shld, HIGH);
  digitalWrite(inh, LOW);
  for (int i = 0; i < 8; i++) {
    incoming = incoming << 1;
    if (digitalRead(qh) == 0) {
      incoming = incoming | 0x01;
    }
    digitalWrite(clk, 1);
    digitalWrite(clk, 0);
  }
  dataas = ~incoming;
  //Serial.println(dataas, BIN);
  return dataas;
}
//============================= VOID SETUP ==================================
void setup() {
  Serial.begin(9600);
  Serial.println("setup");
  pinMode(buzzer, OUTPUT);
  pinMode(rl1, OUTPUT);
  pinMode(rl2, OUTPUT);
  pinMode(inh, OUTPUT);
  pinMode(shld, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(qh, INPUT);
  pinMode(button, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  //====================
  digitalWrite(rl1, 0);
  digitalWrite(rl2, 0);
  digitalWrite(buzzer, 0);
  digitalWrite(led1, 0);
  digitalWrite(led2, 0);
  digitalWrite(led3, 0);
  digitalWrite(led4, 0);
  digitalWrite(led1, 1);

  //======================= van tay =================
  // set the data rate for the sensor serial port
  finger.begin(57600);
  finger.LEDcontrol(FINGERPRINT_LED_ON, 0, FINGERPRINT_LED_BLUE);
  delay(1);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    for (int i = 0; i <= 50; i++)
    {
      digitalWrite(buzzer, 1);
      delay(50);
      digitalWrite(buzzer, 0);
      delay(50);
    }
  }
  finger.getParameters();
  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
    Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
  //===================== BLE bluetube ==============
  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(BLEUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b"));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"),
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  //===================== BLE SMART TAG ==============
  BLEDevice::init("");
  pClient  = BLEDevice::createClient();
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  //===================== watchdogtimer ==============
  watchDogTimer = timerBegin(2, 80, true);
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, WATCHDOG_TIMEOUT_S * 1000000, false);
  timerAlarmEnable(watchDogTimer);
  //==================================================
  //============ rf id =====
  SPI.begin();
  mfrc522.PCD_Init();
  //==================
  EEPROM.begin(512);
  a = EEPROM.read(0);
  //    for (int i = 0; i < 512; i++) { // xoá bộ nhớ epprom nếu đầy quá nó ko thể thêm được nữa
  //      EEPROM.write(i, 0);
  //      EEPROM.commit();
  //      delay(5); //Phải có delay tối thiểu 5 mili giây giữa mối lần write
  //    }
  //======================= wakeup esp 32 ===============================
  print_wakeup_reason();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, 0); //1 = High, 0 = Low
  Serial.println("dayyy");

  //=====================================================================
}//============================================== end setup ============================
//============================= VOID BLUETOOTH ==================================
void Bluetooth() {
  Serial.println();
  Serial.println("BLE Scan restarted.....");
  deviceFound = false;
  BLEScanResults scanResults = pBLEScan->start(1);
  if (deviceFound) {
    Serial.println("Encender Lamara");
    Encendida = true;
    digitalWrite(rl1, HIGH);
    Contador = 0;
    delay(10);
  }
  //  else {
  //    digitalWrite(rl1, LOW);
  //    delay(100);
  //  }
}
//===============================================================================
void themthe()
{
  a = EEPROM.read(0); //đọc ô nhớ 0 xem đã sử dụng bao nhiêu ô nhớ

  if (a == 0) //Nếu chưa có thẻ PHỤ nào
  {
    int diachi_phu = 5; // 5,6,7,8

    if (mfrc522.PICC_IsNewCardPresent())
    {
      if (mfrc522.PICC_ReadCardSerial())
      {
        dem_ngu = 0;
        //Serial.print("Card UID:");
        for (byte i = 0; i < mfrc522.uid.size; i++)
        {

          UID[i] = mfrc522.uid.uidByte[i];
          id_moi[i] = UID[i];
        }
        for (byte i = 0; i < 4; i++)
        {
          EEPROM.write(diachi_phu, id_moi[i]);
          EEPROM.commit();
          diachi_phu = diachi_phu + 1;
          a = diachi_phu;
        }

        EEPROM.write(0, a); //Sau khi lưu 1 thẻ mới vào thì cập nhật số ô nhớ đã sử dụng vào ô 0
        EEPROM.commit();
        delay(500);
        Serial.println("đang luu");
        digitalWrite(buzzer, 1);
        delay(1000);
        digitalWrite(buzzer, 0);
        //        lcd.clear();
        //        chon_menu();

        mfrc522.PICC_HaltA();
      }
    }
  }
  //=====================

  else if ( a != 0) // Đã có 1 or nhiều thẻ phụ
  {
    if (mfrc522.PICC_IsNewCardPresent())
    {
      if (mfrc522.PICC_ReadCardSerial())
      {
        dem_ngu = 0;
        //Serial.print("Card UID:");
        for (byte i = 0; i < mfrc522.uid.size; i++)
        {

          UID[i] = mfrc522.uid.uidByte[i];
          id_moi[i] = UID[i];
        }
        ss_epprom(); //So sánh thẻ mới đưa vào với Eeprom

        if (demco == 1 && demchua == 0 && demmas == 0) //Nếu thẻ đã có
        {
          // Serial.print("................THE DA CO TRONG EEPROM..................");
          // Serial.print(" Zero: "); Serial.print(diachi_zero);
          // Serial.print("   Demco: "); Serial.println(demco);
          o = 0; m = 5; moi = 0; demco = 0; demchua = 0; demmas = 0;
          Serial.println("   THE DA CO!   ");
          digitalWrite(buzzer, 1);
          delay(200);
          digitalWrite(buzzer, 0);
          delay(100);
          digitalWrite(buzzer, 1);
          delay(200);
          digitalWrite(buzzer, 0);
          //in();
        }
        else if (demchua > 0 && demco == 0 && demmas == 0) //Nếu thẻ chưa có...THÌ LƯU THẺ ĐÓ VÀO EEPROM
        {
          tim_zero(); //Tìm vị trí 0 đầu tiên
          //   Serial.print("................THE CHUA CO..................");
          //   Serial.print(" Zero: "); Serial.print(diachi_zero);
          //   Serial.print("   Demchua: "); Serial.print(demchua);
          // Serial.print("  ID mới: ");Serial.print(id_moi[0]);Serial.print(":");Serial.print(id_moi[1]);Serial.print(":");
          // Serial.print(id_moi[2]);Serial.print(":");Serial.println(id_moi[3]);
          if (diachi_zero == a) //Nếu trong đoạn từ 4 đến số ô đã sử dụng không có ô nào có GT 0
            // a là ô cuối cùng lưu thẻ
          {
            for (int i = 0; i < 4; i++)
            {
              EEPROM.write(diachi_zero, id_moi[i]);
              EEPROM.commit();
              diachi_zero = diachi_zero + 1;
              a = diachi_zero;
            }
            EEPROM.write(0, a); //Sau khi lưu 1 thẻ mới vào thì cập nhật số ô nhớ đã sử dụng vào ô 0
            EEPROM.commit();
          }

          else if (diachi_zero == n) ////Nếu trong đoạn từ 4 đến số ô đã sử dụng CÓ ô = 0, thì gán ô đó vào n
          {
            for (int i = 0; i < 4; i++) //Lưu thẻ mới vào bắt đầu từ ô 0 đó
            {
              EEPROM.write(diachi_zero, id_moi[i]);
              EEPROM.commit();
              diachi_zero = diachi_zero + 1;
              //a = diachi_zero;
            }
            diachi_zero = a;
          }
          tim_zero();
          o = 0; m = 5; moi = 0; demco = 0; demchua = 0; demmas = 0;

          Serial.println("   DANG LUU...  ");
          digitalWrite(buzzer, 1);
          delay(1000);
          digitalWrite(buzzer, 0);
          //        lcd.clear();
          //        chon_menu();
          //in();
        }
        mfrc522.PICC_HaltA();
      }
    }
  }
}
//==================================== chế độ đọc thẻ ===========================
void docthe()
{
  tong = 1; bandau = false;
  if (mfrc522.PICC_IsNewCardPresent())
  {
    if (mfrc522.PICC_ReadCardSerial())
    {
      dem_ngu = 0;
      //Serial.print("Card UID:");
      for (byte i = 0; i < mfrc522.uid.size; i++)
      {

        UID[i] = mfrc522.uid.uidByte[i];
        id_moi[i] = UID[i];

      }
      ss_epprom();
      if (demco == 1 && demchua == 0 && demmas == 0) //Nếu thẻ có trong EEPROM
      {
        // Serial.print("................THE DA CO TRONG EEPROM..................");
        // Serial.print(" Zero: "); Serial.print(diachi_zero);
        // Serial.print("   Demco: "); Serial.println(demco);
        o = 0; m = 5; moi = 0; demco = 0; demchua = 0; demmas = 0;
        Serial.println("   MO DEN....   ");
        digitalWrite(rl1, 1);
        for (int i = 0; i < 4; i++)
        {
          id_moi[i] = 0;
        }
        digitalWrite(buzzer, 1);
        delay(1000);
        digitalWrite(buzzer, 0);
      }
      else if (demchua > 0 && demco == 0 && demmas == 0) //Nếu thẻ chưa có trong EEPROM
      {
        //   Serial.print("................THE CHUA CO..................");
        //   Serial.print(" Zero: "); Serial.print(diachi_zero);
        //   Serial.print("   Demchua: "); Serial.print(demchua);
        // Serial.print("  ID mới: ");Serial.print(id_moi[0]);Serial.print(":");Serial.print(id_moi[1]);Serial.print(":");
        // Serial.print(id_moi[2]);Serial.print(":");Serial.println(id_moi[3]);
        o = 0; m = 5; moi = 0; demco = 0; demchua = 0; demmas = 0;
        Serial.println("    SAI THE!    ");
        for (int i = 0; i < 4; i++)
        {
          id_moi[i] = 0;
        }
        digitalWrite(buzzer, 1);
        delay(200);
        digitalWrite(buzzer, 0);
        delay(100);
        digitalWrite(buzzer, 1);
        delay(200);
        digitalWrite(buzzer, 0);
        delay(100);
        digitalWrite(buzzer, 1);
        delay(200);
        digitalWrite(buzzer, 0);

      }
      mfrc522.PICC_HaltA();
    }
  }
}
//===============================================================================
int t1 = 0 , t2 = 0, t_nutnhan3 = 0, t_ngu = 0, dem_t_nutnhan3 = 0, dem_nutnhan3 = 0;
bool trang_thai_sac = 0, bien_dao = 0, bien_daos = 0, daoled = 0;
bool nut_nhan1 = 0, nut_nhan2 = 0, nut_nhan3 = 0, nut_nhan4 = 0, tt1 = 0, tt2 = 0, tt3 = 0, tt4 = 0;
bool tt4s = 0, ttled1 = 0;
String id_vua_doc = "";
String id_sosanh = "";
unsigned char id_doc [4];
unsigned char id_doc_epprom [4];
int vi_tri_o_nho = 0;
bool doc = 0, them = 0;
//int t1_che_do = 0, t2_che_do = 0, t3_che_do = 0, dem_che_do = 0;
//bool a_che_do = 0, b_che_do = 0, chon_che_do = 0;
int demvtsai = 0, tdemsai = 0, ttbuzzer = 0;
int denvantay = 0; // 0 luc khoi dong, 1 luc dung,2 luc sai
void loop() {
  //======================= ble =====================
  if (millis() - t_ble > 5000 && doc == 0)
  {
    t_ble = millis();
    BLEScanResults scanResults = pBLEScan->start(1);
  }

  //===========================================================
  if (millis() - t_pin > 200)
  {
    t_pin = millis();
    adc_pin = analogRead(DO_PIN);
    pin = (adc_pin * 15 ) / (4095);
  }
  //===========================================================
  watchDogRefresh(); // reset Watchdog timer
  //======================== do nhiet do ======================
  nhiet_ntc();
  //===========================================================
  //========================== đọc tín hiệu hc165 ==============
  if (hc165() == 0xff)trang_thai_sac = 0;
  if (hc165() == 0xf7)trang_thai_sac = 1;
  if (trang_thai_sac == 0) // không sạc
  {
    bt1 = 0xef; bt2 = 0xdf; bt3 = 0xbf; bt4 = 0x7f; // không sạc
  }
  if (trang_thai_sac == 1) // CÓ sạc
  {
    bt1 = 0xe7; bt2 = 0xd7; bt3 = 0xb7; bt4 = 0x77; // có sac
  }
  if (hc165() == bt1)nut_nhan1 = 0;
  else nut_nhan1 = 1;
  if (hc165() == bt2)nut_nhan2 = 0;
  else nut_nhan2 = 1;
  if (hc165() == bt3 || hc165() == 0xbf)nut_nhan3 = 0;
  else nut_nhan3 = 1;
  if (hc165() == bt4)nut_nhan4 = 0;
  else nut_nhan4 = 1;
  //============================================== chon che do =============================
  if (digitalRead(button) == 0)
  {
    delay(300);
    digitalWrite(led1, 0);
    digitalWrite(rl1, 0);
    Serial.println("nhấn nhấn");
    Serial.println("ngủ ngủ");
    finger.LEDcontrol(FINGERPRINT_LED_OFF, 0, FINGERPRINT_LED_PURPLE);
    portENTER_CRITICAL_ISR(&mux);//Phần quan trọng cần được bảo vệ khỏi mọi truy cập đồng thời để thay đổi nó
    portEXIT_CRITICAL_ISR(&mux);//Cho phép tiếp tục chạy các task khác
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF); // Tắc bộ nhớ nhanh RTC.
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF); // Tắc bộ nhớ chậm RTC.
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF); // Tắc bộ dao động XTAL.
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF); //Tắc Số lượng miền.
    esp_deep_sleep_start();
  }
  //=================
  if (millis() - t_ngu > 1000 )
  {
    t_ngu = millis();
    dem_ngu++;
  }
  if (dem_ngu > 60)
  {
    //============ ngủ ====
    dem_ngu = 0;
    digitalWrite(rl1, 0);
    digitalWrite(led1, 0);
    finger.LEDcontrol(FINGERPRINT_LED_OFF, 0, FINGERPRINT_LED_PURPLE);
    Serial.println("ngủ ngủ");
    portENTER_CRITICAL_ISR(&mux);//Phần quan trọng cần được bảo vệ khỏi mọi truy cập đồng thời để thay đổi nó
    portEXIT_CRITICAL_ISR(&mux);//Cho phép tiếp tục chạy các task khác
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF); // Tắc bộ nhớ nhanh RTC.
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF); // Tắc bộ nhớ chậm RTC.
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF); // Tắc bộ dao động XTAL.
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF); //Tắc Số lượng miền.
    esp_deep_sleep_start();
  }
  //============================================== ở chế độ thêm thẻ =======================
  if (nut_nhan3 == 0)
  {
    if (millis() - t_nutnhan3 > 1000)
    {
      t_nutnhan3 = millis();
      dem_t_nutnhan3++;
    }
    if (dem_t_nutnhan3 > 3)
    {
      digitalWrite(buzzer, 1);
      delay(100);
      digitalWrite(buzzer, 0);
      delay(100);
      digitalWrite(buzzer, 1);
      delay(100);
      digitalWrite(buzzer, 0);
    }
  }
  if (dem_t_nutnhan3 > 3)
  {
    dem_nutnhan3 = 0;
    dem_ngu = 0;
    themthe();
    doc = 1;
    Serial.println("them");
  }
  //=============================================== ở chế độ doc the ====================
  if (demvtsai > 4 && doc == 0) //============= cảnh báo khi sai vân tay
  {
    //demvtsai = 0, tdemsai = 0, ttbuzzer = 0;
    if (millis() - tdemsai > 200)
    {
      tdemsai = millis();
      digitalWrite(buzzer, ttbuzzer);
      ttbuzzer = !ttbuzzer;
    }
  }
  if (demvtsai <= 4 && doc == 0)
  {
    docthe();
    getFingerprintID();
    Serial.println("doc");
  }
  //======================================= che do blutube dien thoai =====================
  /* if (nut_nhan3 == 0 && tt3 == 0) // nhấn nút 3 lần để vào chế độ
    {
     tt3 = 1;
     dem_nutnhan3++;
     digitalWrite(buzzer, 1);
     delay(50);
     digitalWrite(buzzer, 0);
     dem_t_nutnhan3 = 0;
    }
    if (nut_nhan3 == 1 && tt3 == 1)
    {
     tt3 = 0;
    }
    if (dem_nutnhan3 > 3)
    {
     Serial.println("blutube");
     doc = 1;
    }*/
  //======================================= cac che do nhap nhay den =====================
  if (denvantay == 0)finger.LEDcontrol(FINGERPRINT_LED_ON, 0, FINGERPRINT_LED_PURPLE);
  if (denvantay == 1)finger.LEDcontrol(FINGERPRINT_LED_ON, 0, FINGERPRINT_LED_BLUE);
  if (millis() - t1 > 100 && digitalRead(rl1) == 1) // nhap nnhay den khi bat relay
  {
    t1 = millis();
    digitalWrite(led1, ttled1);
    ttled1 = !ttled1;
  }
  if (millis() - t2 > 300 && doc == 1)
  {
    t2 = millis();
    digitalWrite(led1, ttled1);
    ttled1 = !ttled1;
  }
  if (pin >= 13.4 && pin < 15) // pin xanh
  {
    digitalWrite(led2, 1);
    digitalWrite(led3, 0);
    digitalWrite(led4, 0);
  }
  if (pin >= 11.8 && pin < 13.4) // pin vàng
  {
    digitalWrite(led2, 0);
    digitalWrite(led3, 1);
    digitalWrite(led4, 0);
  }
  if (pin >= 10 && pin < 11.8 || pin < 10) // pin đỏ
  {
    digitalWrite(led2, 0);
    digitalWrite(led3, 0);
    digitalWrite(led4, 1);
  }
  //Serial.println(pin);
}//=================================================== end loop ======================================

//================================================= van tay ==========================================
uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    //================ van tay sai cần thử lại ==============
    for (int i = 0; i < 3; i++)
    {
      finger.LEDcontrol(FINGERPRINT_LED_OFF, 0, FINGERPRINT_LED_RED);
      digitalWrite(buzzer, 1);
      delay(100);
      finger.LEDcontrol(FINGERPRINT_LED_ON, 0, FINGERPRINT_LED_RED);
      digitalWrite(buzzer, 0);
      delay(100);
    }
    demvtsai++;
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  digitalWrite(rl1, 1); // vân tay dúng
  denvantay = 1;
  finger.LEDcontrol(FINGERPRINT_LED_ON, 0, FINGERPRINT_LED_BLUE);
  digitalWrite(buzzer, 1);
  delay(1000);
  digitalWrite(buzzer, 0);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID); //Serial.print("83555");
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}
//===============================================
//=================================================================================
