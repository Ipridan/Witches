#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <ESP32Servo.h>

#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

#include <SPI.h>
#include <MFRC522.h>

// ================== TB6612 ==================
#define AIN1 26
#define AIN2 27
#define BIN1 14
#define BIN2 12
#define PWMA 25
#define PWMB 33
#define STBY 13

int speedMotor = 150;

//================== SERVO ==================
Servo servo1;
Servo servo2;

#define SERVO1_PIN 17
#define SERVO2_PIN 15

// ================== RFID ==================
#define SS_PIN 21
#define RST_PIN 22
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

MFRC522 mfrc522(SS_PIN, RST_PIN);

// ================== LED ==================
#define LED_PIN 16
#define NUMPIXELS 1


byte* readAllNtagPages(int& returnedSize) {
  byte buffer[18];
  byte size = sizeof(buffer);
  MFRC522::StatusCode status;
  status = mfrc522.MIFARE_Read(3, buffer, &size);
  int totalPages = 45;
  returnedSize = totalPages * 4;
  byte* dynamicBuffer = (byte*)malloc(returnedSize);

  for (byte page = 0; page < totalPages; page += 4) {
    status = mfrc522.MIFARE_Read(page, buffer, &size);
    if (status == MFRC522::STATUS_OK) {
      
      for (byte offset = 0; offset < 4; offset++) {
        byte currentPage = page + offset;
        if (currentPage >= totalPages) break;
        int targetIndex = currentPage * 4;
        memcpy(&dynamicBuffer[targetIndex], &buffer[offset * 4], 4);
      }
    } else {
      Serial.printf("  Ошибка чтения страниц %d-%d: %s\n", page, page + 3, mfrc522.GetStatusCodeName(status));
      free(dynamicBuffer);
      returnedSize = 0;
      return nullptr;
    }
  }
  return dynamicBuffer;
}
String getColor() {
  // 1. Пытаемся найти новую карту или "разбудить" уже лежащую в поле (WUPA)
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  // Вместо PICC_IsNewCardPresent используем WakeupA
  MFRC522::StatusCode status = mfrc522.PICC_WakeupA(bufferATQA, &bufferSize);
  
  // Если не удалось разбудить и не удалось найти новую — выходим
  if (status != MFRC522::STATUS_OK) {
    if (!mfrc522.PICC_IsNewCardPresent()) {
      return "error";
    }
  }

  // 2. Читаем серийный номер
  if (!mfrc522.PICC_ReadCardSerial()) {
    return "error";
  }

  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  if (piccType != MFRC522::PICC_TYPE_MIFARE_UL) {
    Serial.println("Это не NTAG/Ultralight метка!");
    mfrc522.PICC_HaltA();
    delay(2000);
    return "error";
  }

  int dataSize = 0;
  byte* tagData = readAllNtagPages(dataSize);
  int indexT = -1;

  for (int i = 25; i < dataSize; i++) {
    if (tagData[i] == 0x54) {
      indexT = i;
      break;
    }
  }

  int textRecordLength = (int)tagData[indexT - 1];
  int langLength = (int)(tagData[indexT + 1] & 0x3F);
  int textStartStartIndex = indexT + 2 + langLength;
  int textLength = textRecordLength - (1 + langLength);

  String decodedMessage = "";
  for (int i = 0; i < textLength; i++) {
    decodedMessage += (char)tagData[textStartStartIndex + i];
  }

  free(tagData);
  mfrc522.PICC_HaltA();
  // Даем время на остановку
  
  // Сбрасываем состояние карты для следующего чтения
  mfrc522.PCD_StopCrypto1();
  return decodedMessage;
}

byte targetUID[7] = { 0x04, 0xFA, 0x31, 0x7A, 0xC1, 0x2A, 0x81 };

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

BLECharacteristic *pRXCharacteristic;
BLECharacteristic *pTXCharacteristic;
String command = "";

// UUID
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// BLE callback
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    command = pCharacteristic->getValue().c_str();
    command.trim();

    Serial.println("Получена команда: ");
    Serial.println(command);
  }
};

// ================== MOTOR FUNCTIONS ==================
void motorStop() {
  // Отключаем движение (coast)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, speedMotor);
  analogWrite(PWMB, speedMotor);
}

void motorForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, speedMotor);
  analogWrite(PWMB, speedMotor);
}

void motorBackward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, speedMotor);
  analogWrite(PWMB, speedMotor);
}

void turnLeft() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);  // левый мотор назад
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);  // правый мотор вперед

  analogWrite(PWMA, speedMotor);
  analogWrite(PWMB, speedMotor);
}

void turnRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);  // левый вперед
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);  // правый назад

  analogWrite(PWMA, speedMotor);
  analogWrite(PWMB, speedMotor);
}

// ================== RFID ==================
void scanRFID() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    pTXCharacteristic->setValue("Error: no card");
    pTXCharacteristic->notify();
    delay(50);
    return;
  }

  if (!mfrc522.PICC_ReadCardSerial()) {
    pTXCharacteristic->setValue("Error: no read");
    pTXCharacteristic->notify();
    delay(50);
    return;
  }

  bool match = true;

  // Проверка совпадения UID
  if (mfrc522.uid.size != sizeof(targetUID)) {
    match = false;
  } else {
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      if (mfrc522.uid.uidByte[i] != targetUID[i]) {
        match = false;
        break;
      }
    }
  }

  if (match) {
    // фиолетовый цвет
    pixels.setPixelColor(0, pixels.Color(255, 0, 255));
    pixels.show();
    delay(3000);
  } else {
    // желтый цвет
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
    delay(3000);

    // Вывод UID

    pTXCharacteristic->setValue("UID: ");
    pTXCharacteristic->notify();
    delay(50);
    for (byte i = 0; i < mfrc522.uid.size; i++) {

      byte segment = mfrc522.uid.uidByte[i];

      pTXCharacteristic->setValue(&segment, 1);
      pTXCharacteristic->notify();
      delay(50);
      pTXCharacteristic->setValue(" ");
      pTXCharacteristic->notify();
      delay(50);
    }
    pTXCharacteristic->setValue("\n");
    pTXCharacteristic->notify();
    delay(50);
  }

  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();

  //mfrc522.PICC_HaltA();
}

void updateLED(String color) {
  if (color == "red")        {pixels.setPixelColor(0, pixels.Color(255, 0, 0)); 
   pixels.show();
   delay(1500);
   pixels.setPixelColor(0, pixels.Color(0, 0, 0));
   pixels.show();}
  else if (color == "blue")   {pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
   pixels.show();
   delay(1500);
   pixels.setPixelColor(0, pixels.Color(0, 0, 0));
   pixels.show();}
  else if (color == "green")  {pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
   pixels.show();
   delay(1500);
   pixels.setPixelColor(0, pixels.Color(0, 0, 0));
   pixels.show();}
  else if (color == "yellow") {pixels.setPixelColor(0, pixels.Color(255, 255, 0)); 
   pixels.show();
   delay(1500);
   pixels.setPixelColor(0, pixels.Color(0, 0, 0));
   pixels.show();}
  else if (color == "white")  {pixels.setPixelColor(0, pixels.Color(255, 255, 255)); 
   pixels.show();
   delay(1500);
   pixels.setPixelColor(0, pixels.Color(0, 0, 0));
   pixels.show();}
  else if (color == "orange") {pixels.setPixelColor(0, pixels.Color(255, 100, 0)); 
   pixels.show();
   delay(1500);
   pixels.setPixelColor(0, pixels.Color(0, 0, 0));
   pixels.show();}
  else if (color == "purple") {pixels.setPixelColor(0, pixels.Color(128, 0, 128));   
  pixels.show();
  delay(1500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
   pixels.show();}
  else if (color == "black") {pixels.setPixelColor(0, pixels.Color(255, 255, 255));   
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));   
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));   
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));   
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));   
  pixels.show();}

}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  pinMode(LED_PIN, OUTPUT);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  servo1.write(0);
  servo2.write(0);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  mfrc522.PCD_Init();

  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();

  motorStop();

  // BLE init
  BLEDevice::init("ESP32_Robot");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTXCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  pTXCharacteristic->addDescriptor(new BLE2902());

  pRXCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

  pRXCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  delay(50);
  Serial.println("Ожидание BLE команды...");
}

// ================== LOOP ==================
void loop() {
  if (command != "") {
    if (command == "!B10;") {
      delay(20);
      servo1.write(45);
      delay(400);
      updateLED(getColor());
    } else if (command == "!B309") {
      delay(20);
      servo1.write(0);
      delay(20);
    } else if (command == "!B20:") {
      delay(20);
      servo2.write(100);
      delay(20);
    } else if (command == "!B408") {
      delay(20);
      servo2.write(0);
      delay(20);
    } else if (command == "!B516")
     {delay (50); motorForward();}
    else if (command == "!B615") {delay (50); motorBackward();}
    else if (command == "!B714") {delay (50); turnLeft();}
    else if (command == "!B813") {delay (50); turnRight();}
    else if (command == "!B507" | command == "!B804" | command == "!B705" | command == "!B606") motorStop();
    // pTXCharacteristic->setValue(command);
    // pTXCharacteristic->notify();
    // delay(50);
    // pTXCharacteristic->setValue("OK");
    // pTXCharacteristic->notify();
    command = "";
  }
  // delay(20);
}