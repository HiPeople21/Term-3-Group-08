#include "Wire.h"
#include "MFRC522_I2C.h"

MFRC522_I2C mfrc522(0x28, -1, &Wire1);  // Create MFRC522 instance.  创建MFRC522实例

void setup()
{
  Serial.begin(9600);
//   while (!Serial);
    // M5.begin();             // Init M5Stack.  初始化M5Stack
    // M5.Power.begin();       // Init power  初始化电源模块
    // M5.lcd.setTextSize(2);  // Set the text size to 2.  设置文字大小为2
    // M5.Lcd.println("MFRC522 Test");
    Wire1.begin();  // Wire init, adding the I2C bus.  Wire初始化, 加入i2c总线

    mfrc522.PCD_Init();  // Init MFRC522.  初始化 MFRC522
    Serial.println("Please put the card\n\nUID:");
}

void loop()
{
    // M5.Lcd.setCursor(40, 47);
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {  // 如果没有读取到新的卡片
        delay(200);
        return;
    }
    // M5.Lcd.fillRect(42, 47, 320, 20, BLACK);
    for (byte i = 0; i < mfrc522.uid.size; i++) {  // Output the stored UID data.  将存储的UID数据输出
        Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    // M5.Lcd.println("");
    Serial.println("");
}