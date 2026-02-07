# 開発環境
- Arduino IDE: 2.3.7
- ESP32 by Espressif Systems: 2.0.17（3.x系は，LovyanGFXがまだ対応していない可能性が高い．）
- Sensirion Core by Sensirion: 0.7.2
- Sensirion I2C SDP by Sensirion: 0.1.0
- LovyanGFX by lovyan03: 1.2.19

# 注意
- ESP32C3のパーティション設定を変更して，フラッシュメモリのプログラム領域APPを増やしています．（Arduino IDE > Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"）
