# 開発環境
- Arduino IDE: 2.3.7
- ESP32 by Espressif Systems: 3.2.0（3.2.1以降は，2026年1月16日時点でLovyanGFX v1.2.7が対応していないようです．）
- Sensirion Core by Sensirion: 0.7.2
- Sensirion I2C SDP by Sensirion: 0.1.0
- LovyanGFX by lovyan03: 1.2.7

# 注意
- ESP32C3のパーティション設定を変更して，フラッシュメモリのプログラム領域APPを増やしています．（Arduino IDE > Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"）
