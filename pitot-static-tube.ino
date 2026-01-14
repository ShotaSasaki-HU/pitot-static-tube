#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSdp.h>

SensirionI2CSdp sdp; // センサオブジェクトの作成

#define SDA_PIN 6
#define SCL_PIN 7
#define SDP810_500Pa_I2C_ADDRESS 0x25

void setup() {
  Serial.begin(115200); // シリアル通信開始
  while (!Serial) {
    delay(100); // シリアル接続待ち
  }

  Wire.begin(SDA_PIN, SCL_PIN); // I2Cの初期化

  sdp.begin(Wire, SDP810_500Pa_I2C_ADDRESS); // センサの初期化
  sdp.stopContinuousMeasurement(); // 念の為，一旦停止．

  // 計測開始（差圧・温度補正あり・平均化モード）
  uint16_t error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
  if (error) {
    char error_message[256];
    errorToString(error, error_message, 256);
    Serial.print("Error initializing SDP810: ");
    Serial.println(error_message);
    while (1) { delay(1000); } // エラーならここで停止
  }

  Serial.println("SDP810 Initialized successfully!");
  Serial.println("------------------------------------------------");
}

void loop() {
  float diff_pressure = 0.0;
  float temperature = 0.0;
  float real_temperature = 0.0;

  // センサから値を読み取る．
  uint16_t error = sdp.readMeasurement(diff_pressure, temperature);

  if (!error) {
    // 差圧（Pascal）
    Serial.print("Press: ");
    Serial.print(diff_pressure, 2); // 小数点2桁まで
    Serial.print(" Pa\t");

    // 温度（Celsius）
    real_temperature = temperature - 3.0; // センサの自己発熱を3.0°Cと仮定（2026年1月15日3時58分）
    Serial.print("Temp: ");
    Serial.print(real_temperature, 2); // 小数点2桁まで
    Serial.print(" °C\t");

    // 流速（km/h）
    // U = sqrt(2 * dP / rho)
    float speed_kmh = 0.0;
    float rho = 1.2; // 空気密度（kg/m^3）
    if (diff_pressure > 0) {
      float speed_ms = sqrt((2.0 * diff_pressure) / rho);
      speed_kmh = speed_ms * 3.6; // speed_ms * 60 * 60 / 1000
    }

    Serial.print("Speed: ");
    Serial.print(speed_kmh, 1); // 小数点1桁まで
    Serial.println(" km/h");
  } else {
    Serial.println("Error reading measurement.");
  }

  delay(100); // 10Hz
}
