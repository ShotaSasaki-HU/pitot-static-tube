#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSdp.h>
#include <LovyanGFX.hpp> // v1.2.7

#define SDA_PIN 6
#define SCL_PIN 7
#define SDP810_500Pa_I2C_ADDRESS 0x25

// ESP32でLovyanGFXを独自設定で利用
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void) {
    { // バス制御の設定
        auto cfg = _bus_instance.config(); // バス設定用の構造体を取得

        // SPIバスの設定
        cfg.spi_host = SPI2_HOST;  // 使用するSPIを選択（ESP32-S2, C3: SPI2_HOST or SPI3_HOST）
        cfg.spi_mode = 0;          // SPI通信モードを設定（0 ~ 3）
        cfg.freq_write = 40000000; // 送信時のSPIクロック（最大80MHz, 80MHzを整数で割った値に丸められます）
        cfg.freq_read = 16000000;  // 受信時のSPIクロック
        cfg.spi_3wire = true;      // 受信をMOSIピンで行う場合はtrueを設定 = 受信線(MISO)を使わない設定？
        cfg.use_lock = true;       // トランザクションロックを使用する場合はtrueを設定
        cfg.dma_channel = SPI_DMA_CH_AUTO; // 使用するDMAチャンネルを設定
        cfg.pin_sclk = 8;
        cfg.pin_mosi = 10;
        cfg.pin_miso = -1; // -1 = disable
        cfg.pin_dc = 5;

        _bus_instance.config(cfg); // 設定値をバスに反映
        _panel_instance.setBus(&_bus_instance); // バスをパネルにセット
    }

    { // 表示パネル制御の設定
        auto cfg = _panel_instance.config(); // 表示パネル設定用の構造体を取得

        cfg.pin_cs           =     4; // CSが接続されているピン番号   (-1 = disable)
        cfg.pin_rst          =     3; // RSTが接続されているピン番号  (-1 = disable)
        cfg.pin_busy         =    -1; // BUSYが接続されているピン番号 (-1 = disable)

        cfg.panel_width      =   240; // 実際に表示可能な幅
        cfg.panel_height     =   240; // 実際に表示可能な高さ
        cfg.offset_x         =     0; // パネルのX方向オフセット量
        cfg.offset_y         =     0; // パネルのY方向オフセット量
        cfg.offset_rotation  =     0; // 回転方向の値のオフセット 0~7 (4~7は上下反転)
        cfg.dummy_read_pixel =     8; // ピクセル読出し前のダミーリードのビット数
        cfg.dummy_read_bits  =     1; // ピクセル以外のデータ読出し前のダミーリードのビット数
        cfg.readable         = false; // データ読出しが可能な場合 trueに設定
        cfg.invert           =  true; // パネルの明暗が反転してしまう場合 trueに設定
        cfg.rgb_order        = false; // パネルの赤と青が入れ替わってしまう場合 trueに設定
        cfg.dlen_16bit       = false; // 16bitパラレルやSPIでデータ長を16bit単位で送信するパネルの場合 trueに設定
        cfg.bus_shared       = false; // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

        _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance); // 使用するパネルをセット
  }
};

SensirionI2CSdp sdp; // センサのインスタンス

LGFX lcd; // ディスプレイのインスタンス
LGFX_Sprite sprite(&lcd); // スプライト（描画バッファ）

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

  lcd.init();
  lcd.setRotation(0); // 回転方向を 0～3 の4方向から設定します．（4～7を使用すると上下反転になります．）
  // 必要に応じてカラーモードを設定（初期値は16）
  // 16の方がSPI通信量が少なく高速に動作しますが，赤と青の諧調が5bitになります．
  // 24の方がSPI通信量が多くなりますが，諧調表現が綺麗になります．
  lcd.setColorDepth(16);

  // スプライト（画用紙）のメモリ確保
  sprite.createSprite(240, 240);
  // テキストサイズとフォント設定
  sprite.setTextSize(2);
  sprite.setTextColor(TFT_WHITE);
  // sprite.setFont(&fonts::Orbitron_Light_24); // カッコいいフォント例
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

    // 空気密度（kg/m^3）
    // http://sasaki.g1.xrea.com/powerpoint/vaporization-heat/03-Air-density.pdf
    float rho = 353.017 / (273.15 + real_temperature); // 気体の質量密度rho = (M * P) / (R * T) / 1000
    Serial.print("Rho: ");
    Serial.print(rho, 2); // 小数点2桁まで
    Serial.print(" kg/m^3\t");

    // 流速（km/h）
    float speed_kmh = 0.0;
    if (diff_pressure > 0) {
      float speed_ms = sqrt((2.0 * diff_pressure) / rho); // 流速U = sqrt(2 * dP / rho)
      speed_kmh = speed_ms * 3.6; // speed_ms * 60 * 60 / 1000
    }
    Serial.print("Speed: ");
    Serial.print(speed_kmh, 1); // 小数点1桁まで
    Serial.println(" km/h");
  } else {
    Serial.println("Error reading measurement.");
  }

  // --- 描画処理 ---

  // 1. 背景を塗りつぶす（消去）
  // sprite.fillScreen(lcd.color888(255, 0, 0)); // 綺麗な赤色
  sprite.fillScreen(lcd.color888(0, 255, 0)); // 黄緑？
  // sprite.fillScreen(lcd.color888(0, 0, 255)); // 綺麗な青色

  // 2. デザインを描く
  // 外周のリング
  sprite.drawCircle(120, 120, 118, lcd.color888(0, 0, 255));
  sprite.drawCircle(120, 120, 117, lcd.color888(0, 0, 255));
  
  // 中心に数値を表示
  sprite.setCursor(90, 100); // 位置合わせ
  sprite.printf("%4.2f", real_temperature);
  
  // 単位
  sprite.setCursor(90, 130);
  sprite.setTextSize(1);
  sprite.print("C");
  sprite.setTextSize(2); // 戻す

  // 3. 画面に転送（ここで初めて表示される）
  sprite.pushSprite(0, 0);

  delay(100); // 10Hz
}
