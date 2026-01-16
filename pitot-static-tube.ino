#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSdp.h>
#include <LovyanGFX.hpp> // v1.2.7
#include <WiFi.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define SDP810_500Pa_I2C_ADDRESS 0x25

class PitotStaticTube {
private:
  SensirionI2CSdp sdp; // センサのインスタンス
  float _diff_pressure_pa = 0.0;
  float _raw_temp_c = 0.0;
  float _calibrated_temp_c = 0.0;
  float _speed_kmh = 0.0;

  float _temp_offset_c = -3.0; // 温度補正オフセット（自己発熱分）

public:
  PitotStaticTube() {}

  bool begin() {
    /**
     * @brief 初期化処理
     */
    sdp.begin(Wire, SDP810_500Pa_I2C_ADDRESS); // センサの初期化
    sdp.stopContinuousMeasurement(); // 念の為，一旦停止．
    // 計測開始（差圧・温度補正あり・平均化モード）
    uint16_t error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
    return (error == 0); // エラーが無ければtrue
  }

  void setTemperatureOffset(float offset) {
    /**
     * @brief 温度補正オフセット値を外部から変更するメソッド（ハードコードを忌避）
     */
    _temp_offset_c = offset;
  }

  bool update() {
    /**
     * @brief ReadとCalibrateとCalculateを行う更新メソッド
     * @return 成功ならtrue
     */
    uint16_t error = sdp.readMeasurement(_diff_pressure_pa, _raw_temp_c);
    if (error) { return false; }

    // 温度補正
    _calibrated_temp_c = _raw_temp_c + _temp_offset_c;

    // 物理計算

    return true;
  }

  void calcAirspeed() {
    /**
     * @brief 空気密度と対気速度の計算を行うメソッド
     */
    // 空気密度（kg/m^3）
    // 気体の質量密度rho = (M * P) / (R * T) / 1000
    // http://sasaki.g1.xrea.com/powerpoint/vaporization-heat/03-Air-density.pdf
    float rho = 353.017 / (273.15 + _calibrated_temp_c);

    // 流速（km/h）
    // 流速U = sqrt(2 * dP / rho)
    if (_diff_pressure_pa > 0) {
      float speed_ms = sqrt((2.0 * _diff_pressure_pa) / rho);
      _speed_kmh = speed_ms * 3.6; // speed_ms * 60 * 60 / 1000
    } else {
      _speed_kmh = 0.0;
    }
  }

  // Getters
  // constメソッドは，メンバ変数を書き換えられないため，副作用が無い．
  float getSpeedKmh() const { return _speed_kmh; }
  float getPressurePa() const { return _diff_pressure_pa; }
  float getCalibratedTemperatureC() const { return _calibrated_temp_c; }
};

// ESP32でLovyanGFXを独自設定で利用
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void) {
    {
      // バス制御の設定
      auto cfg = _bus_instance.config(); // バス設定用の構造体を取得

      // SPIバスの設定
      cfg.spi_host = SPI2_HOST;          // 使用するSPIを選択（ESP32-S2, C3: SPI2_HOST or SPI3_HOST）
      cfg.spi_mode = 0;                  // SPI通信モードを設定（0 ~ 3）
      cfg.freq_write = 40000000;         // 送信時のSPIクロック（最大80MHz, 80MHzを整数で割った値に丸められます）
      cfg.freq_read = 16000000;          // 受信時のSPIクロック
      cfg.spi_3wire = true;              // 受信をMOSIピンで行う場合はtrueを設定 = 受信線(MISO)を使わない設定？
      cfg.use_lock = true;               // トランザクションロックを使用する場合はtrueを設定
      cfg.dma_channel = SPI_DMA_CH_AUTO; // 使用するDMAチャンネルを設定
      cfg.pin_sclk = 8;
      cfg.pin_mosi = 10;
      cfg.pin_miso = -1; // -1 = disable
      cfg.pin_dc = 5;

      _bus_instance.config(cfg);              // 設定値をバスに反映
      _panel_instance.setBus(&_bus_instance); // バスをパネルにセット
    }

    {
      // 表示パネル制御の設定
      auto cfg = _panel_instance.config(); // 表示パネル設定用の構造体を取得

      cfg.pin_cs = 4;    // CSが接続されているピン番号   (-1 = disable)
      cfg.pin_rst = 3;   // RSTが接続されているピン番号  (-1 = disable)
      cfg.pin_busy = -1; // BUSYが接続されているピン番号 (-1 = disable)

      cfg.panel_width = 240;    // 実際に表示可能な幅
      cfg.panel_height = 240;   // 実際に表示可能な高さ
      cfg.offset_x = 0;         // パネルのX方向オフセット量
      cfg.offset_y = 0;         // パネルのY方向オフセット量
      cfg.offset_rotation = 0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      cfg.dummy_read_pixel = 8; // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits = 1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable = false;     // データ読出しが可能な場合 trueに設定
      cfg.invert = true;        // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order = false;    // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit = false;   // 16bitパラレルやSPIでデータ長を16bit単位で送信するパネルの場合 trueに設定
      cfg.bus_shared = false;   // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance); // 使用するパネルをセット
  }
};

// Z軸で回転可能なスプライトを扱うクラス
class RotatableSprite {
private:
  LGFX_Sprite _sprite;  // 針の画像を保持するスプライト
  float _current_angle; // 現在の角度（度数法）

  // static: クラス全体で1個のみ．
  // constexpr: コンパイル時に値が決定する定数．ベタ書きの数値リテラルと同じで，命令に埋め込まれる．
  static constexpr uint32_t TRANSPARENT = 0x000000; // 透過色を黒に設定

public:
  // コンストラクタ
  RotatableSprite(LGFX* lgfx, int16_t w, int16_t h) // wとhはスプライト自身の大きさ
    // 初期化リスト（代入ではなく生成時に初期化できる．）
    : _sprite(lgfx),
      _current_angle(0) // 元の向きからの相対角度
  {
    _sprite.setColorDepth(16);
    _sprite.createSprite(w, h);
    _sprite.setPivot(w >> 1, h >> 1); // スプライト自身の中央で回転
  }

  // 描画ロジックを関数として受け取る．
  // 引数として「自分のスプライトのポインタ」を渡すことで，外部関数が書き込めるようにする．
  using DrawCallBack = std::function<void(LGFX_Sprite*)>;

  // スプライトを描画するメソッド
  void createSpriteImage(DrawCallBack drawFunc) {
    _sprite.fillScreen(TRANSPARENT); // 一旦クリア
    if (drawFunc) {
      drawFunc(&_sprite); // 注入された描画ロジックを実行
    }
  }

  // 回転中心をずらすメソッド（デフォルトはスプライトの中央）
  void setPivot(int16_t x, int16_t y) { _sprite.setPivot(x, y); }

  // 角度の更新メソッド（即時反映）
  void setAngle(float angle) {
    _current_angle = angle; // 元の向きからの相対角度
  }

  // メインキャンバスへの描画
  void draw(LGFX_Sprite* canvas, int32_t x, int32_t y) {
    _sprite.pushRotateZoom(
      canvas,         // dst
      x,              // dst_x
      y,              // dst_y
      _current_angle, // angle (deg)
      1.0f,           // zoom_x
      1.0f,           // zoom_y
      TRANSPARENT     // transp
    );
  }

  // Getters
  uint16_t width() const { return _sprite.width(); }
  uint16_t height() const { return _sprite.height(); }
};

class AirspeedIndicator {
private:
  // 部品
  RotatableSprite _speed_pointer;
  RotatableSprite _vmo_pointer;
  // RollingDial _digital_airspeed;
  // RollingDial _digital_battery;

  // 設定値
  float _max_scale_kmh;
  float _max_operating_airspeed;
  float _max_angle; // 上向き0度で時計回り

public:
  AirspeedIndicator(LGFX* lgfx)
    : _speed_pointer(lgfx, 8, 102),
      _vmo_pointer(lgfx, 11, 102)
  {
    _max_scale_kmh = 60.0;
    _max_operating_airspeed = 35.0;
    _max_angle = 330.0;

    // Speed Pointer
    _speed_pointer.setPivot(_speed_pointer.width() >> 1, _speed_pointer.height());
    _speed_pointer.createSpriteImage([](LGFX_Sprite* sp) {
      int w = sp->width();
      int h = sp->height();

      sp->fillTriangle(w >> 1, 0, w, 16, 0, 16, sp->color888(253, 231, 184));
      sp->fillRect(0, 16, 8, 20, sp->color888(253, 231, 184));
      sp->fillRect(2.5, 36, 3, 66, sp->color888(253, 231, 184));
    });
  }

  void update(float raw_speed_kmh, float dt_s) {
    // Speed Pointer（生値に即応）
    float clamped_speed = constrain(raw_speed_kmh, 0.0, _max_scale_kmh);
    float angle = mapFloat(clamped_speed, 0.0, _max_scale_kmh, 0.0, _max_angle);
    _speed_pointer.setAngle(angle);

    // Vmo Pointer

    // Digital Airspeed

    // Digital Battery
  }

  void draw(LGFX_Sprite* canvas) {
    int center_x = canvas->width() >> 1;
    int center_y = canvas->height() >> 1;

    // Background
    canvas->fillScreen(canvas->color888(28, 26, 29));

    // Digital Airspeed

    // Digital Battery

    // Speed Pointer
    _speed_pointer.draw(canvas, center_x, center_y);

    // Vmo Pointer

    // Circle
    canvas->fillCircle(center_x, center_y, 14, canvas->color888(62, 51, 45));
  }

  // 浮動小数点のmap関数
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

PitotStaticTube sensor;

LGFX lcd; // ディスプレイのインスタンス
LGFX_Sprite canvas(&lcd); // 描画バッファ
AirspeedIndicator indicator(&lcd);

unsigned long prev_time_ms = 0;

void setup() {
  // 無線機能を明示的にOFF（発熱対策・省電力）
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.begin(115200); // シリアル通信開始
  while (!Serial) {
    delay(100); // シリアル接続待ち
  }

  Wire.begin(SDA_PIN, SCL_PIN); // I2Cの初期化
  Wire.setClock(50000); // 50kHz（安定性重視で低め）

  if (!sensor.begin()) {
    Serial.println("Sensor Init Failed!");
    while(1) { delay(100); }
  }

  lcd.init();
  lcd.setRotation(0); // 回転方向を 0～3 の4方向から設定します．（4～7を使用すると上下反転になります．）
  lcd.setColorDepth(16);
  canvas.createSprite(lcd.width(), lcd.height());
}

void loop() {
  // 時間管理
  unsigned long current_time_ms = millis();
  float dt_s = (current_time_ms - prev_time_ms) / 1000.0;
  prev_time_ms = current_time_ms;

  if (sensor.update()) {
    Serial.print("Press: "); Serial.print(sensor.getPressurePa()); Serial.print(" Pa\t");
    Serial.print("Temp: ");  Serial.print(sensor.getCalibratedTemperatureC()); Serial.print(" C\t");
    Serial.print("Speed: "); Serial.print(sensor.getSpeedKmh());   Serial.println(" km/h");
  } else {
    Serial.println("Sensor Read Error!");
  }
  
  // --- 描画処理 ---
  indicator.update(sensor.getCalibratedTemperatureC(), dt_s);
  indicator.draw(&canvas);
  canvas.pushSprite(0, 0); // 転送

  delay(100);
}
