#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSdp.h>
#include <LovyanGFX.hpp>
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
    calcAirspeed();

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
      cfg.pin_sclk = 8;  // ディスプレイの"SCL"
      cfg.pin_mosi = 10; // ディスプレイの"SDA"
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

class RotatingDials {
private:
  LGFX_Sprite _sprite; // 自分専用の描画領域

  int _width;           // 幅
  int _normal_height;   // 標準の高さ
  int _tall_height;     // 高さ（大きい方）
  uint16_t _tall_flags; // 高さを大きくする桁のフラグ（2進数）
  
  int _digits_int; // 整数部の桁数
  int _digits_dec; // 小数部の桁数

  bool _show_decimal_point = true; // 小数点表示の有無

  // アニメーション用状態
  float _current_val;        // 現在表示中の値
  float _max_change_per_sec; // 追従制限

  // static: クラス全体で1個のみ．
  // constexpr: コンパイル時に値が決定する定数．ベタ書きの数値リテラルと同じで，命令に埋め込まれる．
  static constexpr uint32_t TRANSPARENT = 0x000000; // 透過色を黒に設定

public:
  RotatingDials(
    LGFX* lgfx,
    int width, int normal_height, uint16_t tall_flags,
    int digits_int, int digits_dec,
    float max_change_per_sec,
    bool show_decimal_point
  )
    : _sprite(lgfx),

      _width(width),
      _normal_height(normal_height),
      _tall_height(normal_height * 5 / 3),
      _tall_flags(tall_flags),

      _digits_int(digits_int),
      _digits_dec(digits_dec),

      _show_decimal_point(show_decimal_point),

      _max_change_per_sec(max_change_per_sec)
  {
    _sprite.setColorDepth(16);
    _sprite.createSprite(_width, _tall_height);

    // テキスト描画の設定
    _sprite.setTextColor(TFT_WHITE); // 第1引数：文字色，第2引数：背景色
    _sprite.setTextSize(0.7); // 倍率
    _sprite.setTextDatum(textdatum_t::middle_center); // 基準点（Datum）
    // _sprite.setFont(&fonts::Font4);
    // _sprite.setFont(&fonts::Orbitron_Light_24); // SFチックなフォント（32も可）．drawCharだとダメ，drawStringならOK．
    _sprite.setFont(&fonts::FreeSans18pt7b);
  }

  void update(float target_val, float dt_s) {
    /**
     * @brief 追従制限ありの状態で現在値を更新するメソッド
     */
    float diff = target_val - _current_val;
    float step = _max_change_per_sec * dt_s;

    if (abs(diff) < step) {
      _current_val = target_val; // 十分近ければ即時反映
    } else {
      _current_val += (diff > 0 ? step : -step); // 追従制限にぶつかる場合は，最大限変化させる．
    }
  }

  // スプライトを描画するメソッド
  void createSpriteImage() {
    float dial_width = _width / (float)(_digits_int + _digits_dec);

    // ダイヤルの背景色を設定
    _sprite.fillScreen(_sprite.color888(44, 44, 44));

    // ドラム間の区切り線
    int digit_width = _width / (_digits_int + _digits_dec);
    for (int i = 1; i * digit_width <= _width; i++) {
      _sprite.drawLine(i * digit_width, 0, i * digit_width, _tall_height, _sprite.color888(1, 1, 1));
    }

    // _current_val（小数）を文字列に変換
    // 各桁をインデックスで取得できるようにするため．
    char buf[16];
    int digits_full = 0;
    // 以下の条件以外は，仕様外とみなして対応しない．
    if (_digits_int > 0 && _digits_dec > 0) {
      digits_full = _digits_int + 1 + _digits_dec + 1; // 整数の桁数 + 小数点 + 小数の桁数 + 隠れ値
    } else if (_digits_int > 0 && _digits_dec == 0) {
      digits_full = _digits_int + 1 + 1; // 整数の桁数 + 小数点 + 隠れ値
    }
    dtostrf(_current_val, digits_full, _digits_dec + 1, buf); // 第2引数は全幅ではなく，全幅の最低保証である事に注意．

    // bufの左端から描画していく．
    int n = 0; // ダイヤルのインデックスは，bufのインデックスiとは一致しない．
    for (int i = 0; buf[i+1] != 0; i++) {
      if (buf[i] == '.') {
        continue; // 小数点は飛ばすだけで，nは増やさない．
      }else if (buf[i] == ' ') {
        n++; // 空白はダイヤルを埋めるのでnを進める．
        continue;
      } else {
        // if (buf[i] == ' ') buf[i] = '0';

        // ダイヤルのインデックスについて範囲外アクセス防止（念の為）
        if (n >= _digits_int + _digits_dec) break;

        int x_main = (int)((dial_width * n) + (dial_width / 2));

        int y_visual_offset = 3; // 視覚的な中心とフォントの中心を揃えるためのオフセット
        int y_main = (_tall_height >> 1) + y_visual_offset;

        if (isShiftRequired(buf, i)) {
          char last = buf[strlen(buf) - 1]; // 最後尾の数字
          float coef = (last - '0') / 10.0; // C/C++では，'0'〜'9'が文字コード上で連続しているため減算できる．
          y_main += coef * _normal_height;  // 桁上がりによるシフト
        }

        // drawCharだとDatumの設定が無視される．
        _sprite.drawString(String((buf[i] - '0' + 1 + 10) % 10), x_main, y_main - _normal_height);
        _sprite.drawString(String(buf[i]), x_main, y_main);
        _sprite.drawString(String((buf[i] - '0' - 1 + 10) % 10), x_main, y_main + _normal_height);

        // 小数点の描画
        if (_show_decimal_point && n + 1 == _digits_int) {
          int x_dot = x_main + (dial_width / 2) - 2;
          int y_dot = (_tall_height >> 1) + y_visual_offset;
          _sprite.drawString(String('.'), x_dot, y_dot);
        }

        n++; // ダイヤルのインデックスを進める．
      }
    }

    // 白枠
    int x0, y0, x1, y1;
    bool is_tall_prev = false;
    for (int i = _digits_int + _digits_dec - 1; i >= 0 ; i--) {
      bool is_tall = (_tall_flags >> i) & 1;

      // 上の横線
      x0 = (_digits_int + _digits_dec - 1 - i) * dial_width;
      x1 = x0 + dial_width;
      if (is_tall) {
        y0 = 0;
        y1 = 0;
      } else {
        y0 = _normal_height / 3;
        y1 = _normal_height / 3;
      }
      _sprite.drawLine(x0, y0, x1, y1, _sprite.color888(255, 255, 255));

      // 下の横線
      if (is_tall) {
        y0 = _tall_height - 1; // 1ピクセル内側にずらさないとはみ出す．
        y1 = _tall_height - 1;
      } else {
        y0 = _normal_height * 4 / 3 - 1;
        y1 = _normal_height * 4 / 3 - 1;
      }
      _sprite.drawLine(x0, y0, x1, y1, _sprite.color888(255, 255, 255));

      if (i == _digits_int + _digits_dec - 1) {
        // 左端の縦線（無条件で最大長）
        _sprite.drawLine(0, 0, 0, _tall_height, _sprite.color888(255, 255, 255));
      } else if (i == 0) {
        // 右端の縦線（無条件で最大長）
        _sprite.drawLine(_width - 1, 0, _width - 1, _tall_height, _sprite.color888(255, 255, 255));
      }

      // 高さの増減に伴う縦線（増加でも減少でも書く線は同じ）
      if (is_tall != is_tall_prev && i != _digits_int + _digits_dec - 1) { // 左端のダイヤルはスキップ
        // 増減があれば，現在のダイヤルの左側上下に短い線を書く．
        _sprite.drawLine(x0, 0, x0, _normal_height / 3, _sprite.color888(255, 255, 255));
        _sprite.drawLine(x0, _normal_height * 4 / 3, x0, _tall_height, _sprite.color888(255, 255, 255));
      }

      is_tall_prev = is_tall;
    }

    // _tall_flagsによる表示領域の高さ調整
    for (int i = _digits_int + _digits_dec - 1; i >= 0 ; i--) {
      bool is_tall = (_tall_flags >> i) & 1;

      int w = dial_width;
      int h = _normal_height / 3;
      if (!is_tall) {
        int x = (_digits_int + _digits_dec - 1 - i) * dial_width;

        int y = 0;
        _sprite.fillRect(x, y, w, h, TRANSPARENT);

        y = _normal_height * 4 / 3;
        _sprite.fillRect(x, y, w, h, TRANSPARENT);
      }
    }
  }

  // メインキャンバスへの描画
  void draw(LGFX_Sprite* canvas, int32_t x, int32_t y) {
    _sprite.pushSprite(canvas, x, y, TRANSPARENT);
  }

private:
  bool isShiftRequired(const char* buf, int i) {
    /**
     * @brief buf中のインデックスで指定した数字に桁上がりのシフトが必要か判定するメソッド
     */
    for (int j = i + 1; buf[j+1] != '\0'; j++) {
      if (buf[j] == '.') continue;
      if (buf[j] != '9') return false;
    }
    return true;
  }
};

class AirspeedIndicator {
private:
  // 部品
  RotatableSprite _speed_pointer;
  RotatableSprite _vmo_pointer;
  RotatingDials _digital_airspeed;
  RotatingDials _digital_battery;

  // 設定値
  float _max_operating_airspeed;
  float _max_angle; // 上向き0度で時計回り
  float _max_scale_kmh;
  float _major_tick_interval_kmh; // 主目盛の間隔
  float _minor_tick_interval_kmh; // 副目盛の間隔
  float _number_interval_kmh; // 目盛の数字を表示する間隔

public:
  AirspeedIndicator(LGFX* lgfx)
    : _speed_pointer(lgfx, 8, 102),
      _vmo_pointer(lgfx, 11, 102),
      _digital_airspeed(lgfx, 66, 27, 0b001, 2, 1, 10.0, true),
      _digital_battery(lgfx, 66, 27, 0b000, 3, 0, 10.0, false)
  {
    _max_operating_airspeed = 40.0;
    _max_angle = 335.0;
    _max_scale_kmh = 50.0;
    _minor_tick_interval_kmh = 1.0;
    _major_tick_interval_kmh = 5.0 * _minor_tick_interval_kmh;
    _number_interval_kmh = 1.0 * _major_tick_interval_kmh;

    // Speed Pointer
    _speed_pointer.setPivot(_speed_pointer.width() >> 1, _speed_pointer.height());
    _speed_pointer.createSpriteImage([](LGFX_Sprite* sp) {
      int w = sp->width();
      // int h = sp->height();

      sp->fillTriangle(w >> 1, 0, w, 16, 0, 16, sp->color888(253, 231, 184));
      sp->fillRect(0, 16, w, 20, sp->color888(253, 231, 184));
      sp->fillRect(2.5, 36, 3, 66, sp->color888(253, 231, 184));
    });

    // Vmo Pointer
    _vmo_pointer.setPivot(_vmo_pointer.width() >> 1, _vmo_pointer.height());
    _vmo_pointer.createSpriteImage([](LGFX_Sprite* sp) {
      int w = sp->width();
      // int h = sp->height();

      sp->fillTriangle(w >> 1, 0, w, 21, 0, 21, sp->color888(226, 219, 219));
      sp->fillRect(0, 21, w, 81, sp->color888(226, 219, 219));

      sp->fillTriangle(2.8, 9.3, 9.7, 16, 0.5, 19, sp->color888(246, 6, 18));
      sp->fillTriangle(9.7, 16, w, 21, 0.5, 19, sp->color888(246, 6, 18));
      sp->fillTriangle(0.5, 19, w, 21, w, 30, sp->color888(246, 6, 18));

      sp->fillTriangle(0, 32, w, 43, 0, 44.5, sp->color888(246, 6, 18));
      sp->fillTriangle(0, 44.5, w, 43, w, 55, sp->color888(246, 6, 18));

      sp->fillTriangle(0, 56.3, w, 67, 0, 69.3, sp->color888(246, 6, 18));
      sp->fillTriangle(0, 69.3, w, 67, w, 80, sp->color888(246, 6, 18));

      sp->fillTriangle(0, 80, w, 90.8, 0, 90.8, sp->color888(246, 6, 18));
      sp->fillRect(0, 90.8, w, 11.2, sp->color888(246, 6, 18));
    });
    float angle = mapFloat(_max_operating_airspeed, 0.0, _max_scale_kmh, 0.0, _max_angle);
    _vmo_pointer.setAngle(angle);
  }

  void update(float raw_speed_kmh, float dt_s, float battery_level) {
    // Speed Pointer（生値に即応）
    float clamped_speed = constrain(raw_speed_kmh, 0.0, _max_scale_kmh);
    float angle = mapFloat(clamped_speed, 0.0, _max_scale_kmh, 0.0, _max_angle);
    _speed_pointer.setAngle(angle);

    // Digital Airspeed
    _digital_airspeed.update(raw_speed_kmh, dt_s);
    _digital_airspeed.createSpriteImage();

    // Digital Battery
    _digital_battery.update(battery_level, dt_s);
    _digital_battery.createSpriteImage();
  }

  void draw(LGFX_Sprite* canvas) {
    int center_x = canvas->width() >> 1;
    int center_y = canvas->height() >> 1;

    // 背景は静的なスプライトに保存しておきたかったが，おそらくRAMが不足して実現できない．

    // Background
    canvas->fillScreen(canvas->color888(28, 26, 29));

    // Tick Marks
    float r = 102.0;
    for (int i = 0; i <= _max_scale_kmh; i += _major_tick_interval_kmh) {
      float angle = mapFloat(float(i), 0.0, _max_scale_kmh, 0.0, _max_angle);
      angle -= 90.0; // canvasの座標上の単位円における角度
      angle *= DEG_TO_RAD; // 三角関数のために変換
      float x0 = (r * cos(angle)) + float(center_x);
      float y0 = (r * sin(angle)) + float(center_y);
      float x1 = ((r + 18.0) * cos(angle)) + float(center_x);
      float y1 = ((r + 18.0) * sin(angle)) + float(center_y);
      canvas->drawLine(x0, y0, x1, y1, canvas->color888(255, 255, 255));

      for (int j = i + _minor_tick_interval_kmh; j < (i + _major_tick_interval_kmh) && j <= _max_scale_kmh; j += _minor_tick_interval_kmh) {
        angle = mapFloat(float(j), 0.0, _max_scale_kmh, 0.0, _max_angle);
        angle -= 90.0; // canvasの座標上の単位円における角度
        angle *= DEG_TO_RAD; // 三角関数のために変換
        x0 = (r * cos(angle)) + float(center_x);
        y0 = (r * sin(angle)) + float(center_y);
        x1 = ((r + 9.0) * cos(angle)) + float(center_x);
        y1 = ((r + 9.0) * sin(angle)) + float(center_y);
        canvas->drawLine(x0, y0, x1, y1, canvas->color888(255, 255, 255));
      }
    }

    // 目盛の数字
    canvas->setTextColor(TFT_WHITE); // 第1引数：文字色，第2引数：背景色
    canvas->setTextSize(1.0); // 倍率
    canvas->setTextDatum(textdatum_t::middle_center); // 基準点（Datum）
    canvas->setFont(&fonts::FreeSans12pt7b);

    r = 85.0;
    for (int i = 0; i <= _max_scale_kmh; i += _number_interval_kmh) {
      float angle = mapFloat(float(i), 0.0, _max_scale_kmh, 0.0, _max_angle);
      angle -= 90.0; // canvasの座標上の単位円における角度
      angle *= DEG_TO_RAD; // 三角関数のために変換
      float x = (r * cos(angle)) + float(center_x);
      float y = (r * sin(angle)) + float(center_y);
      canvas->drawNumber(i, x, y);
    }

    // 単位の描画
    canvas->setTextColor(canvas->color888(43, 80, 145)); // 第1引数：文字色，第2引数：背景色
    canvas->setTextSize(0.7); // 倍率
    canvas->setTextDatum(textdatum_t::top_left); // 基準点（Datum）
    canvas->setFont(&fonts::FreeSans12pt7b);

    canvas->drawString("KMH", center_x - 33, center_y + 18); // "KMH"
    canvas->drawString("%", center_x + 36, center_y - 40); // '%'

    _digital_airspeed.draw(canvas, center_x - 33, center_y + 25);
    _digital_battery.draw(canvas, center_x - 33, center_y - 64);
    _vmo_pointer.draw(canvas, center_x, center_y);
    _speed_pointer.draw(canvas, center_x, center_y);
    canvas->fillCircle(center_x, center_y, 14, canvas->color888(62, 51, 45)); // 中央の円
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
  indicator.update(sensor.getSpeedKmh(), dt_s, 100.0);
  indicator.draw(&canvas);
  canvas.pushSprite(0, 0); // 転送

  delay(50);
}
