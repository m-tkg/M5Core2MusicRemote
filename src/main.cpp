// M5Core2 を Music Assistant のタッチリモコンにする。
// 画面: アルバムアート / 曲名・アーティスト(マーキー) / 音量スライダー(ドラッグ対応) / 操作アイコン
// 物理タッチボタン: 前の曲 / 再生・一時停止 / 次の曲 (向きに応じて役割を自動入替)
// IMU(加速度)で本体の向き(4方向)を検出し、画面を自動回転する。
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <algorithm>
#include <vector>

#include "config.h"

namespace {

// ---------- 定数 ----------
constexpr uint32_t kPollIntervalMs = 3000;   // 平常時のポーリング間隔
constexpr uint32_t kPollAfterCmdMs = 700;    // コマンド直後の再取得
constexpr uint32_t kDragSendMs = 400;        // ドラッグ中の volume_set 送信間隔
constexpr uint32_t kImuCheckMs = 300;        // 向きチェック間隔
constexpr uint32_t kImuStableMs = 800;       // 向き切替に必要な安定時間
constexpr uint32_t kLcdSleepTimeoutMs = 10UL * 60 * 1000;  // 停止状態がこの時間続いたら液晶オフ
constexpr int kHintSize = 22;                // 操作ボタンのタッチ判定半径

// ---------- 排他制御 (core0: networkTask/spinnerTask, core1: loop) ----------
// 共有変数の読み書きと M5.Display への描画呼び出し全てをこれで保護する。
// ネットワーク I/O 自体はロックの外で行い、結果を反映する短い区間だけロックする。
SemaphoreHandle_t stateMutex = nullptr;

struct StateLock {
  StateLock() { xSemaphoreTakeRecursive(stateMutex, portMAX_DELAY); }
  ~StateLock() { xSemaphoreGiveRecursive(stateMutex); }
};

// HTTP トランザクション (begin〜end) を全タスク横断で直列化する。
// core0 のポーリングと core1 のブラウズ取得が同時にソケットを扱うと
// レスポンスが乱れてパース失敗 (リスト空表示) するのを防ぐ。
// ロック順序の規約: httpMutex → stateMutex の順でのみネスト可。
// StateLock 保持中に HttpLock を取ってはならない (デッドロック防止)。
// 非再帰なので HttpLock 保持中に HttpLock を取る関数 (fetchArt /
// sendCommandBlocking / apiGetArray) を呼んではならない。
SemaphoreHandle_t httpMutex = nullptr;

struct HttpLock {
  HttpLock() { xSemaphoreTake(httpMutex, portMAX_DELAY); }
  ~HttpLock() { xSemaphoreGive(httpMutex); }
};

// loop() 側から networkTask へコマンド送信を委譲するためのキュー
struct NetCmdMsg {
  const char* command;  // 文字列リテラルのみを渡す前提 (静的寿命)
  String* args;         // networkTask 側で delete する
  bool showSpinner;
};
QueueHandle_t cmdQueue = nullptr;

// ---------- 状態 ----------
String playbackState = "unknown";  // playing / paused / idle
int volumeLevel = 0;
String trackTitle = "";
String trackArtist = "";
String imageUrl = "";
bool online = false;  // MA への直近の問い合わせが成功したか
bool shuffleEnabled = false;  // キューのシャッフル設定 (ポーリングで同期)
bool repeatFixSent = false;   // repeat_mode 矯正コマンドを送信済みか
bool artZoom = false;         // アートを全画面表示中か

// 再生位置。ポーリング間隔 (3秒) の間はローカルで秒読みして補間する (◀◀ の3秒しきい値判定に使用)
float elapsedBase = 0;
uint32_t elapsedCapturedAtMs = 0;

uint32_t nextPollAt = 0;
uint32_t messageId = 0;

// 液晶オートオフ
bool lcdSleeping = false;       // 液晶オフ中か
uint32_t nextLcdSleepAt = 0;    // この時刻までに再生が始まらなければ液晶オフにする

// 音量ドラッグ中の管理
bool dragging = false;
uint32_t lastDragSendAt = 0;

// 本体の向き (0=横・通常 / 1=横・逆さ / 2=縦・ボタン右 / 3=縦・ボタン左)
int orient = 0;
int candidateOrient = -1;
uint32_t candidateSince = 0;
uint32_t nextImuCheckAt = 0;
// 実測 (MPU6886): +Y=上端方向, +X=右端方向 (横・通常時)。setRotation との対応表
// (縦向きの 2 値は実機確認で決定: rotation 0/2 の向きは直感と逆だった)
constexpr int kRotFor[4] = {1, 3, 2, 0};

// アルバムアート (JPEG バッファ、imageproxy の size=256 を縮小描画)
uint8_t* artBuf = nullptr;
size_t artLen = 0;
String artUrlLoaded = "";

// 再描画フラグ (領域ごと)
bool dirtyContent = true;  // アート + 曲名
bool dirtyVolume = true;   // 音量 or 再生位置スライダー
bool dirtyHint = true;     // 操作アイコン (再生状態で変わる)
bool dirtyBattery = true;  // 右上のバッテリー + OFFLINE 表示

// ---------- レイアウト ----------
// 向きごとの描画ジオメトリ。上から音量スライダー → アート/曲名 → 操作ボタンの順
struct Geo {
  int W, H;
  bool portrait;
  int artX, artY, artSize;
  float artScale;              // 256px ソースからの縮小率
  int textX, textW;            // マーキー共通
  int titleY, artistY;
  int volX, volY, volW;        // スライダーバー (volY はバー上端)
  int volBtnW;                 // 音量 −/+ ボタンの幅 (高さはバーと同じ)
  int volMinusX, volPlusX;     // 音量 −/+ ボタンの左端 X (バーの左右に配置)
  int ctrlCy;                  // 操作ボタン行の中心 Y
  int ctrlSpacing;              // 操作ボタンの中心間隔
  int zoomSize, zoomX, zoomY;   // アート拡大表示のサイズ・位置
};
constexpr int kVolBarH = 26;
constexpr int kVolStep = 3;       // 音量 −/+ ボタン 1 タップの増減量
constexpr int kStatusRowH = 20;   // 右上バッテリー/OFFLINE 用に確保する上部の帯

Geo geo() {
  Geo g{};
  if (orient <= 1) {  // 横向き (180° 反転は setRotation が吸収するのでレイアウト共通)
    g.W = 320; g.H = 240;
    g.portrait = false;
    g.volBtnW = 32;
    g.volMinusX = 16;
    g.volX = g.volMinusX + g.volBtnW + 8;
    g.volW = g.W - 2 * g.volX;
    g.volPlusX = g.volX + g.volW + 8;
    g.volY = kStatusRowH + 6;
    g.artX = 8; g.artY = g.volY + kVolBarH + 6;
    g.artSize = 132; g.artScale = 132.0f / 256.0f;
    g.textX = g.artX + g.artSize + 10;
    g.textW = g.W - g.textX - 8;
    g.titleY = g.artY + 18;
    g.artistY = g.artY + 70;
    g.ctrlCy = 214;
    g.ctrlSpacing = 80;
    g.zoomSize = 220; g.zoomX = 50; g.zoomY = kStatusRowH;
  } else {  // 縦向き (要素が多く縦の余白が厳しいため、横向きよりアートを詰める)
    g.W = 240; g.H = 320;
    g.portrait = true;
    g.volBtnW = 32;
    g.volMinusX = 8;
    g.volX = g.volMinusX + g.volBtnW + 8;
    g.volW = g.W - 2 * g.volX;
    g.volPlusX = g.volX + g.volW + 8;
    g.volY = kStatusRowH + 6;
    g.artSize = 128; g.artScale = 128.0f / 256.0f;
    g.artX = (g.W - g.artSize) / 2; g.artY = g.volY + kVolBarH + 6;
    g.textX = 8;
    g.textW = g.W - 16;
    g.titleY = g.artY + g.artSize + 6;
    g.artistY = g.titleY + 44;  // LINE Seed 使用時の実際の行高 (~39px) 分の余裕を確保
    g.ctrlCy = 292;
    g.ctrlSpacing = 64;
    g.zoomSize = 240; g.zoomX = 0; g.zoomY = 60;
  }
  return g;
}

// ---------- 通信中スピナー ----------
// ネットワーク処理は別タスク (core0) で行うため、描画はすべて stateMutex で
// 排他制御する (loop() の描画 / spinnerTask の描画が衝突しないように)
bool spinnerActive = false;
int spinnerX = 160, spinnerY = 120;

void spinnerTask(void*) {
  int angle = 0;
  for (;;) {
    if (spinnerActive) {
      StateLock lock;
      if (spinnerActive) {  // ロック取得中に消えている可能性があるので再確認
        auto& d = M5.Display;
        d.fillCircle(spinnerX, spinnerY, 21, TFT_BLACK);
        d.fillArc(spinnerX, spinnerY, 18, 12, angle, angle + 110, TFT_CYAN);
        angle = (angle + 30) % 360;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(70));
  }
}

// 画面中央でスピナーを回し始める
void spinnerBegin() {
  Geo g = geo();
  spinnerX = g.W / 2;
  spinnerY = g.H / 2;
  spinnerActive = true;
}

// スピナーを止める。erase=true なら跡を消して通常画面の再描画を予約する
// (直後に全画面を描き直すフローでは erase=false でよい)
void spinnerEnd(bool erase) {
  StateLock lock;  // spinnerTask が描画中でないことを保証してから消す
  if (!spinnerActive) return;
  spinnerActive = false;
  if (erase) {
    M5.Display.fillCircle(spinnerX, spinnerY, 21, TFT_BLACK);
    dirtyContent = dirtyVolume = dirtyHint = true;
  }
}

// ---------- MA API ----------

// MA API に 1 コマンドを送る (呼び出し側で http.end() する)
// timeoutMs: 大きいプレイリスト取得はサーバ側の生成に 5 秒以上かかることがあるので
// ブラウズ系だけ長めを指定する
int postApi(HTTPClient& http, const char* command, const String& argsJson,
            uint32_t timeoutMs = 5000) {
  String url = String("http://") + MA_HOST + ":" + MA_PORT + "/api";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + MA_TOKEN);
  http.setTimeout(timeoutMs);
  uint32_t id;
  {
    StateLock lock;
    id = ++messageId;
  }
  String payload = String("{\"message_id\":\"") + id + "\",\"command\":\"" +
                   command + "\",\"args\":" + argsJson + "}";
  return http.POST(payload);
}

// レスポンスボディを PSRAM バッファへ読み切る。
// deserializeJson(http.getStream()) は内部の読み待ちが yield() スピンで、
// priority 0 の IDLE0 に CPU が渡らず task watchdog (5秒) で再起動してしまう。
// ここでは毎イテレーション delay(1) を挟んで確実に IDLE へ CPU を譲る
struct BodyBuf {
  char* data = nullptr;
  size_t len = 0;
  ~BodyBuf() { if (data) free(data); }
};

bool readBody(HTTPClient& http, BodyBuf& out, uint32_t timeoutMs) {
  int declared = http.getSize();  // Content-Length (-1 = 不明)
  size_t cap = declared > 0 ? static_cast<size_t>(declared) : 16 * 1024;
  out.data = static_cast<char*>(ps_malloc(cap + 1));
  if (!out.data) return false;
  WiFiClient* stream = http.getStreamPtr();
  uint32_t deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    if (declared > 0 && out.len >= static_cast<size_t>(declared)) break;
    if (out.len == cap) {  // Content-Length 不明時は伸長する
      size_t ncap = cap * 2;
      if (declared > 0 || ncap > 512 * 1024) break;
      char* nd = static_cast<char*>(ps_realloc(out.data, ncap + 1));
      if (!nd) return false;
      out.data = nd;
      cap = ncap;
    }
    int n = stream->read(reinterpret_cast<uint8_t*>(out.data) + out.len,
                         cap - out.len);
    if (n > 0) {
      out.len += n;
    } else if (!stream->connected() && !stream->available()) {
      break;  // 切断済みかつ残データなし = 本文終端
    }
    delay(1);
  }
  out.data[out.len] = '\0';
  return out.len > 0;
}

// 実際に HTTP を叩く (networkTask 上でのみ呼ぶ。ブロッキングして良い)
void sendCommandBlocking(const char* command, const String& argsJson, bool showSpinner) {
  if (showSpinner) spinnerBegin();
  int status;
  {
    HttpLock httpLock;
    HTTPClient http;
    status = postApi(http, command, argsJson);
    http.end();
  }
  if (showSpinner) spinnerEnd(true);
  Serial.printf("cmd %s -> %d\n", command, status);
  if (status >= 200 && status < 300) nextPollAt = millis() + kPollAfterCmdMs;
}

// loop() (core1) から呼ぶ。ブロッキングせず networkTask へキューイングするだけ
void sendCommand(const char* command, const String& argsJson) {
  // タップの反応が分かるよう通信中はスピナーを回す (音量ドラッグ中は煩いので出さない)
  NetCmdMsg msg{command, new String(argsJson), !dragging};
  if (xQueueSend(cmdQueue, &msg, 0) != pdTRUE) {
    Serial.println("cmdQueue full, dropping command");
    delete msg.args;
  }
}

// アルバムアートを取得する (imageproxy は認証不要。許可サイズの 256 を取得し縮小描画)
// networkTask 上でのみ呼ぶ。ダウンロード自体はロックせず、ローカルバッファに
// 取り込んでから短時間だけロックして artBuf を差し替える (drawContent() が
// 描画中の古いバッファを指したまま free されることを防ぐ)
void fetchArt() {
  if (imageUrl == artUrlLoaded) return;
  artUrlLoaded = imageUrl;

  {
    StateLock lock;
    if (artBuf) { free(artBuf); artBuf = nullptr; artLen = 0; }
    dirtyContent = true;
  }
  if (imageUrl.length() == 0) return;

  String url = imageUrl;
  url.replace("size=512", "size=256");

  uint8_t* buf = nullptr;
  size_t got = 0;
  int len = 0;
  {
    HttpLock httpLock;
    HTTPClient http;
    http.begin(url);
    http.setTimeout(8000);
    int status = http.GET();
    if (status != 200) {
      Serial.printf("art fetch failed: %d\n", status);
      http.end();
      return;
    }
    len = http.getSize();
    if (len <= 0 || len > 200 * 1024) {
      http.end();
      return;
    }
    buf = static_cast<uint8_t*>(ps_malloc(len));
    if (!buf) { http.end(); return; }
    WiFiClient* stream = http.getStreamPtr();
    uint32_t deadline = millis() + 8000;
    while (got < static_cast<size_t>(len) && millis() < deadline) {
      int n = stream->read(buf + got, len - got);
      if (n > 0) got += n;
      delay(1);  // 細切れ受信が続いても IDLE0 に CPU を譲る (task watchdog 対策)
    }
    http.end();
  }
  if (got == static_cast<size_t>(len)) {
    StateLock lock;
    artBuf = buf;
    artLen = got;
    dirtyContent = true;
  } else {
    free(buf);
  }
}

void setPlaybackState(const String& st) {
  StateLock lock;
  if (st == playbackState) return;
  bool wasPlaying = (playbackState == "playing");
  playbackState = st;
  dirtyHint = true;
  if (wasPlaying && playbackState != "playing") {
    nextLcdSleepAt = millis() + kLcdSleepTimeoutMs;  // 停止に転じたのでカウントダウン再開
  }
}

// players/get で状態を取得して画面状態を更新する。networkTask 上でのみ呼ぶ。
// HTTP/JSON パースはロックせずに行い、結果をローカル変数で受けてから
// 反映区間だけ短くロックする (loop() の描画を長時間ブロックしないため)
void pollStatus() {
  JsonDocument doc;
  {
    HttpLock httpLock;
    HTTPClient http;
    int status = postApi(http, "players/get",
                         String("{\"player_id\":\"") + MA_PLAYER_ID + "\"}");
    if (status < 200 || status >= 300) {
      http.end();
      StateLock lock;
      if (online) { online = false; dirtyBattery = true; }
      return;
    }

    // 必要なフィールドだけ取り出す (レスポンス全体は数KBある)
    JsonDocument filter;
    filter["playback_state"] = true;
    filter["volume_level"] = true;
    filter["elapsed_time"] = true;
    filter["current_media"]["title"] = true;
    filter["current_media"]["artist"] = true;
    filter["current_media"]["image_url"] = true;

    BodyBuf body;
    bool ok = readBody(http, body, 5000);
    http.end();
    DeserializationError err = DeserializationError::EmptyInput;
    if (ok) {
      err = deserializeJson(doc, static_cast<const char*>(body.data), body.len,
                            DeserializationOption::Filter(filter));
    }
    if (err) {
      Serial.printf("json error: %s\n", err.c_str());
      StateLock lock;
      if (online) { online = false; dirtyBattery = true; }
      return;
    }
  }

  String st = doc["playback_state"] | "idle";
  int vol = doc["volume_level"] | 0;
  String title = doc["current_media"]["title"] | "";
  String artist = doc["current_media"]["artist"] | "";
  String img = doc["current_media"]["image_url"] | "";
  float elapsed = doc["elapsed_time"] | 0.0f;
  uint32_t capturedAt = millis();

  {
    StateLock lock;
    if (!online) { online = true; dirtyBattery = true; }
    elapsedBase = elapsed;
    elapsedCapturedAtMs = capturedAt;
    setPlaybackState(st);  // 再帰ロック (同一タスクなので安全)
    // ドラッグ中はサーバー値で上書きしない (操作中の値を優先)
    if (!dragging && vol != volumeLevel) { volumeLevel = vol; dirtyVolume = true; }
    if (title != trackTitle || artist != trackArtist) {
      trackTitle = title;
      trackArtist = artist;
      dirtyContent = true;
    }
    imageUrl = img;
  }
  fetchArt();
}

// キューの shuffle/repeat_mode を取得し、shuffleEnabled に同期する。
// repeat_mode が "all" でなければ player_queues/repeat を一度だけ送って矯正する
// (dynamic queue では InvalidCommand が返るため、無限リトライしないようワンショットにする)。
// networkTask 上でのみ呼ぶ
void pollQueueSettings() {
  JsonDocument doc;
  {
    HttpLock httpLock;
    HTTPClient http;
    int status = postApi(http, "player_queues/get",
                         String("{\"queue_id\":\"") + MA_PLAYER_ID + "\"}");
    if (status < 200 || status >= 300) {
      http.end();
      return;
    }

    JsonDocument filter;
    filter["shuffle_enabled"] = true;
    filter["repeat_mode"] = true;

    BodyBuf body;
    bool ok = readBody(http, body, 5000);
    http.end();
    DeserializationError err = DeserializationError::EmptyInput;
    if (ok) {
      err = deserializeJson(doc, static_cast<const char*>(body.data), body.len,
                            DeserializationOption::Filter(filter));
    }
    if (err) {
      Serial.printf("json error: %s\n", err.c_str());
      return;
    }
  }

  bool shuffle = doc["shuffle_enabled"] | false;
  String repeatMode = doc["repeat_mode"] | "all";

  bool needFix;
  {
    StateLock lock;
    if (shuffle != shuffleEnabled) {
      shuffleEnabled = shuffle;
      dirtyHint = true;
    }
    if (repeatMode == "all") {
      repeatFixSent = false;
      needFix = false;
    } else {
      needFix = !repeatFixSent;
      if (needFix) repeatFixSent = true;
    }
  }
  if (needFix) {
    sendCommandBlocking("player_queues/repeat",
                        String("{\"queue_id\":\"") + MA_PLAYER_ID +
                            "\",\"repeat_mode\":\"all\"}", false);
  }
}

void sendVolume(int vol) {
  sendCommand("players/cmd/volume_set",
              String("{\"player_id\":\"") + MA_PLAYER_ID +
                  "\",\"volume_level\":" + vol + "}");
}

void sendSeek(int seconds) {
  sendCommand("players/cmd/seek",
              String("{\"player_id\":\"") + MA_PLAYER_ID +
                  "\",\"position\":" + seconds + "}");
}

// 現在の再生位置(秒)。直近ポーリング値をローカルの経過時間で補間する
float currentElapsedSeconds() {
  StateLock lock;
  float e = elapsedBase;
  if (playbackState == "playing") {
    e += (millis() - elapsedCapturedAtMs) / 1000.0f;
  }
  return e;
}

// ---------- 描画 ----------

// 曲名・アーティスト名・見出しに使う標準ビルトインフォント (M5GFX 内蔵の efontJA、ゴシック体)
const lgfx::IFont* titleFont() { return &fonts::efontJA_24; }
const lgfx::IFont* bodyFont() { return &fonts::efontJA_16; }
const lgfx::IFont* headerFont() { return &fonts::efontJA_16_b; }

// 長いテキストのマーキー (先頭で3秒停止 -> 流れ切るまでスクロール -> 先頭が右から流れ込む)
// テキストは setup() 時に一度だけ幅広スプライトへ描画し、tick() では
// そのスプライトを横にずらして viewport へコピーするだけにする
// (毎フレーム drawString をやり直すより軽い)
class Marquee {
 public:
  void setup(const String& text, const lgfx::IFont* font, uint16_t color,
             int x, int y, int w, int h) {
    text_ = text;
    font_ = font;
    color_ = color;
    x_ = x; y_ = y; w_ = w;
    // フォントの実際の行の高さ (ascent+descent) を問い合わせ、指定値より
    // 大きければそちらを使う (LINE Seed 等、想定より縦に大きいフォント対策)
    lgfx::FontMetrics fm;
    font_->getDefaultMetric(&fm);
    h_ = max(h, static_cast<int>(fm.height));
    offset_ = 0;

    viewport_.deleteSprite();
    viewport_.createSprite(w_, h_);

    // 一時キャンバスで文字幅を測ってから、その2周期分の幅広スプライトを作る
    M5Canvas measure(&M5.Display);
    measure.setFont(font_);
    measure.setTextSize(1);
    textW_ = measure.textWidth(text_);
    active_ = textW_ > w_;
    int period = textW_ + kGapPx;

    wide_.deleteSprite();
    wide_.createSprite(active_ ? period * 2 : textW_, h_);
    wide_.fillSprite(TFT_BLACK);
    wide_.setFont(font_);
    wide_.setTextSize(1);
    wide_.setTextColor(color_, TFT_BLACK);
    wide_.setTextDatum(top_left);
    wide_.drawString(text_, 0, 0);  // ここで SD からグリフを読む (1回だけ)
    if (active_) {
      wide_.drawString(text_, period, 0);  // 追いかけの2枚目
    }

    nextActionAt_ = millis() + kHeadPauseMs;
    draw();
  }

  void tick() {
    if (!active_ || millis() < nextActionAt_) return;
    offset_ += kStepPx;
    if (offset_ >= textW_ + kGapPx) {
      offset_ = 0;
      nextActionAt_ = millis() + kHeadPauseMs;
    } else {
      nextActionAt_ = millis() + kStepMs;
    }
    draw();
  }

 private:
  static constexpr uint32_t kHeadPauseMs = 3000;
  static constexpr uint32_t kStepMs = 33;
  static constexpr int kStepPx = 2;
  static constexpr int kGapPx = 60;  // 末尾と次の先頭の間隔

  // 幅広スプライトから viewport へずらしコピーするだけ (フォント描画・SD読込は無し)
  void draw() {
    viewport_.fillSprite(TFT_BLACK);  // 短いテキストで余白が残るケースに備えて毎回クリア
    wide_.pushSprite(&viewport_, -offset_, 0);
    viewport_.pushSprite(&M5.Display, x_, y_);
  }

  M5Canvas wide_;      // テキストを1〜2周期ぶん描画した幅広バッファ
  M5Canvas viewport_;  // 画面に表示する固定サイズの窓
  String text_;
  const lgfx::IFont* font_ = nullptr;
  uint16_t color_ = TFT_WHITE;
  int x_ = 0, y_ = 0, w_ = 0, h_ = 0;
  int textW_ = 0, offset_ = 0;
  bool active_ = false;
  uint32_t nextActionAt_ = 0;
};

Marquee titleMarquee;
Marquee artistMarquee;

// アルバムアート + 曲名 + アーティスト (+ オフライン表示)
void drawContent() {
  auto& d = M5.Display;
  Geo g = geo();
  int top = g.volY + kVolBarH + 6;
  d.fillRect(0, top, g.W, g.ctrlCy - kHintSize - top, TFT_BLACK);

  // アルバムアート
  if (artBuf && artLen > 0) {
    d.drawJpg(artBuf, artLen, g.artX, g.artY, g.artSize, g.artSize, 0, 0,
              g.artScale, g.artScale);
  } else {
    d.drawRoundRect(g.artX, g.artY, g.artSize, g.artSize, 8, TFT_DARKGREY);
    d.setFont(&fonts::FreeSans9pt7b);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setTextDatum(middle_center);
    d.drawString("no art", g.artX + g.artSize / 2, g.artY + g.artSize / 2);
  }

  // 曲名とアーティスト (長い場合はマーキーでスクロール)
  titleMarquee.setup(trackTitle.length() ? trackTitle : String("---"),
                     titleFont(), TFT_WHITE, g.textX, g.titleY, g.textW, 28);
  artistMarquee.setup(trackArtist, bodyFont(), TFT_SILVER, g.textX,
                      g.artistY, g.textW, 20);
}

// アルバムアートの全画面表示 (artZoom 中はこちらのみ描画し、曲名・スライダー・
// 操作ボタンは隠す)
void drawArtZoom() {
  auto& d = M5.Display;
  Geo g = geo();
  d.fillRect(0, kStatusRowH, g.W, g.H - kStatusRowH, TFT_BLACK);

  float scale = g.zoomSize / 256.0f;
  if (artBuf && artLen > 0) {
    d.drawJpg(artBuf, artLen, g.zoomX, g.zoomY, g.zoomSize, g.zoomSize, 0, 0,
              scale, scale);
  } else {
    d.drawRoundRect(g.zoomX, g.zoomY, g.zoomSize, g.zoomSize, 8, TFT_DARKGREY);
    d.setFont(&fonts::FreeSans9pt7b);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setTextDatum(middle_center);
    d.drawString("no art", g.zoomX + g.zoomSize / 2, g.zoomY + g.zoomSize / 2);
  }
}

// 右上のバッテリー残量(充電中は稲妻アイコン付き)と OFFLINE 表示
void drawBattery() {
  auto& d = M5.Display;
  Geo g = geo();
  d.fillRect(0, 0, g.W, kStatusRowH, TFT_BLACK);

  int level = M5.Power.getBatteryLevel();  // 0-100、取得不可なら負値
  bool charging = M5.Power.isCharging() == m5::Power_Class::is_charging;
  int cursor = g.W - 6;  // 右端から左へ積んでいく

  if (level >= 0) {
    // 50%未満は黄色、20%未満は赤で警告する
    uint16_t bodyColor = level < 20 ? TFT_RED : (level < 50 ? TFT_YELLOW : TFT_LIGHTGREY);

    constexpr int kBodyW = 18, kBodyH = 10, kNubW = 2;
    int bodyX = cursor - kBodyW - kNubW;
    int bodyY = (kStatusRowH - kBodyH) / 2;
    d.drawRect(bodyX, bodyY, kBodyW, kBodyH, bodyColor);
    d.fillRect(bodyX + kBodyW, bodyY + 2, kNubW, kBodyH - 4, bodyColor);
    int fillW = (kBodyW - 4) * constrain(level, 0, 100) / 100;
    if (fillW > 0) d.fillRect(bodyX + 2, bodyY + 2, fillW, kBodyH - 4, bodyColor);
    cursor = bodyX - 4;

    d.setFont(&fonts::Font0);  // 小さめの数字フォント
    d.setTextSize(1);
    d.setTextColor(bodyColor, TFT_BLACK);
    d.setTextDatum(middle_right);
    String pct = String(level) + "%";
    d.drawString(pct, cursor, kStatusRowH / 2);
    cursor -= d.textWidth(pct) + 4;

    if (charging) {
      constexpr int kBoltW = 9, kBoltH = 14;
      int bx = cursor - kBoltW;
      int by = (kStatusRowH - kBoltH) / 2;
      d.fillTriangle(bx + kBoltW, by, bx + 2, by + kBoltH / 2, bx + kBoltW / 2 + 1,
                     by + kBoltH / 2, TFT_YELLOW);
      d.fillTriangle(bx + kBoltW / 2 + 1, by + kBoltH / 2, bx + kBoltW - 2,
                     by + kBoltH / 2, bx, by + kBoltH, TFT_YELLOW);
      cursor = bx - 4;
    }
  }

  if (!online) {
    d.setFont(&fonts::FreeSansBold9pt7b);
    d.setTextColor(TFT_RED, TFT_BLACK);
    d.setTextDatum(top_right);
    d.drawString("OFFLINE", cursor, 2);
  }
}

void drawVolume() {
  auto& d = M5.Display;
  Geo g = geo();
  d.fillRect(g.volMinusX, g.volY - 6, g.volPlusX + g.volBtnW - g.volMinusX,
             kVolBarH + 12, TFT_BLACK);

  // バー左右の − / + ボタン
  auto drawVolBtn = [&](int bx, bool plus) {
    d.drawRoundRect(bx, g.volY, g.volBtnW, kVolBarH, 6, TFT_DARKGREY);
    int cx = bx + g.volBtnW / 2;
    int cy = g.volY + kVolBarH / 2;
    d.fillRect(cx - 6, cy - 1, 13, 3, TFT_LIGHTGREY);
    if (plus) d.fillRect(cx - 1, cy - 6, 3, 13, TFT_LIGHTGREY);
  };
  drawVolBtn(g.volMinusX, false);
  drawVolBtn(g.volPlusX, true);

  // 全幅 = 0〜MAX_VOLUME のスケール (右端が上限)
  d.drawRoundRect(g.volX, g.volY, g.volW, kVolBarH, 6, TFT_DARKGREY);
  int fillW = (g.volW - 4) * constrain(volumeLevel, 0, MAX_VOLUME) / MAX_VOLUME;
  if (fillW > 0) {
    d.fillRoundRect(g.volX + 2, g.volY + 2, fillW, kVolBarH - 4, 5,
                    d.color565(60, 130, 220));
  }
  d.fillCircle(g.volX + 2 + fillW, g.volY + kVolBarH / 2, 8, TFT_WHITE);
}

// 操作アイコン (🔀 / ◀◀ / ▶⏸ / ▶▶)。向きによらず常に最下段に横並びで表示する
// ボタン i (0=シャッフル, 1=前, 2=再生/一時停止, 3=次) の中心 X は cx + (i-1.5)*g.ctrlSpacing
void drawHint() {
  auto& d = M5.Display;
  Geo g = geo();
  uint16_t c = TFT_LIGHTGREY;

  d.fillRect(0, g.ctrlCy - kHintSize, g.W, kHintSize * 2, TFT_BLACK);
  int cy = g.ctrlCy;
  int cx = g.W / 2;
  auto btnCx = [&](int i) {
    return cx + static_cast<int>((i - 1.5f) * g.ctrlSpacing);
  };

  // 🔀 シャッフル (状態はアイコン色で表現: ON=シアン / OFF=ダークグレー)
  int sx = btnCx(0);
  uint16_t sc = shuffleEnabled ? TFT_CYAN : TFT_DARKGREY;
  // 交差する2本の線 (2px幅に見えるよう1px分ずらして重ね描き)
  d.drawLine(sx - 11, cy - 6, sx + 5, cy + 6, sc);
  d.drawLine(sx - 11, cy - 5, sx + 5, cy + 7, sc);
  d.drawLine(sx - 11, cy + 6, sx + 5, cy - 6, sc);
  d.drawLine(sx - 11, cy + 7, sx + 5, cy - 5, sc);
  // 右端の矢頭 (上下2個)
  d.fillTriangle(sx + 5, cy - 9, sx + 5, cy - 3, sx + 11, cy - 6, sc);
  d.fillTriangle(sx + 5, cy + 3, sx + 5, cy + 9, sx + 11, cy + 6, sc);

  // ◀◀ (前の曲)
  int px = btnCx(1);
  d.fillTriangle(px - 11, cy, px, cy - 8, px, cy + 8, c);
  d.fillTriangle(px, cy, px + 11, cy - 8, px + 11, cy + 8, c);

  // ▶ / ⏸ (中央)
  int mx = btnCx(2);
  if (playbackState == "playing") {
    d.fillRect(mx - 8, cy - 8, 6, 16, c);
    d.fillRect(mx + 2, cy - 8, 6, 16, c);
  } else {
    d.fillTriangle(mx - 7, cy - 9, mx - 7, cy + 9, mx + 9, cy, c);
  }

  // ▶▶ (次の曲)
  int nx = btnCx(3);
  d.fillTriangle(nx - 11, cy - 8, nx - 11, cy + 8, nx, cy, c);
  d.fillTriangle(nx, cy - 8, nx, cy + 8, nx + 11, cy, c);
}

void redrawAll() {
  StateLock lock;
  M5.Display.fillScreen(TFT_BLACK);
  if (artZoom) {
    drawArtZoom();
  } else {
    drawContent();
    drawVolume();
    drawHint();
  }
  drawBattery();
  dirtyContent = dirtyVolume = dirtyHint = dirtyBattery = false;
}

void redraw() {
  StateLock lock;
  if (artZoom) {
    // 拡大中は曲名・スライダー・操作ボタンを隠しているので、それらの dirty は
    // フラグだけ落とす (解除時に redrawAll() で描き直す)
    if (dirtyContent) { drawArtZoom(); dirtyContent = false; }
    dirtyVolume = false;
    dirtyHint = false;
    if (dirtyBattery) { drawBattery(); dirtyBattery = false; }
  } else {
    if (dirtyContent) { drawContent(); dirtyContent = false; }
    if (dirtyVolume) { drawVolume(); dirtyVolume = false; }
    if (dirtyHint) { drawHint(); dirtyHint = false; }
    if (dirtyBattery) { drawBattery(); dirtyBattery = false; }
  }
}

// ---------- 選曲ブラウザ ----------
// コンテンツ領域の長押しで開く 2 階層ブラウザ。
// レベル0: 再生キュー / お気に入り曲 / プレイリスト一覧, レベル1: 曲一覧

enum RowKind : uint8_t {
  kSrcQueue,        // 再生キュー (ソース)
  kSrcFavorites,    // お気に入り曲 (ソース)
  kSrcPlaylist,     // プレイリスト (ソース)
  kTrackQueue,      // キュー内の曲
  kTrackPlaylist,   // プレイリスト内の曲
  kTrackFavorite,   // お気に入りの曲
};

struct BrowseRow {
  String label;
  String uri;
  String id;        // queue_item_id または item_id
  String provider;
  uint8_t kind;
  String artist;     // 歌手順ソート・頭文字ジャンプ用 (曲行のみ。無い行は空)
  uint16_t origIdx;  // ロード時の並び (元の順への復帰に使う。ロード後に採番)
};

constexpr int kHeaderH = 36;
constexpr int kRowH = 40;
constexpr int kBrowseMaxRows = 200;
constexpr int kFetchLimit = 50;

const char* const kExcludedPlaylistNames[] = BROWSE_EXCLUDE_PLAYLISTS;

bool isExcludedPlaylist(const String& name) {
  for (const char* excluded : kExcludedPlaylistNames) {
    if (name == excluded) return true;
  }
  return false;
}

bool browsing = false;
int browseLevel = 0;
std::vector<BrowseRow> browseRows;
int scrollY = 0;
BrowseRow currentSource;
bool moreAvailable = false;  // offset で追加ロード可能か (キュー/お気に入りのみ)
int fetchedCount = 0;

// 表示順 (ロードごとにデフォルトへリセット。デフォルトは曲名順、再生キューのみ追加順)
enum SortMode : uint8_t { kSortOrig, kSortTitle, kSortArtist };
SortMode browseSortMode = kSortTitle;
bool sortPanelOpen = false;  // ソート/頭文字ジャンプのパネル表示中か
const char* kSortModeNames[3] = {"追加順", "曲名順", "歌手順"};
// 頭文字ジャンプの文字: group 0=#(数字/記号) 1..26=A..Z 27..36=あ行..わ行
const char* kIndexChars[37] = {
    "#", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
    "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
    "あ", "か", "さ", "た", "な", "は", "ま", "や", "ら", "わ"};
// パネルの行割り: {開始 index, 文字数} × 4行
constexpr int kIndexRowSpec[4][2] = {{0, 9}, {9, 9}, {18, 9}, {27, 10}};
constexpr int kSortRowH = 44;  // ソート方式行の高さ
constexpr int kIdxRowH = 38;   // 頭文字行の高さ (4行 + 方式行がヘッダー下に収まる高さ)

// リストのタッチ状態
bool listTouching = false;
bool listMoved = false;
int listTouchStartY = 0, listLastY = 0;

// 大きな JSON (プレイリスト全曲など) は PSRAM に置く
struct PsAllocator : ArduinoJson::Allocator {
  void* allocate(size_t size) override { return ps_malloc(size); }
  void deallocate(void* ptr) override { free(ptr); }
  void* reallocate(void* ptr, size_t size) override { return ps_realloc(ptr, size); }
};
PsAllocator psAllocator;

// 幅に収まるように末尾を「…」で切り詰める (リスト行用)
String truncateToWidth(String s, int maxW) {
  auto& d = M5.Display;
  if (d.textWidth(s) <= maxW) return s;
  while (s.length() > 3 && d.textWidth(s + "…") > maxW) {
    s.remove(s.length() - 3);  // UTF-8 途中切りを避けて多めに削る
  }
  return s + "…";
}

// API を叩いて配列レスポンスを filter 付きで取得する。
// 一時的な通信失敗で空リスト表示にならないよう最大3回リトライする
bool apiGetArray(const char* command, const String& argsJson, JsonDocument& filter,
                 JsonDocument& doc) {
  HttpLock httpLock;
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(200));
    HTTPClient http;
    // Apple Music のプレイリスト一覧は MA 側で生成に時間がかかる。
    // 15 秒では For work があるのに一覧全体を取得失敗扱いになる。
    int status = postApi(http, command, argsJson, 60000);
    if (status < 200 || status >= 300) {
      Serial.printf("browse http error: %d (attempt %d)\n", status, attempt + 1);
      http.end();
      continue;
    }
    BodyBuf body;
    bool ok = readBody(http, body, 15000);  // プレイリスト全曲は大きいので長めに待つ
    http.end();
    DeserializationError err = DeserializationError::EmptyInput;
    if (ok) {
      err = deserializeJson(doc, static_cast<const char*>(body.data), body.len,
                            DeserializationOption::Filter(filter));
    }
    if (!err) return true;
    Serial.printf("browse json error: %s (attempt %d)\n", err.c_str(), attempt + 1);
  }
  return false;
}

// ---------- ブラウズ結果のキャッシュ ----------
// リスト取得は MA サーバ側の生成が遅い (大きいプレイリストで数秒) ので、
// 取得結果をソースごとに PSRAM へキャッシュして 2 回目以降は即表示する。
// 行データは String のまま持つと内部 RAM を食い潰すため、取得順に
// パックしたブロブ (kind + label\0 uri\0 id\0 provider\0 artist\0 ...) で保持する。
// エントリの検索/差し替え/読み出しは stateMutex で保護 (core0 の先読みと共用)
constexpr uint32_t kBrowseCacheTtlMs = 10UL * 60 * 1000;

struct CachedList {
  String key;        // "sources" / "favorites" / プレイリスト uri
  char* blob = nullptr;
  int rowCount = 0;
  int fetchedCount = 0;
  bool moreAvailable = false;
  uint32_t fetchedAt = 0;
};
std::vector<CachedList> browseCache;

// rows を取得順 (origIdx 順) でキャッシュに保存する
void cacheStore(const String& key, const std::vector<BrowseRow>& rows,
                int fetchedCnt, bool moreAvail) {
  std::vector<const BrowseRow*> order;
  order.reserve(rows.size());
  for (const auto& r : rows) order.push_back(&r);
  std::sort(order.begin(), order.end(),
            [](const BrowseRow* a, const BrowseRow* b) { return a->origIdx < b->origIdx; });
  size_t need = 0;
  for (const BrowseRow* r : order) {
    need += 1 + r->label.length() + 1 + r->uri.length() + 1 + r->id.length() + 1 +
            r->provider.length() + 1 + r->artist.length() + 1;
  }
  char* blob = static_cast<char*>(ps_malloc(need ? need : 1));
  if (!blob) return;
  char* p = blob;
  for (const BrowseRow* r : order) {
    *p++ = static_cast<char>(r->kind);
    for (const String* s : {&r->label, &r->uri, &r->id, &r->provider, &r->artist}) {
      memcpy(p, s->c_str(), s->length() + 1);
      p += s->length() + 1;
    }
  }
  StateLock lock;
  CachedList* e = nullptr;
  for (auto& c : browseCache) {
    if (c.key == key) { e = &c; break; }
  }
  if (!e) {
    browseCache.emplace_back();
    e = &browseCache.back();
    e->key = key;
  }
  if (e->blob) free(e->blob);
  e->blob = blob;
  e->rowCount = order.size();
  e->fetchedCount = fetchedCnt;
  e->moreAvailable = moreAvail;
  e->fetchedAt = millis();
}

// キャッシュがあれば browseRows へ展開する (core1 専用)。TTL 切れは外れ扱い
bool cacheLoad(const String& key) {
  StateLock lock;
  for (const auto& c : browseCache) {
    if (c.key != key) continue;
    if (millis() - c.fetchedAt > kBrowseCacheTtlMs) return false;
    browseRows.clear();
    const char* p = c.blob;
    for (int i = 0; i < c.rowCount; i++) {
      BrowseRow r;
      r.kind = static_cast<uint8_t>(*p++);
      for (String* s : {&r.label, &r.uri, &r.id, &r.provider, &r.artist}) {
        *s = p;
        p += strlen(p) + 1;
      }
      r.origIdx = i;
      browseRows.push_back(r);
    }
    fetchedCount = c.fetchedCount;
    moreAvailable = c.moreAvailable;
    return true;
  }
  return false;
}

// 有効なキャッシュが存在するか (先読みのスキップ判定用)
bool cacheHasFresh(const String& key) {
  StateLock lock;
  for (const auto& c : browseCache) {
    if (c.key == key) return millis() - c.fetchedAt <= kBrowseCacheTtlMs;
  }
  return false;
}

// ソート用にカタカナ→ひらがな・英小文字→大文字へ正規化する
// (バイト順比較でも「あ」と「ア」、"a" と "A" が同じ位置に並ぶように)
String kanaFold(const String& s) {
  String out;
  out.reserve(s.length());
  const char* p = s.c_str();
  while (*p) {
    uint8_t c = static_cast<uint8_t>(*p);
    int len = c < 0x80 ? 1 : (c & 0xE0) == 0xC0 ? 2 : (c & 0xF0) == 0xE0 ? 3 : 4;
    int avail = 1;
    while (avail < len && p[avail]) avail++;
    if (len == 3 && avail == 3) {
      uint32_t cp = (static_cast<uint32_t>(c & 0x0F) << 12) |
                    ((static_cast<uint8_t>(p[1]) & 0x3F) << 6) |
                    (static_cast<uint8_t>(p[2]) & 0x3F);
      if (cp >= 0x30A1 && cp <= 0x30F6) cp -= 0x60;  // カタカナ→ひらがな
      out += static_cast<char>(0xE0 | (cp >> 12));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (len == 1) {
      out += static_cast<char>((c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c);
    } else {
      for (int i = 0; i < avail; i++) out += p[i];
    }
    p += avail;
  }
  return out;
}

// 頭文字のグループ番号: 0=#(数字/記号) 1..26=A..Z 27..36=あ行..わ行。
// 該当なし(漢字等)=-1
int initialGroupOf(const String& s) {
  if (!s.length()) return -1;
  uint8_t c = static_cast<uint8_t>(s[0]);
  if (c < 0x80) {
    if (c >= 'A' && c <= 'Z') return 1 + (c - 'A');
    if (c >= 'a' && c <= 'z') return 1 + (c - 'a');
    return 0;  // ASCII の数字・記号
  }
  if ((c & 0xF0) != 0xE0 || s.length() < 3) return -1;
  uint32_t cp = (static_cast<uint32_t>(c & 0x0F) << 12) |
                ((static_cast<uint8_t>(s[1]) & 0x3F) << 6) |
                (static_cast<uint8_t>(s[2]) & 0x3F);
  if (cp >= 0x30A1 && cp <= 0x30F6) cp -= 0x60;  // カタカナ→ひらがな
  if (cp < 0x3041 || cp > 0x3096) return -1;
  if (cp <= 0x304A) return 27;  // ぁ..お
  if (cp <= 0x3054) return 28;  // か..ご
  if (cp <= 0x305E) return 29;  // さ..ぞ
  if (cp <= 0x3069) return 30;  // た..ど
  if (cp <= 0x306E) return 31;  // な..の
  if (cp <= 0x307D) return 32;  // は..ぽ
  if (cp <= 0x3082) return 33;  // ま..も
  if (cp <= 0x3088) return 34;  // ゃ..よ
  if (cp <= 0x308D) return 35;  // ら..ろ
  return 36;                    // ゎ..ん
}

// browseSortMode に従って表示順を並べ替える。
// レベル0 は先頭1行 (再生キュー) を固定したままプレイリストのみ対象
void applyBrowseSort() {
  size_t fixed = (browseLevel == 0) ? 1 : 0;
  if (browseRows.size() <= fixed) return;
  auto begin = browseRows.begin() + fixed;
  switch (browseSortMode) {
    case kSortOrig:
      std::stable_sort(begin, browseRows.end(),
                       [](const BrowseRow& a, const BrowseRow& b) { return a.origIdx < b.origIdx; });
      break;
    case kSortTitle:
      std::stable_sort(begin, browseRows.end(), [](const BrowseRow& a, const BrowseRow& b) {
        return kanaFold(a.label) < kanaFold(b.label);
      });
      break;
    case kSortArtist:
      std::stable_sort(begin, browseRows.end(), [](const BrowseRow& a, const BrowseRow& b) {
        bool ae = a.artist.length() == 0, be = b.artist.length() == 0;
        if (ae != be) return be;  // 歌手情報がある行を前へ
        if (!ae) {
          String ka = kanaFold(a.artist), kb = kanaFold(b.artist);
          if (ka != kb) return ka < kb;
        }
        return kanaFold(a.label) < kanaFold(b.label);
      });
      break;
  }
}

// 頭文字ジャンプの判定対象文字列 (歌手順表示中は歌手名で判定)
const String& jumpKeyOf(const BrowseRow& row) {
  if (browseSortMode == kSortArtist && row.artist.length()) return row.artist;
  return row.label;
}

// 現在のソートのまま、指定グループで始まる最初の行へスクロールする。
// 該当がなければ後続のグループ (G→H→…→わ) を順に探して最初の行へ
void jumpToInitial(int group) {
  size_t fixed = (browseLevel == 0) ? 1 : 0;
  for (int target = group; target <= 36; target++) {
    for (size_t i = fixed; i < browseRows.size(); i++) {
      if (initialGroupOf(jumpKeyOf(browseRows[i])) != target) continue;
      Geo g = geo();
      int listH = g.H - kHeaderH;
      int maxScroll = max(0, static_cast<int>(browseRows.size()) * kRowH - listH);
      scrollY = constrain(static_cast<int>(i) * kRowH, 0, maxScroll);
      return;
    }
  }
}

void drawBrowseLoading() {
  StateLock lock;
  auto& d = M5.Display;
  Geo g = geo();
  d.fillScreen(TFT_BLACK);
  d.setFont(bodyFont());
  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.setTextDatum(middle_center);
  d.drawString("読み込み中...", g.W / 2, g.H / 2 - 40);
  spinnerBegin();
}

void drawBrowse() {
  spinnerEnd(false);  // 全画面を描き直すので跡消しは不要
  StateLock lock;
  auto& d = M5.Display;
  Geo g = geo();
  d.fillScreen(TFT_BLACK);
  uint16_t sep = d.color565(60, 60, 60);

  // ヘッダー: 左=戻る, 中央=タイトル, 右=ソートパネル呼び出し (現在の並びを表示)
  d.setFont(headerFont());
  d.setTextSize(1);
  d.setTextColor(TFT_CYAN, TFT_BLACK);
  d.setTextDatum(middle_left);
  d.drawString("< 戻る", 8, kHeaderH / 2);
  d.setTextDatum(middle_right);
  d.drawString(kSortModeNames[browseSortMode], g.W - 8, kHeaderH / 2);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextDatum(middle_center);
  String title = (browseLevel == 0) ? String("選曲") : currentSource.label;
  d.drawString(truncateToWidth(title, g.W - 180), g.W / 2, kHeaderH / 2);
  d.drawFastHLine(0, kHeaderH - 1, g.W, TFT_LIGHTGREY);

  // 行 (スクロール中の欠け行がヘッダーに食い込まないようクリップする)
  int listH = g.H - kHeaderH;
  int maxScroll = max(0, static_cast<int>(browseRows.size()) * kRowH - listH);
  scrollY = constrain(scrollY, 0, maxScroll);
  d.setFont(bodyFont());
  d.setTextDatum(middle_left);
  d.setClipRect(0, kHeaderH, g.W, listH);
  int i = scrollY / kRowH;
  int y = kHeaderH - (scrollY % kRowH);
  for (; i < static_cast<int>(browseRows.size()) && y < g.H; i++, y += kRowH) {
    auto& row = browseRows[i];
    bool isSource = row.kind <= kSrcPlaylist;
    d.setTextColor(isSource && browseLevel == 0 ? TFT_WHITE : TFT_SILVER, TFT_BLACK);
    d.drawString(truncateToWidth(row.label, g.W - 24), 10, y + kRowH / 2);
    d.drawFastHLine(0, y + kRowH - 1, g.W, sep);
  }
  d.clearClipRect();
  if (browseRows.empty()) {
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setTextDatum(middle_center);
    d.drawString("(空です)", g.W / 2, g.H / 2);
  }

  // 簡易スクロールバー
  if (maxScroll > 0) {
    int barH = max(20, listH * listH / (static_cast<int>(browseRows.size()) * kRowH));
    int barY = kHeaderH + (listH - barH) * scrollY / maxScroll;
    d.fillRoundRect(g.W - 5, barY, 4, barH, 2, TFT_DARKGREY);
  }
}

// ソート方式と頭文字ジャンプのパネル (ヘッダー直下に重ね描き)
// 行1: 追加順/曲名順/歌手順、行2-5: # A..Z / あ..わ の頭文字グリッド
void drawSortPanel() {
  StateLock lock;
  auto& d = M5.Display;
  Geo g = geo();
  int y0 = kHeaderH;
  int panelH = kSortRowH + kIdxRowH * 4;
  uint16_t bg = d.color565(20, 30, 45);
  uint16_t sel = d.color565(30, 90, 160);
  d.fillRect(0, y0, g.W, panelH, bg);
  d.drawRect(0, y0, g.W, panelH, TFT_LIGHTGREY);
  d.setFont(bodyFont());
  d.setTextSize(1);
  d.setTextDatum(middle_center);
  for (int m = 0; m < 3; m++) {
    int x = g.W * m / 3, w = g.W / 3;
    if (m == browseSortMode) d.fillRoundRect(x + 4, y0 + 4, w - 8, kSortRowH - 8, 6, sel);
    d.setTextColor(m == browseSortMode ? TFT_WHITE : TFT_SILVER);
    d.drawString(kSortModeNames[m], x + w / 2, y0 + kSortRowH / 2);
  }
  d.drawFastHLine(0, y0 + kSortRowH, g.W, TFT_DARKGREY);
  d.setTextColor(TFT_CYAN);
  for (int row = 0; row < 4; row++) {
    int start = kIndexRowSpec[row][0], cols = kIndexRowSpec[row][1];
    int y = y0 + kSortRowH + kIdxRowH * row;
    for (int col = 0; col < cols; col++) {
      d.drawString(kIndexChars[start + col], g.W * col / cols + g.W / cols / 2,
                   y + kIdxRowH / 2);
    }
  }
}

// パネル表示中のタップ。パネル外タップは何もせず閉じる
void handleSortPanelTap(int x, int y) {
  sortPanelOpen = false;
  Geo g = geo();
  int rel = y - kHeaderH;
  if (rel >= 0 && rel < kSortRowH) {
    browseSortMode = static_cast<SortMode>(constrain(x * 3 / g.W, 0, 2));
    scrollY = 0;
    applyBrowseSort();
  } else if (rel >= kSortRowH && rel < kSortRowH + kIdxRowH * 4) {
    int row = (rel - kSortRowH) / kIdxRowH;
    int start = kIndexRowSpec[row][0], cols = kIndexRowSpec[row][1];
    jumpToInitial(start + constrain(x * cols / g.W, 0, cols - 1));
  }
  drawBrowse();  // パネル消去も兼ねて全再描画
}

// タップされた行の背景をハイライトする (通信開始前にすぐ呼び、選択結果を視覚化する)
void highlightRow(int idx) {
  Geo g = geo();
  int y = kHeaderH + idx * kRowH - scrollY;
  if (y + kRowH <= kHeaderH || y >= g.H) return;  // 画面外ならスキップ
  StateLock lock;
  auto& d = M5.Display;
  uint16_t bg = d.color565(30, 90, 160);
  d.setClipRect(0, kHeaderH, g.W, g.H - kHeaderH);  // ヘッダーへの食い込み防止
  d.fillRect(0, y, g.W, kRowH, bg);
  d.setFont(bodyFont());
  d.setTextSize(1);
  d.setTextColor(TFT_WHITE, bg);
  d.setTextDatum(middle_left);
  d.drawString(truncateToWidth(browseRows[idx].label, g.W - 24), 10, y + kRowH / 2);
  d.clearClipRect();
}

// ソース一覧を rows へ取得する (UI 非依存。core0 の先読みからも呼ぶ)。
// 戻り値: プレイリスト一覧の取得に成功したか (失敗でも固定1行は返す)
bool fetchSources(std::vector<BrowseRow>& rows) {
  rows.clear();
  rows.push_back({"♪ 再生キュー", "", "", "", kSrcQueue});

  // プレイリストは MA ライブラリ経由 (music/playlists/library_items) だと
  // 「明示的にライブラリへ追加した分」しか出ない。YT Music アプリ側で
  // 作っただけのプレイリストも拾うため、music/browse でプロバイダーを
  // 直接覗く (root -> 各音楽プロバイダー -> playlists フォルダ)
  JsonDocument rootFilter;
  rootFilter[0]["provider"] = true;
  rootFilter[0]["path"] = true;
  JsonDocument rootDoc(&psAllocator);
  bool ok = apiGetArray("music/browse", "{\"path\":\"root\"}", rootFilter, rootDoc);
  if (ok) {
    for (JsonObject prov : rootDoc.as<JsonArray>()) {
      String path = String(prov["path"] | "") + "playlists";

      JsonDocument plFilter;
      plFilter[0]["name"] = true;
      plFilter[0]["uri"] = true;
      plFilter[0]["item_id"] = true;
      plFilter[0]["provider"] = true;
      plFilter[0]["media_type"] = true;
      JsonDocument plDoc(&psAllocator);
      String args = String("{\"path\":\"") + path + "\"}";
      if (!apiGetArray("music/browse", args, plFilter, plDoc)) {
        continue;
      }
      for (JsonObject o : plDoc.as<JsonArray>()) {
        String nm = String(o["name"] | "");
        String mt = String(o["media_type"] | "");
        bool excluded = isExcludedPlaylist(nm);
        if (mt != "playlist") continue;  // ".." 等を除外
        if (excluded) continue;
        rows.push_back({nm, String(o["uri"] | ""),
                        String(o["item_id"] | ""), String(o["provider"] | ""),
                        kSrcPlaylist});
        if (static_cast<int>(rows.size()) >= kBrowseMaxRows) break;
      }
    }
  }
  for (size_t i = 0; i < rows.size(); i++) rows[i].origIdx = i;
  return ok;
}

// 曲一覧 1 ページを out へ追記取得する (UI 非依存。core0 の先読みからも呼ぶ)。
// offset はキュー/お気に入りのみ有効。gotN: 今回取得した行数
bool fetchTracksPage(const BrowseRow& source, int offset, std::vector<BrowseRow>& out,
                     int& gotN) {
  gotN = 0;
  JsonDocument doc(&psAllocator);
  if (source.kind == kSrcQueue) {
    JsonDocument filter;
    filter[0]["name"] = true;
    filter[0]["queue_item_id"] = true;
    String args = String("{\"queue_id\":\"") + MA_PLAYER_ID +
                  "\",\"limit\":" + kFetchLimit + ",\"offset\":" + offset + "}";
    if (!apiGetArray("player_queues/items", args, filter, doc)) return false;
    for (JsonObject o : doc.as<JsonArray>()) {
      out.push_back({String(o["name"] | ""), "",
                     String(o["queue_item_id"] | ""), "", kTrackQueue});
      gotN++;
      if (static_cast<int>(out.size()) >= kBrowseMaxRows) break;
    }
    return true;
  }
  if (source.kind == kSrcFavorites) {
    JsonDocument filter;
    filter[0]["name"] = true;
    filter[0]["uri"] = true;
    filter[0]["artists"][0]["name"] = true;
    String args = String("{\"favorite\":true,\"limit\":") + kFetchLimit +
                  ",\"offset\":" + offset + ",\"order_by\":\"sort_name\"}";
    if (!apiGetArray("music/tracks/library_items", args, filter, doc)) return false;
    for (JsonObject o : doc.as<JsonArray>()) {
      String artist = o["artists"][0]["name"] | "";
      String label = String(o["name"] | "");
      if (artist.length()) label += " / " + artist;
      out.push_back({label, String(o["uri"] | ""), "", "", kTrackFavorite, artist});
      gotN++;
      if (static_cast<int>(out.size()) >= kBrowseMaxRows) break;
    }
    return true;
  }
  // プレイリスト (offset 非対応、一括で返るので上限まで取り込む)
  JsonDocument filter;
  filter[0]["name"] = true;
  filter[0]["uri"] = true;
  filter[0]["artists"][0]["name"] = true;
  String args = String("{\"item_id\":\"") + source.id +
                "\",\"provider_instance_id_or_domain\":\"" + source.provider +
                "\",\"allow_dynamic_tracks\":true}";
  if (!apiGetArray("music/playlists/playlist_tracks", args, filter, doc)) return false;
  for (JsonObject o : doc.as<JsonArray>()) {
    String artist = o["artists"][0]["name"] | "";
    String label = String(o["name"] | "");
    if (artist.length()) label += " / " + artist;
    out.push_back({label, String(o["uri"] | ""), "", "", kTrackPlaylist, artist});
    gotN++;
    if (static_cast<int>(out.size()) >= kBrowseMaxRows) break;
  }
  return true;
}

// キャッシュのキー。キューは再生で頻繁に変わるのでキャッシュ対象外 (空文字)
String cacheKeyOf(const BrowseRow& source) {
  if (source.kind == kSrcFavorites) return "favorites";
  if (source.kind == kSrcPlaylist) return source.uri;
  return "";
}

void loadSources() {
  scrollY = 0;
  moreAvailable = false;
  browseSortMode = kSortTitle;  // デフォルトは名前順
  sortPanelOpen = false;
  browseLevel = 0;
  if (cacheLoad("sources")) {  // キャッシュ命中なら通信なしで即表示
    applyBrowseSort();
    drawBrowse();
    return;
  }
  drawBrowseLoading();
  bool ok = fetchSources(browseRows);
  if (ok) cacheStore("sources", browseRows, 0, false);
  applyBrowseSort();
  drawBrowse();
}

// レベル1 の曲一覧をロードする。append=true なら offset 続きから追加
void loadTracks(bool append) {
  String key = cacheKeyOf(currentSource);
  if (!append) {
    scrollY = 0;
    fetchedCount = 0;
    moreAvailable = false;
    // デフォルトは曲名順。再生キューだけは再生順に意味があるので追加順
    browseSortMode = (currentSource.kind == kSrcQueue) ? kSortOrig : kSortTitle;
    sortPanelOpen = false;
    browseLevel = 1;
    if (key.length() && cacheLoad(key)) {  // キャッシュ命中なら通信なしで即表示
      applyBrowseSort();
      drawBrowse();
      return;
    }
    drawBrowseLoading();
    browseRows.clear();
  }
  size_t firstNew = browseRows.size();
  int gotN = 0;
  bool ok = fetchTracksPage(currentSource, fetchedCount, browseRows, gotN);
  // 追加分にロード順を記録し、ソート表示中なら並びへ組み込む
  for (size_t i = firstNew; i < browseRows.size(); i++) browseRows[i].origIdx = i;
  if (ok) {
    fetchedCount += gotN;
    moreAvailable = (currentSource.kind != kSrcPlaylist) && gotN == kFetchLimit &&
                    static_cast<int>(browseRows.size()) < kBrowseMaxRows;
    if (key.length()) cacheStore(key, browseRows, fetchedCount, moreAvailable);
  }
  if (browseSortMode != kSortOrig) applyBrowseSort();
  drawBrowse();
}

// ---------- ブラウズ先読み (networkTask 上で実行) ----------
// 起動後にソース一覧→お気に入り→各プレイリストを 1 ステップずつ取得して
// キャッシュを温める。ブラウズ中とコマンド処理待ちの間は停止する
int prefetchStage = 0;  // 0=ソース一覧 1..=prefetchList の順次取得 -1=完了
std::vector<BrowseRow> prefetchTargets;

// 1 ステップ進める。false = 取得失敗 (呼び出し側で間隔を空けて再試行)
bool prefetchStep() {
  if (prefetchStage < 0) return true;
  if (prefetchStage == 0) {
    std::vector<BrowseRow> rows;
    if (!fetchSources(rows)) return false;
    cacheStore("sources", rows, 0, false);
    prefetchTargets.clear();
    for (const auto& r : rows) {
      if (r.kind == kSrcFavorites || r.kind == kSrcPlaylist) prefetchTargets.push_back(r);
    }
    prefetchStage = 1;
    Serial.printf("prefetch: sources ok, %d lists\n", prefetchTargets.size());
    return true;
  }
  int idx = prefetchStage - 1;
  if (idx >= static_cast<int>(prefetchTargets.size())) {
    prefetchTargets.clear();
    prefetchTargets.shrink_to_fit();
    prefetchStage = -1;
    Serial.println("prefetch: done");
    return true;
  }
  const BrowseRow& src = prefetchTargets[idx];
  String key = cacheKeyOf(src);
  bool ok = true;
  if (!cacheHasFresh(key)) {  // ユーザー操作で取得済みならスキップ
    std::vector<BrowseRow> rows;
    int gotN = 0;
    ok = fetchTracksPage(src, 0, rows, gotN);
    if (ok) {
      for (size_t i = 0; i < rows.size(); i++) rows[i].origIdx = i;
      bool more = (src.kind == kSrcFavorites) && gotN == kFetchLimit &&
                  static_cast<int>(rows.size()) < kBrowseMaxRows;
      cacheStore(key, rows, gotN, more);
    }
  }
  prefetchStage++;  // 失敗してもスキップして先へ (TTL 内の再訪 or ユーザー操作で埋まる)
  return ok;
}

void exitBrowse() {
  { StateLock lock; browsing = false; }
  sortPanelOpen = false;
  browseRows.clear();
  browseRows.shrink_to_fit();
  redrawAll();
}

void browseBack() {
  if (browseLevel == 1) {
    loadSources();
  } else {
    exitBrowse();
  }
}

void selectBrowseRow(int idx) {
  if (idx < 0 || idx >= static_cast<int>(browseRows.size())) return;
  highlightRow(idx);  // 通信 (スピナー表示) の前に選択行を視覚化する
  BrowseRow row = browseRows[idx];
  switch (row.kind) {
    case kSrcQueue:
    case kSrcFavorites:
    case kSrcPlaylist:
      currentSource = row;
      loadTracks(false);
      return;
    case kTrackQueue:
      sendCommand("player_queues/play_index",
                  String("{\"queue_id\":\"") + MA_PLAYER_ID + "\",\"index\":\"" +
                      row.id + "\"}");
      break;
    case kTrackPlaylist: {
      // プレイリスト全体をキューに入れ、選んだ曲から再生。
      // MA はシャッフル ON のままキューをロードすると start_item ごと並べ替えて
      // しまい選んだ曲から始まらないため、一時的に OFF にして投入後に戻す
      // (cmdQueue は networkTask が FIFO で連続処理するので順序保証あり)
      bool shuffle;
      { StateLock lock; shuffle = shuffleEnabled; }
      if (shuffle) {
        sendCommand("player_queues/shuffle",
                    String("{\"queue_id\":\"") + MA_PLAYER_ID +
                        "\",\"shuffle_enabled\":false}");
      }
      sendCommand("player_queues/play_media",
                  String("{\"queue_id\":\"") + MA_PLAYER_ID + "\",\"media\":\"" +
                      currentSource.uri + "\",\"option\":\"replace\",\"start_item\":\"" +
                      row.uri + "\"}");
      if (shuffle) {
        sendCommand("player_queues/shuffle",
                    String("{\"queue_id\":\"") + MA_PLAYER_ID +
                        "\",\"shuffle_enabled\":true}");
      }
      break;
    }
    case kTrackFavorite:
      sendCommand("player_queues/play_media",
                  String("{\"queue_id\":\"") + MA_PLAYER_ID + "\",\"media\":\"" +
                      row.uri + "\",\"option\":\"replace\"}");
      break;
  }
  { StateLock lock; repeatFixSent = false; }  // 新しいキュー内容になるので矯正をやり直す
  exitBrowse();
}

void browseScrollBy(int dy) {
  Geo g = geo();
  int listH = g.H - kHeaderH;
  int maxScroll = max(0, static_cast<int>(browseRows.size()) * kRowH - listH);
  int newY = constrain(scrollY + dy, 0, maxScroll);
  if (newY != scrollY) {
    scrollY = newY;
    drawBrowse();
  }
  // 末尾付近まで来たら追加ロード
  if (moreAvailable && scrollY >= maxScroll - kRowH) {
    loadTracks(true);
  }
}

void handleBrowseTouch() {
  auto t = M5.Touch.getDetail();
  if (sortPanelOpen) {
    // パネル表示中はリスト操作を止め、リリースだけ拾う
    if (t.wasReleased()) handleSortPanelTap(t.x, t.y);
    return;
  }
  if (t.wasPressed()) {
    listTouching = true;
    listMoved = false;
    listTouchStartY = t.y;
    listLastY = t.y;
    return;
  }
  if (!listTouching) return;

  if (t.isPressed()) {
    if (abs(t.y - listTouchStartY) > 10) listMoved = true;
    if (listMoved && t.y != listLastY) {
      browseScrollBy(listLastY - t.y);
      listLastY = t.y;
    }
    return;
  }

  if (t.wasReleased()) {
    listTouching = false;
    if (listMoved) return;
    if (t.y < kHeaderH) {
      Geo g = geo();
      if (t.x < 110) {
        browseBack();
      } else if (t.x >= g.W - 90) {  // 右上: ソートパネルを開く
        sortPanelOpen = true;
        drawSortPanel();
      }
      return;
    }
    selectBrowseRow((scrollY + t.y - kHeaderH) / kRowH);
  }
}

// ブラウズ中の物理ボタン: 前=1ページ上 / 中央=戻る / 次=1ページ下
void handleBrowseButtons() {
  if (sortPanelOpen) return;  // パネル表示中はスクロールでパネルを壊さない
  bool prevIsA = (orient == 0 || orient == 3);
  auto& upBtn = prevIsA ? M5.BtnA : M5.BtnC;
  auto& downBtn = prevIsA ? M5.BtnC : M5.BtnA;
  Geo g = geo();
  int page = ((g.H - kHeaderH) / kRowH) * kRowH;

  if (upBtn.wasPressed()) browseScrollBy(-page);
  if (downBtn.wasPressed()) browseScrollBy(page);
  if (M5.BtnB.wasPressed()) browseBack();
}

// ---------- IMU による向き補正 ----------

// 実測 (MPU6886): 横・通常で立てると +Y が上、+X は右端方向。
// 重力が 0.5G 以上かかっている軸で 4 方向を判定し、kImuStableMs の安定時間で確定する。
void checkOrientation() {
  if (millis() < nextImuCheckAt) return;
  nextImuCheckAt = millis() + kImuCheckMs;

  float ax, ay, az;
  if (!M5.Imu.getAccel(&ax, &ay, &az)) return;

  int desired = -1;
  if (fabsf(ay) >= fabsf(ax)) {
    if (ay > 0.5f) desired = 0;        // 横・通常 (上端が上)
    else if (ay < -0.5f) desired = 1;  // 横・逆さ
  } else {
    if (ax > 0.5f) desired = 2;        // 縦・右端が上 (ボタン列は右)
    else if (ax < -0.5f) desired = 3;  // 縦・左端が上 (ボタン列は左)
  }
  if (desired < 0 || desired == orient) {
    candidateOrient = -1;
    return;
  }
  if (desired != candidateOrient) {
    candidateOrient = desired;
    candidateSince = millis();
    return;
  }
  if (millis() - candidateSince < kImuStableMs) return;

  orient = desired;
  candidateOrient = -1;
  {
    StateLock lock;
    M5.Display.setRotation(kRotFor[orient]);
    Serial.printf("orientation: %d (rotation %d)\n", orient, kRotFor[orient]);
    if (browsing) {
      drawBrowse();
    } else {
      redrawAll();
    }
  }
}

// ---------- 液晶オートオフ ----------

// 停止状態が一定時間続いたら液晶をオフにする(通常ループの末尾から毎回呼ぶ)
void updateLcdSleep() {
  if (lcdSleeping) return;
  StateLock lock;
  if (playbackState == "playing") return;
  if (millis() < nextLcdSleepAt) return;
  M5.Display.sleep();
  lcdSleeping = true;
}

// タップで液晶を復帰させる
void wakeLcd() {
  StateLock lock;
  M5.Display.wakeup();
  lcdSleeping = false;
  nextLcdSleepAt = millis() + kLcdSleepTimeoutMs;  // ここからまた10分カウント
  redrawAll();
}

// ---------- 入力 ----------

// 操作アクション (物理ボタンと画面アイコンの両方から呼ぶ)。
// 再生位置が3秒を超えていたら曲の先頭に戻り、それ以外は前の曲へ移動する
void doPrev() {
  if (currentElapsedSeconds() > 3.0f) {
    sendSeek(0);
  } else {
    sendCommand("players/cmd/previous",
                String("{\"player_id\":\"") + MA_PLAYER_ID + "\"}");
  }
}

void doNext() {
  sendCommand("players/cmd/next",
              String("{\"player_id\":\"") + MA_PLAYER_ID + "\"}");
}

void doPlayPause() {
  // 楽観更新: 表示を即トグル
  setPlaybackState(playbackState == "playing" ? "paused" : "playing");
  sendCommand("players/cmd/play_pause",
              String("{\"player_id\":\"") + MA_PLAYER_ID + "\"}");
}

void doShuffleToggle() {
  // 楽観更新: 表示を即トグル。サーバー値は次回の pollQueueSettings() で上書き同期される
  bool next;
  { StateLock lock; shuffleEnabled = !shuffleEnabled; next = shuffleEnabled; dirtyHint = true; }
  sendCommand("player_queues/shuffle",
              String("{\"queue_id\":\"") + MA_PLAYER_ID +
                  "\",\"shuffle_enabled\":" + (next ? "true" : "false") + "}");
}

// 画面上の操作アイコンのタップ判定。ボタンは常にスライダー下に横並び
// (中心 g.W/2 から g.ctrlSpacing 間隔で 4 つ: 0=シャッフル/1=前/2=再生/3=次)
void handleHintTouch() {
  auto t = M5.Touch.getDetail();
  if (!t.wasPressed() || dragging) return;
  Geo g = geo();

  // 帯の厚み方向は指の太さぶん広めに取る (スライダーとは重ならない範囲)
  if (t.y < g.ctrlCy - kHintSize - 6 || t.y >= g.ctrlCy + kHintSize + 6) return;

  int cx = g.W / 2;
  int idx = lroundf((t.x - cx) / static_cast<float>(g.ctrlSpacing) + 1.5f);
  // 中心から ±ctrlSpacing/2 を超えていたら無視 (端の外側をはじく)
  if (idx < 0 || idx > 3) return;

  switch (idx) {
    case 0: doShuffleToggle(); break;
    case 1: doPrev(); break;
    case 2: doPlayPause(); break;
    case 3: doNext(); break;
  }
}

// コンテンツ領域 (操作ボタン・スライダー以外) の長押しで選曲ブラウザを開く。
// 加えて、アート矩形の短いタップでアート全画面表示 (artZoom) をトグルする。
// 拡大中は画面のどこをタップしても復帰できるようにする
void handleContentTouch() {
  static uint32_t pressStartMs = 0;
  static int pressX = 0, pressY = 0;
  static bool armed = false;
  static bool pressInArt = false;

  auto t = M5.Touch.getDetail();
  Geo g = geo();

  if (t.wasPressed()) {
    if (artZoom) {
      // 拡大中は除外領域なし、どこでもタップで復帰できるよう常に arm する
      armed = true;
      pressStartMs = millis();
      pressX = t.x;
      pressY = t.y;
      pressInArt = false;
      return;
    }
    // スライダー領域と操作ボタン行は除外
    bool inSlider = t.y >= g.volY - 12 && t.y < g.volY + kVolBarH + 12;
    bool inCtrl = t.y >= g.ctrlCy - kHintSize - 6 && t.y < g.ctrlCy + kHintSize + 6;
    if (!inSlider && !inCtrl) {
      armed = true;
      pressStartMs = millis();
      pressX = t.x;
      pressY = t.y;
      pressInArt = t.x >= g.artX && t.x < g.artX + g.artSize &&
                  t.y >= g.artY && t.y < g.artY + g.artSize;
    } else {
      armed = false;
    }
    return;
  }
  if (!armed) return;

  if (t.isPressed()) {
    if (abs(t.x - pressX) > 12 || abs(t.y - pressY) > 12) {
      armed = false;
      return;
    }
    if (!artZoom && millis() - pressStartMs >= 700) {
      armed = false;
      { StateLock lock; browsing = true; }
      loadSources();
    }
    return;
  }
  if (t.wasReleased()) {
    if (armed) {
      if (artZoom) {
        artZoom = false;
        redrawAll();
      } else if (pressInArt) {
        artZoom = true;
        redrawAll();
      }
    }
    armed = false;
  }
}

// 音量を delta ぶん増減して即送信 (−/+ ボタン用)
void stepVolume(int delta) {
  int vol;
  {
    StateLock lock;
    vol = constrain(volumeLevel + delta, 0, MAX_VOLUME);
    if (vol == volumeLevel) return;
    volumeLevel = vol;
    dirtyVolume = true;
  }
  sendVolume(vol);
}

// スライダーのドラッグ/タップ処理と −/+ ボタン
void handleSliderTouch() {
  auto t = M5.Touch.getDetail();
  Geo g = geo();

  if (!dragging) {
    // スライダー行 (指の太さぶん上下に余裕を持たせる) で押されたら判定
    if (t.wasPressed() && t.y >= g.volY - 12 && t.y < g.volY + kVolBarH + 12) {
      // 先に −/+ ボタンを判定 (バーとの隙間はボタン側に倒す)
      if (t.x >= g.volMinusX - 8 && t.x < g.volMinusX + g.volBtnW + 8) {
        stepVolume(-kVolStep);
        return;
      }
      if (t.x >= g.volPlusX - 8 && t.x < g.volPlusX + g.volBtnW + 8) {
        stepVolume(+kVolStep);
        return;
      }
      if (t.x >= g.volX && t.x < g.volX + g.volW) {
        dragging = true;
      } else {
        return;
      }
    } else {
      return;
    }
  }

  if (t.isPressed()) {
    int vol = constrain((t.x - g.volX) * MAX_VOLUME / g.volW, 0, MAX_VOLUME);
    bool changed;
    {
      StateLock lock;
      changed = (vol != volumeLevel);
      if (changed) {
        volumeLevel = vol;
        dirtyVolume = true;
      }
    }
    if (changed) {
      // ドラッグ中は送信を間引く
      if (millis() - lastDragSendAt >= kDragSendMs) {
        lastDragSendAt = millis();
        sendVolume(vol);
      }
    }
  }
  if (t.wasReleased()) {
    dragging = false;
    int finalVol;
    { StateLock lock; finalVol = volumeLevel; }
    sendVolume(finalVol);  // 最終値を確定送信
  }
}

// ---------- Wi-Fi ----------

// 初回接続 (最大15秒だけ待ち、失敗しても起動は継続する)
void connectWiFiInitial() {
  auto& d = M5.Display;
  d.fillScreen(TFT_BLACK);
  d.setFont(&fonts::FreeSans9pt7b);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextDatum(middle_center);
  d.drawString(String("Connecting: ") + WIFI_SSID, 160, 120);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t deadline = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(300);
  }
  Serial.printf("wifi: %s\n", WiFi.status() == WL_CONNECTED
                                  ? WiFi.localIP().toString().c_str()
                                  : "not connected (continuing)");
  d.fillScreen(TFT_BLACK);
  dirtyContent = dirtyVolume = dirtyHint = dirtyBattery = true;
}

// 切断時の再接続。networkTask 上で呼ぶので loop() をブロックしない
void ensureWiFi() {
  static uint32_t lastAttemptAt = 0;
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastAttemptAt < 5000) return;
  lastAttemptAt = millis();
  Serial.println("wifi: reconnecting");
  WiFi.reconnect();
}

// ---------- ネットワークタスク (core0) ----------
// コマンド送信とステータスポーリングをすべてここで行う。loop() (core1) は
// これに一切ブロックされないので、タッチ検知/描画が常に応答する
void networkTask(void*) {
  for (;;) {
    NetCmdMsg msg;
    while (xQueueReceive(cmdQueue, &msg, 0) == pdTRUE) {
      sendCommandBlocking(msg.command, *msg.args, msg.showSpinner);
      delete msg.args;
    }
    ensureWiFi();
    // ブラウズキャッシュの先読み: 接続済み・ブラウズ中でない・コマンド待ちなしのとき
    // 500ms 間隔で 1 ソースずつ取得 (失敗時の連打も この間隔が抑える)
    static uint32_t nextPrefetchAt = 0;
    if (prefetchStage >= 0 && millis() >= nextPrefetchAt &&
        WiFi.status() == WL_CONNECTED && uxQueueMessagesWaiting(cmdQueue) == 0) {
      bool busy;
      { StateLock lock; busy = browsing; }
      if (!busy) {
        nextPrefetchAt = millis() + (prefetchStep() ? 500 : 30000);
      }
    }
    if (millis() >= nextPollAt) {
      nextPollAt = millis() + kPollIntervalMs;
      // ブラウズ中は定期ポーリングを止め、core1 のリスト取得と
      // httpMutex を奪い合わないようにする (コマンド処理は継続)
      bool skipPoll;
      { StateLock lock; skipPoll = browsing; }
      if (WiFi.status() != WL_CONNECTED) {
        StateLock lock;
        if (online) { online = false; dirtyBattery = true; }
      } else if (!skipPoll) {
        uint32_t t0 = millis();
        pollStatus();
        pollQueueSettings();
        uint32_t dt = millis() - t0;
        if (dt > 2000) Serial.printf("poll slow: %u ms\n", dt);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = true;
  M5.begin(cfg);
  Serial.begin(115200);
  stateMutex = xSemaphoreCreateRecursiveMutex();
  httpMutex = xSemaphoreCreateMutex();
  cmdQueue = xQueueCreate(8, sizeof(NetCmdMsg));
  M5.Display.setRotation(kRotFor[orient]);
  M5.Display.setBrightness(120);
  nextLcdSleepAt = millis() + kLcdSleepTimeoutMs;
  xTaskCreatePinnedToCore(spinnerTask, "spinner", 2048, nullptr, 1, nullptr, 0);
  connectWiFiInitial();
  pollStatus();
  nextPollAt = millis() + kPollIntervalMs;
  redrawAll();
  // 通信処理を別タスクに分離し、loop() (タッチ/描画) を常に応答可能に保つ
  xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0);
}

void loop() {
  M5.update();
  M5.Imu.update();

  if (lcdSleeping) {
    // 液晶オフ中: タップで復帰するだけ。他の操作ハンドラは一切呼ばない
    // (ポーリング/Wi-Fi維持は networkTask がバックグラウンドで継続する)
    auto t = M5.Touch.getDetail();
    if (t.wasPressed()) {
      wakeLcd();
    } else {
      checkOrientation();
    }
    delay(20);
    return;
  }

  if (browsing) {
    // 選曲ブラウザ中: リスト操作のみ (通常画面の描画・マーキーは停止)
    handleBrowseButtons();
    handleBrowseTouch();
    checkOrientation();
    delay(10);
    return;
  }

  // バッテリー残量は変化が緩やかなので低頻度で更新する (アート拡大中も表示は残る)
  static uint32_t nextBatteryCheckAt = 0;
  if (millis() >= nextBatteryCheckAt) {
    nextBatteryCheckAt = millis() + 30000;
    dirtyBattery = true;
  }

  if (artZoom) {
    // アート拡大中: タップでの復帰のみ (曲名/スライダー/操作行は隠しているため
    // それらの入力ハンドラやマーキーは呼ばない。ポーリングは networkTask 側で継続する)
    handleContentTouch();
    checkOrientation();
    updateLcdSleep();
    if (!lcdSleeping) redraw();
    delay(20);
    return;
  }

  handleSliderTouch();
  handleHintTouch();
  handleContentTouch();
  checkOrientation();
  {
    StateLock lock;
    titleMarquee.tick();
    artistMarquee.tick();
  }

  updateLcdSleep();
  if (!lcdSleeping) redraw();
  delay(20);
}
