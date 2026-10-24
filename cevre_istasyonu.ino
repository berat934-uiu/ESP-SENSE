/*
 * ============================================================================
 *  ESP32 ÇEVRE İSTASYONU  -  v2.4 (kararlılık düzeltmeleri)
 *  Sensörler : DHT11 (sıcaklık/nem), Mikrofon (analog), LDR (Voltaj Bölücü)
 *  Ekran     : 20x4 I2C LCD (0x27)
 *  Arayüz    : Gömülü web paneli (canlı grafik, 4 sensör)
 *
 *  v2.4 değişiklikleri:
 *   - Sensör görevi önceliği web sunucusuyla eşitlendi (öncelik 1), döngü
 *     beklemesi 30 ms: handleClient() artık aç kalmıyor.
 *   - Kalibrasyon sırasında vTaskDelay ile sunucuya nefes aldırılıyor.
 *   - Wi-Fi: persistent(false), autoReconnect, WiFi.reconnect(), olay
 *     yakalayıcı ile kopma sebep kodu seri monitöre yazılıyor.
 *   - HTTP cevaplarında "Connection: close" (takılı soketleri önler).
 *   - Web paneli: fetch zaman aşımı (3 sn), yoklama aralığı 1000 ms.
 * ============================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DHT.h>
#include <math.h>

// ============================== YAPILANDIRMA ================================
namespace Config {
  // Pinler
  constexpr uint8_t PIN_DHT = 23;
  constexpr uint8_t PIN_MIC = 34;   // ADC1 - Mikrofon AO
  constexpr uint8_t PIN_LDR = 35;   // ADC1 - LDR Sinyal (Voltaj Bölücü)

  // Wi-Fi
  const char* WIFI_SSID = "bip";
  const char* WIFI_PASS = "12345678";
  const char* HOSTNAME  = "cevre";
  constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
  constexpr uint32_t WIFI_RETRY_INTERVAL_MS  = 15000;
  const char* AP_SSID = "ESP32-Cevre";
  const char* AP_PASS = "12345678";

  // Zamanlama
  constexpr uint32_t DHT_PERIOD_MS     = 2000;
  constexpr uint32_t DISPLAY_PERIOD_MS = 250;
  constexpr uint32_t MIC_WINDOW_MS     = 40;   // RMS örnek penceresi (ms)
  constexpr uint32_t SENSOR_LOOP_MS    = 30;   // sensör görevi bekleme (web sunucusuna zaman bırakır)

  // MIC (RMS) Kalibrasyon / Filtre
  constexpr int  MIC_RMS_FULLSCALE = 300;
  constexpr float EMA_LDR          = 0.20f;
  constexpr float EMA_MIC_ATTACK   = 0.70f;
  constexpr float EMA_MIC_RELEASE  = 0.25f;
  constexpr uint32_t MIC_CALIB_MS  = 3000;

  // LCD
  constexpr uint8_t LCD_ADDR = 0x27;
  constexpr uint8_t LCD_COLS = 20;
  constexpr uint8_t LCD_ROWS = 4;
}

// ============================= ORTAK DURUM ==================================
struct SensorData {
  float    temperature = 0.0f;
  float    humidity    = 0.0f;
  bool     dhtOk       = false;
  float    sound       = 0.0f;
  float    light       = 0.0f;
};

namespace SharedState {
  SensorData data;
  SemaphoreHandle_t mutex = nullptr;

  void init() { mutex = xSemaphoreCreateMutex(); }

  void write(const SensorData& d) {
    if (!mutex) return;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(20))) { data = d; xSemaphoreGive(mutex); }
  }
  SensorData read() {
    SensorData copy;
    if (!mutex) return copy;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(20))) { copy = data; xSemaphoreGive(mutex); }
    return copy;
  }
}

// ============================== NESNELER ====================================
DHT dht(Config::PIN_DHT, DHT11);
LiquidCrystal_I2C lcd(Config::LCD_ADDR, Config::LCD_COLS, Config::LCD_ROWS);
WebServer server(80);

bool apMode = false;
volatile bool micCalibRequested  = false; // Seri'den 'c' ile set edilir
volatile bool httpCalibRequested = false; // /api/calibrate ile set edilir
volatile int  lastDisconnectReason = 0;   // son Wi-Fi kopma sebep kodu

// ============================ SENSOR AJANI ==================================
namespace SensorAgent {

  static float micFullScale = (float)Config::MIC_RMS_FULLSCALE;

  // Ham RMS değeri (normalize edilmemiş)
  float sampleSoundRMS_raw() {
    const uint32_t window_us = Config::MIC_WINDOW_MS * 1000u;
    const uint32_t start = micros();
    uint64_t sumSq = 0;
    uint64_t sum = 0;
    uint32_t count = 0;

    while ((uint32_t)(micros() - start) < window_us) {
      int v = analogRead(Config::PIN_MIC);
      sumSq += (uint64_t)v * (uint64_t)v;
      sum += (uint64_t)v;
      count++;
    }

    if (count == 0) return 0.0f;
    float mean = (float)sum / (float)count;
    float meanSq = (float)sumSq / (float)count;
    float variance = meanSq - (mean * mean);
    if (variance < 0.0f) variance = 0.0f;
    return sqrtf(variance);
  }

  float sampleSound_pct_from_raw(float rawRms) {
    if (micFullScale <= 0.001f) return 0.0f;
    float pct = (rawRms / micFullScale) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
  }

  // Kalibrasyon: süre boyunca maksimum RMS'i gözle, scale = max*1.2
  void performCalibration(uint32_t durationMs) {
    Serial.printf("[MIC CALIB] Basliyor %lums\n", (unsigned long)durationMs);
    uint32_t t0 = millis();
    float observedMax = 1.0f;
    while (millis() - t0 < durationMs) {
      float r = sampleSoundRMS_raw();
      if (r > observedMax) observedMax = r;
      vTaskDelay(pdMS_TO_TICKS(5));   // web sunucusuna ve Wi-Fi'ye zaman bırak
    }
    if (observedMax < 1.0f) observedMax = 1.0f;
    micFullScale = observedMax * 1.2f;
    Serial.printf("[MIC CALIB] tamam. observedMax=%.2f -> micFullScale=%.2f\n", observedMax, micFullScale);
  }

  float sampleLight() {
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) sum += analogRead(Config::PIN_LDR);
    float raw = (float)sum / 8.0f;
    float pct = (raw / 4095.0f) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
  }

  void task(void*) {
    SensorData local = SharedState::read();
    float fSound = local.sound;
    float fLight = (local.light > 0.0f) ? local.light : -1.0f;
    uint32_t lastDht = 0;

    for (;;) {
      if (micCalibRequested || httpCalibRequested) {
        micCalibRequested = false;
        httpCalibRequested = false;
        performCalibration(Config::MIC_CALIB_MS);
      }

      // --- Ses (RMS)
      float rawRms = sampleSoundRMS_raw();
      float rawPct = sampleSound_pct_from_raw(rawRms);
      float a = (rawPct > fSound) ? Config::EMA_MIC_ATTACK : Config::EMA_MIC_RELEASE;
      if (!isnan(fSound)) fSound += a * (rawPct - fSound);
      else fSound = rawPct;
      local.sound = fSound;

      // --- LDR (EMA)
      float rawLight = sampleLight();
      fLight = (fLight < 0) ? rawLight : fLight + Config::EMA_LDR * (rawLight - fLight);
      local.light = fLight;

      // --- DHT11
      if (millis() - lastDht >= Config::DHT_PERIOD_MS) {
        lastDht = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t) && !isnan(h)) {
          local.temperature = t;
          local.humidity    = h;
          local.dhtOk       = true;
        } else {
          local.dhtOk = false;
        }
      }

      SharedState::write(local);
      vTaskDelay(pdMS_TO_TICKS(Config::SENSOR_LOOP_MS));
    }
  }
}

// ============================ EKRAN AJANI ===================================
namespace DisplayAgent {

  void printRow(uint8_t row, const char* text) {
    char buf[Config::LCD_COLS + 1];
    snprintf(buf, sizeof(buf), "%-*.*s", Config::LCD_COLS, Config::LCD_COLS, text);
    lcd.setCursor(0, row);
    lcd.print(buf);
  }

  void makeBar(char* out, float pct, int width) {
    int fill = (int)roundf((pct / 100.0f) * width);
    if (fill > width) fill = width;
    if (fill < 0) fill = 0;
    for (int i = 0; i < width; i++) out[i] = (i < fill) ? (char)0xFF : '.';
    out[width] = '\0';
  }

  void splash(const char* l1, const char* l2, const char* l3 = "", const char* l4 = "") {
    lcd.clear();
    printRow(0, l1); printRow(1, l2); printRow(2, l3); printRow(3, l4);
  }

  void render(const SensorData& d) {
    char line[32], bar[16];

    if (d.dhtOk) snprintf(line, sizeof(line), "SIC:%.1f%cC NEM:%.1f%%", d.temperature, (char)223, d.humidity);
    else         snprintf(line, sizeof(line), "SIC:--.-%cC NEM:--.-%%", (char)223);
    printRow(0, line);

    makeBar(bar, d.sound, 6);
    snprintf(line, sizeof(line), "SES :[%-6s] %4.1f%%", bar, d.sound);
    printRow(1, line);

    makeBar(bar, d.light, 6);
    snprintf(line, sizeof(line), "ISIK:[%-6s] %4.1f%%", bar, d.light);
    printRow(2, line);

    // Satır 3: IP veya durum (kopma sebebi varsa göster)
    if (WiFi.status() == WL_CONNECTED)      snprintf(line, sizeof(line), "IP:%s", WiFi.localIP().toString().c_str());
    else if (apMode)                        snprintf(line, sizeof(line), "AP:%s R:%d", WiFi.softAPIP().toString().c_str(), (int)lastDisconnectReason);
    else                                    snprintf(line, sizeof(line), "WiFi OFFLINE R:%d", (int)lastDisconnectReason);
    printRow(3, line);
  }

  void task(void*) {
    for (;;) {
      render(SharedState::read());
      vTaskDelay(pdMS_TO_TICKS(Config::DISPLAY_PERIOD_MS));
    }
  }
}

// ============================ WEB ARAYÜZÜ ===================================
static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="tr"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Çevre İstasyonu</title>
<style>
:root{--bg:#070b14;--panel:rgba(17,25,40,.72);--line:rgba(255,255,255,.07);--tx:#e6edf7;--mut:#7d8ba3;
--t:#ff7a3d;--h:#22d3ee;--m:#b172ff;--l:#fbbf24;--ok:#22c55e;--bad:#ef4444}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--tx);
min-height:100vh;padding:20px;background-image:radial-gradient(900px 500px at 15% -10%,rgba(99,102,241,.22),transparent),
radial-gradient(700px 400px at 100% 0,rgba(34,211,238,.12),transparent)}
.app{max-width:1000px;margin:0 auto}
header{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:22px}
.brand{display:flex;align-items:center;gap:12px}
.logo{width:40px;height:40px;border-radius:12px;display:grid;place-items:center;font-weight:800;
background:linear-gradient(135deg,#6366f1,#22d3ee);box-shadow:0 8px 24px -8px #6366f1}
h1{font-size:18px;letter-spacing:.3px}
.sub{font-size:12px;color:var(--mut);font-family:ui-monospace,Consolas,monospace}
.badge{display:flex;align-items:center;gap:8px;padding:7px 14px;border-radius:999px;border:1px solid var(--line);
background:var(--panel);font-size:12px;color:var(--mut)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--bad)}
.on .dot{background:var(--ok);box-shadow:0 0 10px var(--ok);animation:p 1.6s infinite}
@keyframes p{50%{opacity:.35}}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}
@media(max-width:860px){.grid{grid-template-columns:repeat(2,1fr)}}
@media(max-width:460px){.grid{grid-template-columns:1fr}}
.card{background:var(--panel);backdrop-filter:blur(14px);border:1px solid var(--line);border-radius:18px;padding:18px;
position:relative;overflow:hidden;box-shadow:0 18px 40px -20px rgba(0,0,0,.7)}
.card::before{content:"";position:absolute;inset:0 0 auto 0;height:2px;background:var(--c);opacity:.8}
.lab{font-size:11px;font-weight:600;letter-spacing:.9px;text-transform:uppercase;color:var(--mut);display:flex;justify-content:space-between}
.val{font-size:36px;font-weight:800;margin:8px 0 10px;color:var(--c);font-variant-numeric:tabular-nums}
.val small{font-size:15px;font-weight:500;opacity:.65;margin-left:3px}
.bar{height:6px;border-radius:6px;background:rgba(255,255,255,.07);overflow:hidden;margin-bottom:10px}
.bar i{display:block;height:100%;width:0;background:var(--c);border-radius:6px;transition:width .35s ease}
canvas{width:100%;height:56px;display:block}
.row{display:grid;grid-template-columns:1fr 1.4fr;gap:14px;margin-top:14px}
@media(max-width:700px){.row{grid-template-columns:1fr}}
.panel h3{font-size:11px;letter-spacing:.9px;text-transform:uppercase;color:var(--mut);margin-bottom:12px}
.kv{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px dashed var(--line);font-size:13px}
.kv:last-child{border:0}.kv b{font-family:ui-monospace,Consolas,monospace;font-weight:600}
.log{font-family:ui-monospace,Consolas,monospace;font-size:12px;height:190px;overflow:auto;line-height:1.7}
.log div{color:var(--mut)}.log span{color:#5b6b86;margin-right:8px}
.log .w{color:var(--l)}.log .e{color:var(--bad)}.log .g{color:var(--ok)}
footer{text-align:center;color:var(--mut);font-size:11px;margin-top:18px}
</style></head><body><div class="app">
<header>
 <div class="brand"><div class="logo">E</div><div><h1>Çevre İstasyonu</h1><div class="sub">esp32 · telemetri paneli v2.4</div></div></div>
 <div class="badge" id="conn"><span class="dot"></span><span id="connTxt">Bağlanıyor…</span></div>
</header>
<section class="grid">
 <div class="card" style="--c:var(--t)"><div class="lab">Sıcaklık<span>DHT11</span></div><div class="val" id="vT">--<small>°C</small></div><div class="bar"><i id="bT"></i></div><canvas id="cT"></canvas></div>
 <div class="card" style="--c:var(--h)"><div class="lab">Nem<span>DHT11</span></div><div class="val" id="vH">--<small>%</small></div><div class="bar"><i id="bH"></i></div><canvas id="cH"></canvas></div>
 <div class="card" style="--c:var(--m)"><div class="lab">Ses<span>MIC</span></div><div class="val" id="vM">--<small>%</small></div><div class="bar"><i id="bM"></i></div><canvas id="cM"></canvas></div>
 <div class="card" style="--c:var(--l)"><div class="lab">Işık<span>LDR</span></div><div class="val" id="vL">--<small>%</small></div><div class="bar"><i id="bL"></i></div><canvas id="cL"></canvas></div>
</section>
<section class="row">
 <div class="card panel" style="--c:#6366f1"><h3>Sistem</h3>
  <div class="kv"><span>IP adresi</span><b id="sIp">-</b></div>
  <div class="kv"><span>Wi-Fi sinyali</span><b id="sRssi">-</b></div>
  <div class="kv"><span>Boş bellek</span><b id="sHeap">-</b></div>
  <div class="kv"><span>Çalışma süresi</span><b id="sUp">-</b></div>
  <div class="kv"><span>Gecikme</span><b id="sLat">-</b></div>
  <div class="kv"><span>DHT durumu</span><b id="sDht">-</b></div>
 </div>
 <div class="card panel" style="--c:#22d3ee"><h3>Olay günlüğü</h3><div class="log" id="log"></div></div>
</section>
<footer>Veri akışı 1 sn · yerel ağ üzerinden doğrudan ESP32'den</footer>
</div>
<script>
const $=id=>document.getElementById(id),N=60;
const H={T:[],H:[],M:[],L:[]},COL={T:'#ff7a3d',H:'#22d3ee',M:'#b172ff',L:'#fbbf24'};
let online=null,lastLoud=0,lastDark=null,fails=0;
function log(m,c=''){const d=document.createElement('div'),t=new Date().toLocaleTimeString('tr-TR');
 d.innerHTML='<span>'+t+'</span><b class="'+c+'">'+m+'</b>';const l=$('log');l.prepend(d);while(l.children.length>60)l.lastChild.remove()}
function spark(k,lo,hi){const c=$('c'+k),r=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
 c.width=w*r;c.height=h*r;const x=c.getContext('2d');x.scale(r,r);const a=H[k];if(a.length<2)return;
 const pt=(v,i)=>[i*(w/(N-1)),h-2-((v-lo)/(hi-lo))*(h-6)];
 x.beginPath();a.forEach((v,i)=>{const[p,q]=pt(v,i+N-a.length);i?x.lineTo(p,q):x.moveTo(p,q)});
 x.strokeStyle=COL[k];x.lineWidth=2;x.lineJoin='round';x.stroke();
 x.lineTo(w,h);x.lineTo((N-a.length)*(w/(N-1)),h);x.closePath();
 const g=x.createLinearGradient(0,0,0,h);g.addColorStop(0,COL[k]+'55');g.addColorStop(1,COL[k]+'00');x.fillStyle=g;x.fill()}
function set(k,v,txt,pct,lo,hi){$('v'+k).firstChild.nodeValue=txt;$('b'+k).style.width=Math.max(0,Math.min(100,pct))+'%';
 H[k].push(v);if(H[k].length>N)H[k].shift();spark(k,lo,hi)}
function up(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'s '+m+'dk '+(s%60)+'sn'}
function setConn(o){if(o===online)return;online=o;$('conn').className='badge'+(o?' on':'');
 $('connTxt').textContent=o?'Canlı bağlantı':'Bağlantı koptu';log(o?'Cihaza bağlanıldı':'Cihaza ulaşılamıyor',o?'g':'e')}
async function tick(){const t0=performance.now();
 const ctl=new AbortController(),to=setTimeout(()=>ctl.abort(),3000);
 try{const r=await fetch('/api/data',{cache:'no-store',signal:ctl.signal});
  const d=await r.json();clearTimeout(to);fails=0;setConn(true);
  set('T',d.temp,d.temp.toFixed(1),d.temp/50*100,0,50);
  set('H',d.hum,d.hum.toFixed(1),d.hum,0,100);
  set('M',d.mic,d.mic.toFixed(1),d.mic,0,100);
  set('L',d.ldr,d.ldr.toFixed(1),d.ldr,0,100);
  $('sIp').textContent=d.ip;$('sRssi').textContent=d.rssi?d.rssi+' dBm':'AP modu';
  $('sHeap').textContent=(d.heap/1024).toFixed(0)+' KB';$('sUp').textContent=up(d.up);
  $('sLat').textContent=Math.round(performance.now()-t0)+' ms';
  $('sDht').textContent=d.dht?'Sağlıklı':'Okunamıyor';
  if(d.mic>=70&&Date.now()-lastLoud>5000){lastLoud=Date.now();log('Yüksek ses algılandı (%'+d.mic.toFixed(1)+')','w')}
  const dark=d.ldr<15;if(lastDark!==null&&dark!==lastDark)log(dark?'Ortam karardı':'Ortam aydınlandı','w');lastDark=dark;
 }catch(e){clearTimeout(to);fails++;if(fails>=2)setConn(false)}
 setTimeout(tick,1000)}
log('Panel başlatıldı v2.4');tick();
</script></body></html>)HTML";

// ============================ AĞ AJANI ======================================
namespace NetworkAgent {

  void handleRoot() {
    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Connection", "close");
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  }

  void handleData() {
    SensorData d = SharedState::read();
    char json[512];
    String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    snprintf(json, sizeof(json),
      "{\"temp\":%.1f,\"hum\":%.1f,\"mic\":%.1f,\"ldr\":%.1f,\"dht\":%s,"
      "\"rssi\":%d,\"heap\":%u,\"up\":%lu,\"ip\":\"%s\"}",
      d.temperature, d.humidity, d.sound, d.light, d.dhtOk ? "true" : "false",
      (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0,
      (unsigned)ESP.getFreeHeap(), (unsigned long)(millis() / 1000), ip.c_str());
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", json);
  }

  void handleCalibrate() {
    httpCalibRequested = true;
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"calibration started\"}");
    Serial.println("[HTTP] calibration requested");
  }

  void handlePing() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "pong");
  }

  void handleNotFound() { server.send(404, "text/plain", "404 - Bulunamadi"); }

  // Wi-Fi olayları: kopma sebep kodunu yakala
  void onWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        lastDisconnectReason = info.wifi_sta_disconnected.reason;
        Serial.printf("[NET] KOPTU, sebep kodu=%d\n", (int)lastDisconnectReason);
        break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        lastDisconnectReason = 0;
        Serial.printf("[NET] IP alindi: %s\n", WiFi.localIP().toString().c_str());
        break;
      default:
        break;
    }
  }

  bool connectStation() {
    WiFi.persistent(false);          // flash'a sürekli yazma
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(Config::HOSTNAME);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    Serial.printf("[NET] WiFi'ye baglaniliyor: %s\n", Config::WIFI_SSID);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASS);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < Config::WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    return WiFi.status() == WL_CONNECTED;
  }

  void startAccessPoint() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.softAP(Config::AP_SSID, Config::AP_PASS);
    apMode = true;
    Serial.printf("[NET] AP modu: %s -> %s\n", Config::AP_SSID, WiFi.softAPIP().toString().c_str());
  }

  void begin() {
    WiFi.onEvent(onWiFiEvent);

    DisplayAgent::splash("  CEVRE ISTASYONU", "WiFi baglaniyor...", (String("SSID: ") + Config::WIFI_SSID).c_str());
    if (connectStation()) {
      Serial.printf("[NET] Baglandi: %s\n", WiFi.localIP().toString().c_str());
      DisplayAgent::splash("  CEVRE ISTASYONU", "DURUM: BAGLANDI", (String("IP:") + WiFi.localIP().toString()).c_str(), "Web sunucu aktif");
    } else {
      startAccessPoint();
      DisplayAgent::splash("  CEVRE ISTASYONU", "WiFi YOK -> AP modu", (String("AP:") + Config::AP_SSID).c_str(), WiFi.softAPIP().toString().c_str());
    }
    if (MDNS.begin(Config::HOSTNAME)) MDNS.addService("http", "tcp", 80);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/data", HTTP_GET, handleData);
    server.on("/api/calibrate", HTTP_GET, handleCalibrate);
    server.on("/ping", HTTP_GET, handlePing);
    server.on("/data", HTTP_GET, handleData);
    server.onNotFound(handleNotFound);
    server.begin();
    delay(500);
    lcd.clear();

    Serial.println("[NET] Web sunucu hazir");
  }

  void service() {
    server.handleClient();

    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 5000) {
      lastStatus = millis();
      String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
      Serial.printf("[NET STATUS] WiFi=%d IP=%s RSSI=%d Heap=%u\n",
                    WiFi.status(), ip.c_str(),
                    (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0,
                    (unsigned)ESP.getFreeHeap());
    }

    // Bağlantı yoksa nazikçe yeniden bağlan (disconnect+begin yerine reconnect)
    static uint32_t lastRetry = 0;
    if (WiFi.status() != WL_CONNECTED && millis() - lastRetry > Config::WIFI_RETRY_INTERVAL_MS) {
      lastRetry = millis();
      Serial.println("[NET] Yeniden baglaniliyor...");
      WiFi.reconnect();
    }
  }
}

// ============================== BAŞLATMA ====================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32 Cevre Istasyonu v2.4 (kararlilik duzeltmeleri)");

  SharedState::init();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  dht.begin();
  lcd.init();
  lcd.backlight();

  NetworkAgent::begin();

  // Öncelik 1: loop() (web sunucusu) ile eşit, böylece sunucu aç kalmaz
  xTaskCreatePinnedToCore(SensorAgent::task,  "SensorAgent",  4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(DisplayAgent::task, "DisplayAgent", 4096, nullptr, 1, nullptr, 1);

  Serial.println("[BOOT] Ajanlar calisiyor");
  Serial.println("[INFO] Ses kalibrasyonu icin Seri'ye 'c' gonder (3s olcum) veya http://<IP>/api/calibrate");
}

void loop() {
  NetworkAgent::service();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') {
      micCalibRequested = true;
      Serial.println("[CMD] mic calibration requested");
    }
  }

  delay(2);
}
