/******************************************************
 * esp32_ota_web.ino  — Single-file ESP32 OTA Webserver
 * - Wi-Fi STA with AP fallback
 * - Web UI: upload firmware, progress, live logs (polling)
 * - Streams .fw (FWUP header + data chunks) to STM32F103
 * - UART protocol: STX|TYPE|SEQ|LEN|PAYLOAD|CRC16, ACK/NACK+retry
 * - Endpoints: /, /upload (POST), /info, /enter_boot, /verify, /run, /reset, /log
 *
 * Wiring:
 *   ESP32 UART2: TX=GPIO17 → STM32 PA10 (USART1 RX)
 *                RX=GPIO16 ← STM32 PA9  (USART1 TX)  [optional to read replies; here we only need ACK/NACK]
 *   GND common.
 * Optional: GPIO25 → NRST (STM32) for hardware reset.
 ******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <FS.h>

// ====== Config Wi-Fi ======
const char* WIFI_SSID = "Tam";   // change to your home network
const char* WIFI_PASS = "mko9ijnh";

const char* AP_SSID   = "ESP32-OTA";
const char* AP_PASS   = "12345678";       // ≥8 characters

// ====== UART to STM32 ======
HardwareSerial Uart(2);
// Default pins for Serial2 on ESP32: RX2=GPIO16, TX2=GPIO17 (changeable below)
const int UART_RX_PIN = 16;
const int UART_TX_PIN = 17;
const uint32_t UART_BAUD = 115200;

// Optional NRST control
const int PIN_NRST = 25;   // set to -1 to disable
const bool NRST_ACTIVE_LOW = true;

// ===== Web server =====
WebServer server(80);

// ====== Log buffer (simple ring) ======
static const size_t LOG_MAX_LINES = 400;
String g_logs[LOG_MAX_LINES];
size_t g_log_head = 0;   // next write position
size_t g_log_count = 0;  // total lines ever pushed (monotonic)

// push a log line (will be retrievable via /log?since=<n>)
void push_log(const String& s) {
  g_logs[g_log_head] = s;
  g_log_head = (g_log_head + 1) % LOG_MAX_LINES;
  g_log_count++;
  // also print to serial monitor (optional)
  Serial.println(s);
}

// get logs newer than 'since' counter
String collect_logs_since(size_t since) {
  String out;
  size_t start = (g_log_count > LOG_MAX_LINES) ? (g_log_count - LOG_MAX_LINES) : 0;
  if (since < start) since = start;
  if (since >= g_log_count) return out;
  size_t lines_to_emit = g_log_count - since;
  // map "since index" to buffer index
  size_t first_index = (g_log_head + LOG_MAX_LINES - (g_log_count - since)) % LOG_MAX_LINES;
  for (size_t i = 0; i < lines_to_emit; ++i) {
    size_t idx = (first_index + i) % LOG_MAX_LINES;
    out += g_logs[idx] + "\n";
  }
  return out;
}

// ===== CRC16-CCITT (0x1021), init=0xFFFF =====
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else               crc = (crc << 1);
    }
  }
  return crc;
}

// ===== CRC32 (Ethernet, poly 0x04C11DB7, reflected) =====
// Standard CRC-32 (IEEE 802.3). init=0xFFFFFFFF, xorout=0xFFFFFFFF
uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

// ===== UART Protocol =====
enum : uint8_t {
  STX = 0x55,
  TYPE_H = 0x01,   // Header
  TYPE_D = 0x02,   // Data
  TYPE_E = 0x03,   // End
  TYPE_Q = 0x10    // Command
};

enum : uint8_t {
  CMD_INFO   = 1,
  CMD_BOOT   = 2,
  CMD_ERASE  = 3,
  CMD_BEGIN  = 4,
  CMD_VERIFY = 5,
  CMD_RUN    = 6
};

// ACK/NACK format: 0xAA seq or 0xEE seq
static const uint8_t ACK_BYTE  = 0xAA;
static const uint8_t NACK_BYTE = 0xEE;

// Send one framed packet and wait for ACK/NACK of the same seq.
// Send 1 frame and wait for ACK/NACK with same SEQ (timeout + retry).
bool send_frame_with_ack(uint8_t type, uint8_t seq, const uint8_t* payload, uint16_t len,
                         uint32_t timeout_ms, int max_retry) {
  // build frame
  size_t frame_len = 1 + 1 + 1 + 2 + len + 2; // STX+TYPE+SEQ+LEN+PAYLOAD+CRC16
  std::unique_ptr<uint8_t[]> buf(new uint8_t[frame_len]);
  uint8_t* p = buf.get();
  p[0] = STX;
  p[1] = type;
  p[2] = seq;
  p[3] = (uint8_t)(len & 0xFF);
  p[4] = (uint8_t)(len >> 8);
  if (len && payload) memcpy(p + 5, payload, len);
  uint16_t c16 = crc16_ccitt(p, 5 + len);
  p[5 + len] = (uint8_t)(c16 & 0xFF);
  p[6 + len - 1] = (uint8_t)(c16 >> 8); // careful: index = 5+len -> LSB; 6+len-1 -> MSB (== 5+len+1)

  // note: fix index: CRC16 LSB at [5+len], MSB at [6+len]? We need 2 bytes.
  // Let's correct precisely:
  p[5 + len]     = (uint8_t)(c16 & 0xFF);      // LSB
  p[5 + len + 1] = (uint8_t)((c16 >> 8) & 0xFF); // MSB

  for (int attempt = 0; attempt <= max_retry; ++attempt) {
    // flush RX buffer before send (avoid stale bytes)
    while (Uart.available()) Uart.read();

    // DEBUG: dump first bytes being sent (for CMD_BOOT will be 8 bytes)
    String dbg = "TX:";
    for (size_t i=0;i< (5 + len + 2); ++i) { // STX..CRC16
      char tmp[6]; sprintf(tmp," %02X", p[i]);
      dbg += tmp;
    }
    push_log(dbg);

    Uart.write(p, frame_len);
    Uart.flush();

    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms) {
      // waiting for ACK:
      if (Uart.available() >= 2) {
        uint8_t r0 = Uart.read();
        uint8_t r1 = Uart.read();
        char tmp[64];
        sprintf(tmp, "RX: %02X %02X (waiting seq=%u)", r0, r1, seq);
        push_log(tmp);
        if (r0 == ACK_BYTE && r1 == seq) return true;
        if (r0 == NACK_BYTE && r1 == seq) { push_log("NACK seq=" + String(seq)); break; }
      }

      delay(2);
    }
    // timeout -> retry
    push_log("Timeout waiting ACK seq=" + String(seq) + ", attempt " + String(attempt+1));
  }
  return false;
}

// Send command Q with 1-byte payload (CMD_*).
bool send_cmd(uint8_t cmd, uint8_t seq=0x00, uint32_t timeout_ms=500, int max_retry=2) {
  return send_frame_with_ack(TYPE_Q, seq, &cmd, 1, timeout_ms, max_retry);
}

// Stream FW: header (FWUP...), then data chunks, then END, then VERIFY.
// Returns true if the entire process is successful.
bool stream_firmware(File &f, uint16_t fw_ver, uint32_t fw_size, uint32_t fw_crc32, uint32_t fw_flags,
                     size_t chunk_bytes, uint32_t &elapsed_ms, float &kbps) {
  // Build FWUP header (little-endian)
  uint8_t hdr[4 + 2 + 4 + 4 + 4];
  memcpy(hdr, "FWUP", 4);
  hdr[4] = (uint8_t)(fw_ver & 0xFF);
  hdr[5] = (uint8_t)((fw_ver >> 8) & 0xFF);

  hdr[6] = (uint8_t)(fw_size & 0xFF);
  hdr[7] = (uint8_t)((fw_size >> 8) & 0xFF);
  hdr[8] = (uint8_t)((fw_size >> 16) & 0xFF);
  hdr[9] = (uint8_t)((fw_size >> 24) & 0xFF);

  hdr[10] = (uint8_t)(fw_crc32 & 0xFF);
  hdr[11] = (uint8_t)((fw_crc32 >> 8) & 0xFF);
  hdr[12] = (uint8_t)((fw_crc32 >> 16) & 0xFF);
  hdr[13] = (uint8_t)((fw_crc32 >> 24) & 0xFF);

  hdr[14] = (uint8_t)(fw_flags & 0xFF);
  hdr[15] = (uint8_t)((fw_flags >> 8) & 0xFF);
  hdr[16] = (uint8_t)((fw_flags >> 16) & 0xFF);
  hdr[17] = (uint8_t)((fw_flags >> 24) & 0xFF);

  // Sequence rolls over 0..255
  uint8_t seq = 1;

  push_log("Sending HEADER ver=" + String(fw_ver) + " size=" + String(fw_size) + " crc32=0x" + String(fw_crc32, HEX));
  if (!send_frame_with_ack(TYPE_H, seq++, hdr, sizeof(hdr), 500, 5)) {
    push_log("ERROR: HEADER failed");
    return false;
  }

  // optional: tell bootloader to BEGIN (reserve/prepare)
  if (!send_cmd(CMD_BEGIN, seq++, 500, 3)) {
    push_log("WARN: CMD_BEGIN not acknowledged (continuing)");
  }

  // Data chunks
  std::unique_ptr<uint8_t[]> buf(new uint8_t[chunk_bytes]);
  uint32_t sent = 0;
  uint32_t t0 = millis();
  while (sent < fw_size) {
    size_t to_read = min(chunk_bytes, (size_t)(fw_size - sent));
    size_t n = f.read(buf.get(), to_read);
    if (n != to_read) {
      push_log("ERROR: read SPIFFS " + String(n) + "/" + String(to_read));
      return false;
    }
    if (!send_frame_with_ack(TYPE_D, seq++, buf.get(), (uint16_t)n, 500, 5)) {
      push_log("ERROR: send DATA failed at " + String(sent));
      return false;
    }
    sent += n;
    // progress
    int pct = (int)((sent * 100ull) / fw_size);
    push_log("Chunk OK, sent=" + String(sent) + "/" + String(fw_size) + " (" + String(pct) + "%)");
  }

  // End
  if (!send_frame_with_ack(TYPE_E, seq++, nullptr, 0, 500, 3)) {
    push_log("ERROR: END failed");
    return false;
  }

  // Verify
  if (!send_cmd(CMD_VERIFY, seq++, 2000, 3)) {
    push_log("ERROR: VERIFY not ACKed");
    return false;
  }

  elapsed_ms = millis() - t0;
  kbps = (fw_size / 1024.0f) / (elapsed_ms / 1000.0f);
  push_log("Transfer done in " + String(elapsed_ms) + " ms (~" + String(kbps, 1) + " KB/s)");

  return true;
}

// ===== Simple HTML UI (inline) =====
const char* INDEX_HTML = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<title>ESP32 → STM32 OTA</title>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<style>
 body{font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:16px;max-width:900px}
 .card{border:1px solid #4443;padding:16px;border-radius:12px;margin-bottom:16px;background:#111;color:#eee}
 .row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
 .btn{padding:8px 14px;border-radius:10px;border:1px solid #666;background:#222;color:#eee;cursor:pointer}
 .btn:hover{background:#333}
 .log{height:260px;overflow:auto;white-space:pre-wrap;border:1px solid #333;padding:8px;background:#000;color:#0f0;font-family:ui-monospace,Consolas,monospace}
 .bar{height:8px;background:#333;border-radius:6px;overflow:hidden}
 .bar>div{height:100%;width:0;background:#09f}
 input[type=file]{border:1px dashed #666;padding:10px;border-radius:10px;background:#181818;color:#ddd}
 small{opacity:.7}
</style>
</head>
<body>
<h2>ESP32 → STM32F103 OTA Web</h2>

<div class="card">
  <div class="row">
    <button class="btn" id="btnInfo">Device Info</button>
    <button class="btn" id="btnBoot">Enter Bootloader</button>
    <button class="btn" id="btnVerify">Verify</button>
    <button class="btn" id="btnRun">Run App</button>
    <button class="btn" id="btnReset">Reboot ESP32</button>
  </div>
  <div id="info"><small>status...</small></div>
</div>

<div class="card">
  <h3>Upload Firmware (.bin)</h3>
  <input id="fw" type="file" accept=".bin,.fw"/>
  <div class="bar" style="margin:10px 0"><div id="prog"></div></div>
  <div id="status"><small>Idle</small></div>
  <div class="row">
    <label>FW Version: <input type="number" id="ver" value="2" min="1" style="width:80px"/></label>
    <label>Chunk Size:
      <select id="chunk">
        <option value="512">512</option>
        <option value="768">768</option>
        <option value="1024">1024</option>
      </select>
    </label>
  </div>
</div>

<div class="card">
  <h3>Logs</h3>
  <div id="log" class="log"></div>
</div>

<script>
const logEl = document.getElementById('log');
let since = 0;
function fetchLogs(){
  fetch('/log?since='+since).then(r=>r.text()).then(t=>{
    if (t.length>0){
      logEl.textContent += t;
      logEl.scrollTop = logEl.scrollHeight;
      since += (t.match(/\n/g)||[]).length;
    }
  }).catch(()=>{});
}
setInterval(fetchLogs, 600);

function $(id){return document.getElementById(id)}
function setProg(p){$('prog').style.width=p+'%'}
function setStatus(s){$('status').innerHTML='<small>'+s+'</small>'}

$('btnInfo').onclick=()=>fetch('/info').then(r=>r.text()).then(t=>$('info').innerHTML='<small>'+t+'</small>');
$('btnBoot').onclick=()=>fetch('/enter_boot').then(()=>{});
$('btnVerify').onclick=()=>fetch('/verify').then(()=>{});
$('btnRun').onclick=()=>fetch('/run').then(()=>{});
$('btnReset').onclick=()=>fetch('/reset').then(()=>{});

$('fw').addEventListener('change', async (ev)=>{
  const f = ev.target.files[0];
  if(!f){return;}
  setStatus('Uploading '+f.name+' ('+f.size+' bytes) ...');
  setProg(0);
  const ver = parseInt($('ver').value||'1');
  const chunk = parseInt($('chunk').value||'512');

  const form = new FormData();
  form.append('fw', f);
  form.append('ver', ver);
  form.append('chunk', chunk);

  const xhr = new XMLHttpRequest();
  xhr.open('POST','/upload');
  xhr.upload.onprogress = (e)=>{
    if (e.lengthComputable){
      setProg(Math.round(e.loaded*100/e.total));
      setStatus('Uploading... '+Math.round(e.loaded/1024)+' / '+Math.round(e.total/1024)+' KB');
    }
  };
  xhr.onload = ()=>{
    setProg(100);
    setStatus('Upload done: '+xhr.responseText);
  };
  xhr.onerror = ()=> setStatus('Upload failed');
  xhr.send(form);
});
</script>
</body>
</html>
)HTML";

// ====== Helpers ======
void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  push_log("WiFi: connecting to " + String(WIFI_SSID));
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    push_log("WiFi STA connected: " + WiFi.localIP().toString());
  } else {
    push_log("WiFi STA failed. Starting AP...");
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(AP_SSID, AP_PASS)) {
      push_log("AP: " + String(AP_SSID) + " IP=" + WiFi.softAPIP().toString());
    } else {
      push_log("AP start failed!");
    }
  }
}

void nrst_pulse() {
  if (PIN_NRST < 0) return;
  pinMode(PIN_NRST, OUTPUT);
  if (NRST_ACTIVE_LOW) {
    digitalWrite(PIN_NRST, LOW);
    delay(20);
    digitalWrite(PIN_NRST, HIGH);
  } else {
    digitalWrite(PIN_NRST, HIGH);
    delay(20);
    digitalWrite(PIN_NRST, LOW);
  }
  push_log("NRST pulse sent");
}

// ====== HTTP Handlers ======
void handle_root() {
  server.send(200, "text/html", INDEX_HTML);
}

void handle_info() {
  String s = "ESP32 OTA Web\n";
  s += "WiFi: " + String(WiFi.getMode()==WIFI_MODE_AP? "AP":"STA");
  s += " IP=" + (WiFi.getMode()==WIFI_MODE_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString());
  s += "\nUART: 115200 8N1, TX2=GPIO17 RX2=GPIO16\n";
  s += "Use /upload to send new firmware.\n";
  server.send(200, "text/plain", s);
}

void handle_log() {
  size_t since = 0;
  if (server.hasArg("since")) {
    since = (size_t) strtoul(server.arg("since").c_str(), nullptr, 10);
  }
  String out = collect_logs_since(since);
  server.send(200, "text/plain", out);
}

// Enter bootloader: send Q:BOOT (app should set boot_request and reset).
void handle_enter_boot() {
  push_log("[BTN] Enter Bootloader");
  bool ok = send_cmd(CMD_BOOT, 0x01, 500, 3);
  if (!ok) {
    push_log("BOOT cmd not ACKed. Trying NRST pulse...");
    nrst_pulse();
  }
  server.send(200, "text/plain", ok ? "BOOT command sent" : "BOOT timed out (maybe reset by NRST)");
}

// Ask bootloader to VERIFY (after upload) again:
void handle_verify() {
  push_log("[BTN] VERIFY");
  bool ok = send_cmd(CMD_VERIFY, 0x33, 2000, 2);
  server.send(200, "text/plain", ok ? "VERIFY ACK" : "VERIFY timeout");
}

void handle_run() {
  push_log("[BTN] RUN APP");
  bool ok = send_cmd(CMD_RUN, 0x44, 500, 2);
  server.send(200, "text/plain", ok ? "RUN ACK" : "RUN timeout");
}

void handle_reset() {
  push_log("[BTN] Reboot ESP32");
  server.send(200, "text/plain", "ESP32 restarting...");
  delay(100);
  ESP.restart();
}

// Write binary file — DO NOT read ver/chunk, DO NOT call server.send()
void handle_upload_store() {
  HTTPUpload& up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    SPIFFS.remove("/fw.bin");
    File f = SPIFFS.open("/fw.bin", FILE_WRITE);
    if (!f) { push_log("SPIFFS open fail"); return; }
    f.close();
    push_log("Upload start: " + String(up.filename.c_str()));
    return;
  }

  if (up.status == UPLOAD_FILE_WRITE) {
    File f = SPIFFS.open("/fw.bin", FILE_APPEND);
    if (!f) { push_log("SPIFFS write fail"); return; }
    f.write(up.buf, up.currentSize);
    f.close();
    return;
  }

  if (up.status == UPLOAD_FILE_END) {
    push_log("Upload done: size=" + String(up.totalSize) + " bytes");
    return;  // DO NOT send() here
  }
}

// Complete: read parameters, calculate CRC, stream via UART, return response
void handle_upload_done() {
  if (!server.hasArg("ver") || !server.hasArg("chunk")) {
    server.send(400, "text/plain", "Missing ver/chunk");
    push_log("Upload done: Missing ver/chunk");
    return;
  }

  uint16_t fw_ver    = (uint16_t)strtoul(server.arg("ver").c_str(),   nullptr, 10);
  size_t   chunk_sz  = (size_t)  strtoul(server.arg("chunk").c_str(), nullptr, 10);
  if (chunk_sz != 512 && chunk_sz != 768 && chunk_sz != 1024) chunk_sz = 512;

  // Calculate CRC + size
  File f = SPIFFS.open("/fw.bin", FILE_READ);
  if (!f) { server.send(500, "text/plain", "SPIFFS read fail"); return; }
  uint32_t crc = 0, total = 0;
  std::unique_ptr<uint8_t[]> buf(new uint8_t[1024]);
  int n;
  while ((n = f.read(buf.get(), 1024)) > 0) { crc = crc32_update(crc, buf.get(), n); total += n; }
  f.close();
  push_log("Uploaded " + String(total) + " bytes, CRC32=0x" + String(crc, HEX));

  // Send INFO/ERASE (best-effort), then stream
  (void)send_cmd(CMD_INFO,   0x20, 500, 1);
  (void)send_cmd(CMD_ERASE,  0x21, 2000, 1);

  File fw = SPIFFS.open("/fw.bin", FILE_READ);
  if (!fw) { server.send(500, "text/plain", "SPIFFS reopen fail"); return; }
  uint32_t elapsed = 0; float speed = 0;
  bool ok = stream_firmware(fw, fw_ver, total, crc, 0 /*flags*/, chunk_sz, elapsed, speed);
  fw.close();

  if (ok) {
    push_log("VERIFY requested OK. You can press 'Run App'.");
    char msg[96];
    snprintf(msg, sizeof(msg), "OK, %lu bytes, %.1f KB/s, %lums",
             (unsigned long)total, speed, (unsigned long)elapsed);
    server.send(200, "text/plain", msg);
  } else {
    server.send(500, "text/plain", "Stream failed (see logs)");
  }
}

// ====== setup/loop ======
void setup() {
  Serial.begin(115200);
  delay(50);
  push_log("ESP32 OTA Webserver booting...");

  if (!SPIFFS.begin(true)) {
    push_log("SPIFFS mount failed!");
  }

  setup_wifi();

  // UART2 to STM32
  Uart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  push_log("UART2 opened @115200");

  // Optional NRST idle state
  if (PIN_NRST >= 0) {
    pinMode(PIN_NRST, OUTPUT);
    if (NRST_ACTIVE_LOW) digitalWrite(PIN_NRST, HIGH);
    else                 digitalWrite(PIN_NRST, LOW);
  }

  // HTTP routes
  server.on("/", HTTP_GET, handle_root);
  server.on("/info", HTTP_GET, handle_info);
  server.on("/log", HTTP_GET, handle_log);
  server.on("/enter_boot", HTTP_GET, handle_enter_boot);
  server.on("/verify", HTTP_GET, handle_verify);
  server.on("/run", HTTP_GET, handle_run);
  server.on("/reset", HTTP_GET, handle_reset);

  // File upload
  server.on("/upload", HTTP_POST,
    handle_upload_done,     // FINAL: read ver/chunk, stream + send() response
    handle_upload_store     // UPLOAD: write /fw.bin, no send()
  );
  server.begin();
  push_log("HTTP server started");
}

void loop() {
  server.handleClient();
}
