// USB (UVC) camera viewer with FPS counter for ATD5-P4
//  - USB host stack + UVC class driver (MJPEG)
//  - JPEG decode with the ESP32-P4 hardware JPEG decoder (RGB565 output)
//  - Shown with Display.drawBitmap(), FPS drawn at the top-right of the image
#include <Arduino.h>
#include <ATD5-P4.h>
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "driver/jpeg_decode.h"

// Hybrid-compile (custom_sdkconfig) leaves "-Wl,--wrap=log_printf" in the link flags
// but not the wrapper itself, so provide it here.
extern "C" int __wrap_log_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int n = vprintf(fmt, ap);
  va_end(ap);
  return n;
}

// ---- Camera settings ----
// The mode is picked automatically from what the camera offers (MJPEG only):
// the largest frame not bigger than the screen (width/height multiple of 16 for the
// hardware decoder), with FPS closest to TARGET_FPS.
#define MAX_CAM_W   LCD_WIDTH
#define MAX_CAM_H   LCD_HEIGHT
#define TARGET_FPS  30
#define JPEG_MAX_SIZE (512 * 1024)

static unsigned camW = 0, camH = 0;
static float camFps = 0;
static int imgX = 0, imgY = 0;

static uint8_t  *jpegBuf = nullptr;   // latest MJPEG frame from camera (HW-decoder aligned)
static size_t    jpegLen = 0;
static volatile bool frameReady = false;
static uint16_t *outBuf = nullptr;    // decoded RGB565
static size_t    outBufSize = 0;
static SemaphoreHandle_t jpegMutex;
static volatile bool camConnected = false;
static jpeg_decoder_handle_t jpegDec = nullptr;
static volatile uint32_t statRx = 0, statSkip = 0, statBig = 0, statDecErr = 0;

// ---------------- Tiny 5x7 font for the FPS overlay ----------------
struct Glyph { char c; uint8_t col[5]; };
static const Glyph FONT[] = {
  {'0', {0x3E,0x51,0x49,0x45,0x3E}}, {'1', {0x00,0x42,0x7F,0x40,0x00}},
  {'2', {0x42,0x61,0x51,0x49,0x46}}, {'3', {0x21,0x41,0x45,0x4B,0x31}},
  {'4', {0x18,0x14,0x12,0x7F,0x10}}, {'5', {0x27,0x45,0x45,0x45,0x39}},
  {'6', {0x3C,0x4A,0x49,0x49,0x30}}, {'7', {0x01,0x71,0x09,0x05,0x03}},
  {'8', {0x36,0x49,0x49,0x49,0x36}}, {'9', {0x06,0x49,0x49,0x29,0x1E}},
  {'.', {0x00,0x60,0x60,0x00,0x00}}, {' ', {0x00,0x00,0x00,0x00,0x00}},
  {'F', {0x7F,0x09,0x09,0x09,0x01}}, {'P', {0x7F,0x09,0x09,0x09,0x06}},
  {'S', {0x46,0x49,0x49,0x49,0x31}},
};

static const uint8_t *findGlyph(char c) {
  for (auto &g : FONT) if (g.c == c) return g.col;
  return FONT[11].col; // space
}

// Draw text into a RGB565 buffer (stride = bufW) at its top-right corner
static void drawTextTopRight(uint16_t *buf, int bufW, int bufH, const char *text,
                             int scale, uint16_t fg, uint16_t bg) {
  const int charW = 6 * scale, h = 9 * scale;
  int len = strlen(text);
  int w = len * charW + 2 * scale;
  if (w > bufW || h > bufH) return;
  int x0 = bufW - w;
  for (int y = 0; y < h; y++) {
    uint16_t *row = buf + (size_t)y * bufW + x0;
    for (int x = 0; x < w; x++) {
      uint16_t px = bg;
      int gy = y / scale - 1, cx = x - scale;
      if (gy >= 0 && gy < 7 && cx >= 0) {
        int ci = cx / charW, gx = (cx % charW) / scale;
        if (ci < len && gx < 5 && (findGlyph(text[ci])[gx] >> gy & 1)) px = fg;
      }
      row[x] = px;
    }
  }
}

// ---------------- USB host / UVC ----------------
static void usbLibTask(void *) {
  while (true) {
    uint32_t flags;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
    if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
  }
}

static bool frameCallback(const uvc_host_frame_t *frame, void *) {
  statRx++;
  if (frame->data_len > JPEG_MAX_SIZE) statBig++;
  if (frameReady) statSkip++;
  if (!frameReady && frame->data_len > 0 && frame->data_len <= JPEG_MAX_SIZE) {
    if (xSemaphoreTake(jpegMutex, 0) == pdTRUE) {
      memcpy(jpegBuf, frame->data, frame->data_len);
      jpegLen = frame->data_len;
      frameReady = true;
      xSemaphoreGive(jpegMutex);
    }
  }
  return true; // frame already copied -> ownership goes back to the driver immediately
}

static void streamEventCallback(const uvc_host_stream_event_data_t *ev, void *) {
  if (ev->type == UVC_HOST_DEVICE_DISCONNECTED) {
    camConnected = false;
    uvc_host_stream_close(ev->device_disconnected.stream_hdl);
    Serial.println("Camera disconnected");
  } else if (ev->type == UVC_HOST_TRANSFER_ERROR) {
    Serial.printf("USB transfer error: %d\n", ev->transfer_error.error);
  }
}

// Read the camera's mode list and choose a MJPEG mode. Returns false if none.
static bool pickMode(uint8_t devAddr, uint8_t streamIdx) {
  static uvc_host_frame_info_t list[48];
  size_t n = sizeof(list) / sizeof(list[0]);
  esp_err_t err = uvc_host_get_frame_list(devAddr, streamIdx, &list, &n);
  if (err != ESP_OK) {
    Serial.printf("uvc_host_get_frame_list failed: %s\n", esp_err_to_name(err));
    return false;
  }
  Serial.printf("Camera offers %u modes:\n", (unsigned)n);
  long bestArea = 0;
  camFps = 0;
  for (size_t i = 0; i < n; i++) {
    const uvc_host_frame_info_t &f = list[i];
    const char *fmt = f.format == UVC_VS_FORMAT_MJPEG ? "MJPEG" : f.format == UVC_VS_FORMAT_YUY2 ? "YUY2" :
                      f.format == UVC_VS_FORMAT_H264 ? "H264" : f.format == UVC_VS_FORMAT_NV12 ? "NV12" : "other";
    // choose the FPS closest to TARGET_FPS (intervals are in 100 ns units)
    const uint32_t want = 10000000UL / TARGET_FPS;
    uint32_t iv = f.default_interval;
    if (f.interval_type == 0) {
      iv = want < f.interval_min ? f.interval_min : (want > f.interval_max ? f.interval_max : want);
    } else {
      long bestD = 0x7FFFFFFF;
      for (int k = 0; k < f.interval_type && k < CONFIG_UVC_INTERVAL_ARRAY_SIZE; k++) {
        long d = labs((long)f.interval[k] - (long)want);
        if (d < bestD) { bestD = d; iv = f.interval[k]; }
      }
    }
    float fps = iv ? 10000000.0f / iv : 0;
    Serial.printf("  %-5s %ux%u @ %.1f fps\n", fmt, f.h_res, f.v_res, fps);
    if (f.format != UVC_VS_FORMAT_MJPEG || f.h_res > MAX_CAM_W || f.v_res > MAX_CAM_H) continue;
    if ((f.h_res & 15) || (f.v_res & 15)) continue; // HW decoder output is 16-aligned
    long area = (long)f.h_res * f.v_res;
    if (area > bestArea || (area == bestArea && fps > camFps)) {
      bestArea = area; camW = f.h_res; camH = f.v_res; camFps = fps;
    }
  }
  if (!bestArea) {
    Serial.println("No usable MJPEG mode (<= screen size, multiple of 16) found.");
    return false;
  }
  imgX = (LCD_WIDTH - (int)camW) / 2;
  imgY = (LCD_HEIGHT - (int)camH) / 2;
  Serial.printf("Using MJPEG %ux%u @ %.1f fps\n", camW, camH, camFps);
  return true;
}

static void openCamera() {
  const uvc_host_stream_config_t cfg = {
    .event_cb = streamEventCallback,
    .frame_cb = frameCallback,
    .user_ctx = nullptr,
    .usb = { .dev_addr = UVC_HOST_ANY_DEV_ADDR, .vid = UVC_HOST_ANY_VID,
             .pid = UVC_HOST_ANY_PID, .uvc_stream_index = 0 },
    .vs_format = { .h_res = camW, .v_res = camH, .fps = camFps,
                   .format = UVC_VS_FORMAT_MJPEG },
    .advanced = { .number_of_frame_buffers = 3, .frame_size = 0,
                  .frame_heap_caps = MALLOC_CAP_SPIRAM, .number_of_urbs = 3,
                  .urb_size = 0, .user_frame_buffers = nullptr },
  };
  uvc_host_stream_hdl_t hdl = nullptr;
  esp_err_t err = uvc_host_stream_open(&cfg, 1000, &hdl);
  if (err != ESP_OK) {
    Serial.printf("uvc_host_stream_open failed: %s\n", esp_err_to_name(err));
    return;
  }
  err = uvc_host_stream_start(hdl);
  if (err != ESP_OK) {
    Serial.printf("uvc_host_stream_start failed: %s\n", esp_err_to_name(err));
    uvc_host_stream_close(hdl);
    return;
  }
  camConnected = true;
  Serial.println("Camera streaming");
}

static volatile bool newDevice = false;
static uint8_t newDevAddr = 0, newStreamIdx = 0;
static void driverEventCallback(const uvc_host_driver_event_data_t *ev, void *) {
  if (ev->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
    newDevAddr = ev->device_connected.dev_addr;
    newStreamIdx = ev->device_connected.uvc_stream_index;
    newDevice = true;
  }
}

// Decode jpegBuf into outBuf with the hardware decoder. Returns true on success.
static bool decodeFrame() {
  jpeg_decode_cfg_t cfg = {};
  cfg.output_format = JPEG_DECODE_OUT_FORMAT_RGB565;
  cfg.rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR; // standard little-endian RGB565
  cfg.conv_std = JPEG_YUV_RGB_CONV_STD_BT601;
  uint32_t outSize = 0;
  esp_err_t err = jpeg_decoder_process(jpegDec, &cfg, jpegBuf, (uint32_t)jpegLen,
                                       (uint8_t *)outBuf, (uint32_t)outBufSize, &outSize);
  return err == ESP_OK;
}

void setup() {
  Serial.begin(115200);

  Display.begin(0, 100);
  Display.fillScreen(0x0000);

  jpegMutex = xSemaphoreCreateMutex();

  // Buffers for the HW JPEG decoder must come from jpeg_alloc_decoder_mem()
  size_t got = 0;
  jpeg_decode_memory_alloc_cfg_t inCfg = { .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER };
  jpeg_decode_memory_alloc_cfg_t outCfg = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
  jpegBuf = (uint8_t *)jpeg_alloc_decoder_mem(JPEG_MAX_SIZE, &inCfg, &got);
  outBufSize = (size_t)LCD_WIDTH * LCD_HEIGHT * 2;
  outBuf = (uint16_t *)jpeg_alloc_decoder_mem(outBufSize, &outCfg, &got);
  if (!jpegBuf || !outBuf) {
    Serial.println("Out of memory (PSRAM)");
    while (true) delay(1000);
  }
  outBufSize = got;

  jpeg_decode_engine_cfg_t engCfg = { .intr_priority = 0, .timeout_ms = 100 };
  ESP_ERROR_CHECK(jpeg_new_decoder_engine(&engCfg, &jpegDec));

  // USB host + UVC driver
  const usb_host_config_t hostCfg = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
  ESP_ERROR_CHECK(usb_host_install(&hostCfg));
  xTaskCreatePinnedToCore(usbLibTask, "usb_lib", 4096, nullptr, 10, nullptr, 0);

  const uvc_host_driver_config_t drvCfg = {
    .driver_task_stack_size = 4096,
    .driver_task_priority = 11,
    .xCoreID = 0,
    .create_background_task = true,
    .event_cb = driverEventCallback,
    .user_ctx = nullptr,
  };
  ESP_ERROR_CHECK(uvc_host_install(&drvCfg));
  Serial.println("Waiting for USB camera...");
}

void loop() {
  static uint32_t statT = 0, decUs = 0, drawUs = 0, shown = 0;
  static uint32_t fpsCount = 0, fpsT0 = millis();
  static float fps = 0;
  static bool shownWait = false;

  if (camConnected && millis() - statT >= 1000) {
    statT = millis();
    unsigned n = shown ? shown : 1;
    Serial.printf("[stat] rx=%u skipped=%u too_big=%u decode_err=%u | shown=%u avg decode=%u us draw=%u us jpeg=%u B\n",
                  (unsigned)statRx, (unsigned)statSkip, (unsigned)statBig, (unsigned)statDecErr,
                  (unsigned)shown, (unsigned)(decUs / n), (unsigned)(drawUs / n), (unsigned)jpegLen);
  }

  if (newDevice) {
    newDevice = false;
    if (pickMode(newDevAddr, newStreamIdx)) openCamera();
  }

  if (!frameReady) {
    if (!camConnected && !shownWait) {
      shownWait = true;
      Display.fillScreen(0x0000);
    }
    delay(1);
    return;
  }
  shownWait = false;

  // Decode the latest frame (camera callback may not overwrite it meanwhile)
  bool ok = false;
  uint32_t t0 = micros();
  if (xSemaphoreTake(jpegMutex, portMAX_DELAY) == pdTRUE) {
    ok = decodeFrame();
    frameReady = false;
    xSemaphoreGive(jpegMutex);
  }
  decUs += micros() - t0;
  if (!ok) {
    statDecErr++;
    return;
  }

  // FPS: update twice a second
  fpsCount++;
  uint32_t now = millis();
  if (now - fpsT0 >= 500) {
    fps = fpsCount * 1000.0f / (now - fpsT0);
    fpsCount = 0;
    fpsT0 = now;
  }
  char txt[16];
  snprintf(txt, sizeof(txt), "FPS %.1f", fps);
  drawTextTopRight(outBuf, camW, camH, txt, 3, 0xFFE0, 0x0000); // yellow on black

  t0 = micros();
  Display.drawBitmap(imgX, imgY, imgX + camW, imgY + camH, outBuf);
  drawUs += micros() - t0;
  shown++;
}
