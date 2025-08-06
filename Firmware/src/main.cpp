#include <FastLED.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <QMC5883LCompass.h>
#include <esp_log.h>
#include <esp_task_wdt.h>

#include "board.h"
#include "event.h"

using namespace mcompass;

ESP_EVENT_DEFINE_BASE(MCOMPASS_EVENT);

static const char *TAG = "MAIN";
esp_event_loop_handle_t eventLoop;

void dispatcher(void *handler_arg, esp_event_base_t base, int32_t id,
                void *event_data) {
  static uint32_t last_update = 0;
  Event::Body *evt = (Event::Body *)event_data;
  Context &context = Context::getInstance();
  switch (evt->type) {
  case Event::Type::BUTTON_CLICK: {
    ESP_LOGI(TAG, "BUTTON_CLICK");
    auto deviceState = context.getDeviceState();
    auto workType = context.getWorkType();
    switch (deviceState) {
    case State::COMPASS: {
      // 切换罗盘工作类型
      if (workType == WorkType::SPAWN) {
        // 每次切换到指南针模式,需要将数据源设置为传感器
        context.setSubscribeSource(Event::Source::SENSOR);
      }
      context.toggleWorkType();
      ESP_LOGI(TAG, "Current work type: %s",
               utils::workType2Str(context.getWorkType()).c_str());
      break;
    }
    default:
      break;
    }
  } break;
  case Event::Type::BUTTON_LONG_PRESS: {
    ESP_LOGI(TAG, "BUTTON_LONG_PRESS");
    auto deviceState = context.getDeviceState();
    auto workType = context.getWorkType();
    auto currentLoc = context.getCurrentLocation();
    switch (deviceState) {
    // COMPASS模式下响应长按事件
    case State::COMPASS: {
      // 出生针模式下， 长按设置新的出生点
      if (workType == WorkType::SPAWN) {
        // 检查GPS坐标是否有效
        if (gps::isValidGPSLocation(currentLoc)) {
          preference::saveSpawnLocation(currentLoc);
          context.setSpawnLocation(currentLoc);
          ESP_LOGI(TAG, "Set spawn location to {%.2f,%.2f}",
                   currentLoc.latitude, currentLoc.longitude);
        } else {
          ESP_LOGW(TAG, "Can't set spawn location, invalid GPS data.");
        }
      } else if (workType == WorkType::SOUTH) {
        // 指南针模式下， 长按切换数据源
        if (context.getSubscribeSource() == Event::Source::SENSOR) {
          context.setSubscribeSource(Event::Source::NETHER);
          ESP_LOGI(TAG, "Switch Data Source to NETHER");
        } else if (context.getSubscribeSource() == Event::Source::NETHER) {
          context.setSubscribeSource(Event::Source::SENSOR);
          ESP_LOGI(TAG, "Switch Data Source to SENSOR");
        }
      }
      break;
    }
    default:
      break;
    }
  } break;
  case Event::Type::FACTORY_RESET: {
    ESP_LOGI(TAG, "FACTORY_RESET");
    // 启动异步倒计时任务
    xTaskCreate(
        [](void *ctx) {
          ESP_LOGI(TAG, "cast context %p", ctx);
          auto context = static_cast<Context *>(ctx);
          auto deviceState = context->getDeviceState();
          ESP_LOGI(TAG, "deviceState=%d", deviceState);
          if (deviceState != State::COMPASS) {
            vTaskDelete(NULL);
            return;
          }
          context->setDeviceState(State::INFO);
          ESP_LOGW(TAG, "Factory Reset!!!");
          // 恢复出厂设置
          // 倒计时3秒
          pixel::counterDown(3);
          preference::factoryReset();
          // 重启
          esp_restart();
        },
        "factory_reset", 4096, &context, 1, NULL);
  } break;
  case Event::Type::AZIMUTH: {
    // ESP_LOGI(TAG, "evt->azimuth.angle=%d",
    // evt->azimuth.angle);
    // 状态校验, 非COMPASS状态忽略方位角数据
    if (context.getDeviceState() != State::COMPASS)
      return;
    // 校验刷新频率, 限制帧率30Hz
    if (millis() - last_update < 33)
      return;
    switch (context.getWorkType()) {
    case WorkType::SPAWN: {
      pixel::setPointerColor(context.getColor().spawnColor);
      // 当前位置无效, 显示来自Nether的方位角
      if (!gps::isValidGPSLocation(context.getCurrentLocation())) {
        ESP_LOGI(TAG, "GPS azimuth=%d evt->source=%d", evt->azimuth.angle,
                 evt->source);
        if (evt->source == Event::Source::NETHER) {
          pixel::showByAzimuth(evt->azimuth.angle);
        }
      } else {

        // 当前位置有效, 显示计算目标位置方位角
        pixel::showFrameByLocation(context.getSpawnLocation().latitude,
                                   context.getSpawnLocation().longitude,
                                   context.getCurrentLocation().latitude,
                                   context.getCurrentLocation().longitude,
                                   evt->azimuth.angle);
      }
      break;
    }
    case WorkType::SOUTH: {
      ESP_LOGI(TAG, "RAW  RX  Az:%d",azimuth);
      static int lastAzimuth = 0;
      // 指南针模式下忽略非订阅的源,否则会受到随机数据影响
      if (context.getSubscribeSource() != evt->source)
        return;
      context.setAzimuth(evt->azimuth.angle);
      pixel::setPointerColor(context.getColor().southColor);
      pixel::showByAzimuth(evt->azimuth.angle);
      // 减少日志打印
      if (lastAzimuth != evt->azimuth.angle) {
        ESP_LOGI(TAG, "SOUTH azimuth=%d evt->source=%d", evt->azimuth.angle,
                 evt->source);
        lastAzimuth = evt->azimuth.angle;
      }
      break;
    }
    default:
      break;
    }
    last_update = millis();
  } break;
  case Event::Type::TEXT: { // 状态校验, 非INFO状态,忽略TEXT
    if (context.getDeviceState() != State::INFO)
      return;
    ESP_LOGW(TAG, "TEXT %s", evt->TEXT.text);
    xTaskCreate(
        [](void *pvParams) {
          auto event = (Event::Body *)pvParams;
          Context &context = Context::getInstance();
          String text = String(event->TEXT.text);
          int x = 0;
          size_t length = text.length();
          int xLimit = -length * 4;
          while (context.getDeviceState() == State::INFO) {
            FastLED.clear();
            for (size_t i = 0; i < length; i++) {
              pixel::drawChar(text.charAt(i), x + i * 4, 0, CRGB::Green);
            }
            FastLED.show();
            delay(100);
            x--;
            if (x < xLimit) {
              x = 10;
            }
          }
          ESP_LOGI(TAG, "Exit Info State");
          vTaskDelete(NULL);
        },
        "info", 2048, evt, 1, NULL);
  } break;
  case Event::Type::SENSOR_CALIBRATE: {
    ESP_LOGI(TAG, "SENSOR_CALIBRATE");
    xTaskCreate(
        [](void *ctx) {
          auto context = static_cast<Context *>(ctx);
          auto deviceState = context->getDeviceState();
          ESP_LOGI(TAG, "deviceState=%d", deviceState);
          context->setDeviceState(State::INFO);
          Event::Body event;
          event.type = Event::Type::TEXT;
          event.source = Event::Source::OTHER;
          strcpy(event.TEXT.text, "Claibrate");
          // 倒计时3秒
          pixel::counterDown(3);
          ESP_ERROR_CHECK(esp_event_post_to(context->getEventLoop(),
                                            MCOMPASS_EVENT, 0, &event,
                                            sizeof(event), 0));
          ESP_LOGW(TAG, "Calibrate Start");
          sensor::calibrate();
          context->setDeviceState(State::COMPASS);
          ESP_LOGW(TAG, "Calibrate Done.");
          esp_restart();
        },
        "calibrate", 8192, &context, configMAX_PRIORITIES - 1, NULL);
  } break;
  default:
    break;
  }
}
void setup() {
  // 延时,用于一些特殊情况下能够重新烧录
  delay(1000);
  Context &context = Context::getInstance();
  esp_event_loop_args_t loop_args = {
      .queue_size = 128,
      .task_name = "event_loop",
      .task_priority = configMAX_PRIORITIES - 1,
      .task_stack_size = 2048,
  };
  ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &eventLoop));
  context.setEventLoop(eventLoop);
  // 注册事件处理程序
  ESP_ERROR_CHECK(esp_event_handler_register_with(eventLoop, MCOMPASS_EVENT, 0,
                                                  dispatcher, NULL));
  ESP_LOGI(TAG, "Event loop created %p", eventLoop);

  // 初始化硬件
  board::init();

  // 输出上下文内容
  ESP_LOGI(
      TAG,
      "Context {\n "
      "ServerMode:%d\n PointerColor{spawnColor:0x%x,southColor:0x%x}\n "
      "Brightness:%d\n "
      "SpawnLocation:{latitude:%.2f,longitude:%.2f}\n WiFi:%s\n Model:%s\n "
      "HasSensor:%d\n Sensor Model:%s\n "
      "detectGPS:%d\n "
      "}",
      context.getServerMode(), context.getColor().spawnColor,
      context.getColor().southColor, context.getBrightness(),
      context.getSpawnLocation().latitude, context.getSpawnLocation().longitude,
      context.getSsid(), context.getModel() == Model::GPS ? "GPS" : "LITE",
      context.getHasSensor(),
      utils::sensorModel2Str(context.getSensorModel()).c_str(),
      context.getDetectGPS());

  ESP_LOGW(TAG, "Board initialized");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(200)); }
