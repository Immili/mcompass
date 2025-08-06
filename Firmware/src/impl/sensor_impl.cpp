#include "board.h"
#include <Arduino.h>
#include <math.h>

#include "MagneticSensor.h"
#include "QMC5883LAdapter.h"
#include "QMC5883PAdapter.h"
#include "MMC5883MAAdapter.h"

using namespace mcompass;
static const char *TAG = "SENSOR";

static MagneticSensor *magneticSensor;
static SensorModel sm = SensorModel::UNKNOWN;

void sensor::init(Context *context) {
  // 初始化i2c
  Wire.begin();
  // 0x0D QMC5883L
  // 0x2C QMC5883P
  // 0x30 MMC5883MA
  // 遍历已知的地磁传感器地址,判断当前地磁传感器型号

  Wire.beginTransmission(0x0D);
  if (Wire.endTransmission() == 0) {
    ESP_LOGI(TAG, "Found QMC5883L at address 0x0D");
    sm = SensorModel::QMC5883L;
    magneticSensor = new QMC5883LAdapter();
  }
  Wire.beginTransmission(0x2C);
  if (Wire.endTransmission() == 0) {
    ESP_LOGI(TAG, "Found QMC5883P at address 0x2C");
    sm = SensorModel::QMC5883P;
    magneticSensor = new QMC5883PAdapter();
  }
  Wire.beginTransmission(0x30);
  if (Wire.endTransmission() == 0) {
    ESP_LOGI(TAG, "Found MMC5883MA at address 0x30");
    sm = SensorModel::MMC5883MA;
    magneticSensor = new MMC5883MAAdapter();
  }

  // 进行一次全量I2C扫描
  if (sm == SensorModel::UNKNOWN) {
    Serial.println(
        "Unknown magnetometer found. Performing full I2C bus scan...");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++) { // I2C地址范围通常是0x01到0x7F
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
        Serial.print("  I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("!");
      }
    }
    ESP_LOGE(TAG, "Sensor init failed");
    context->setHasSensor(false);
    context->setDeviceState(State::INFO);
    Event::Body event;
    event.type = Event::Type::TEXT;
    event.source = Event::Source::SENSOR;
    memcpy(event.TEXT.text, SENSOR_ERROR, sizeof(SENSOR_ERROR));
    ESP_ERROR_CHECK(esp_event_post_to(context->getEventLoop(), MCOMPASS_EVENT,
                                      0, &event, sizeof(event), 0));
    return;
  }
  magneticSensor->init();
  context->setSensorModel(sm);

  // 还原传感器的校准数据
  preference::CalibrationData data = preference::getCalibration();
  if (data.offsets[0] != 0 || data.offsets[1] != 0 || data.offsets[2] != 0 ||
      data.scales[0] != 0 || data.scales[1] != 0 || data.scales[2] != 0) {
    ESP_LOGW(TAG, "restore CalibrationOffsets(%f, %f,%f)", data.offsets[0],
             data.offsets[1], data.offsets[2]);
    ESP_LOGW(TAG, "restore CalibrationScales(%f, %f,%f)", data.scales[0],
             data.scales[1], data.scales[2]);
    magneticSensor->setCalibrationOffsets(data.offsets[0], data.offsets[1],
                                          data.offsets[2]);
    magneticSensor->setCalibrationScales(data.scales[0], data.scales[1],
                                         data.scales[2]);
  }
}

void sensor::calibrate() {
  if (nullptr == magneticSensor) {
    return;
  }
  magneticSensor->calibrate();
  ESP_LOGW(TAG, "setCalibrationOffsets(%f, %f,%f)",
           magneticSensor->getCalibrationOffset(0),
           magneticSensor->getCalibrationOffset(1),
           magneticSensor->getCalibrationOffset(2));
  ESP_LOGW(TAG, "setCalibrationScales(%f, %f,%f)",
           magneticSensor->getCalibrationScale(0),
           magneticSensor->getCalibrationScale(1),
           magneticSensor->getCalibrationScale(2));
  preference::CalibrationData data;
  data.offsets[0] = magneticSensor->getCalibrationOffset(0);
  data.offsets[1] = magneticSensor->getCalibrationOffset(1);
  data.offsets[2] = magneticSensor->getCalibrationOffset(2);
  data.scales[0] = magneticSensor->getCalibrationScale(0);
  data.scales[1] = magneticSensor->getCalibrationScale(1);
  data.scales[2] = magneticSensor->getCalibrationScale(2);
  preference::setCalibration(data);
  ESP_LOGW(TAG, "Calibration data saved to preferences");
}

/**
 * @brief 获取当前方位角
 */

int sensor::getAzimuth() {
  if (nullptr == magneticSensor) {
    return 0;
  }
  magneticSensor->read();
  int azimuth = magneticSensor->getAzimuth();
  switch (sm) {
  case SensorModel::QMC5883P: {
    // QMC5883P的方位角需要特殊处理,他的Y轴和QMC5883L的Y轴是反向的
    // 需要将Y轴的值取反, 并且坐标相对于QMC5883L需要旋转90度
    if (azimuth >= 360) {
      azimuth -= 360;
    }
    break;
  }
  case SensorModel::QMC5883L: {
    azimuth = 360 - azimuth; // 将方位角转换为0-360度范围
  }
    case SensorModel::MMC5883MA: {
    ESP_LOGI(TAG, "RAW  Az:%d",azimuth);
    azimuth = 180-azimuth ; // 将方位角转换为0-360度范围 
    if(azimuth<0){
      azimuth+=360;
    }
  }
  default:
    break;
  }
  return azimuth;
}

bool sensor::available() { return nullptr != magneticSensor; }
