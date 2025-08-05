// 文件: MMC5883MACompass.cpp
#include "MMC5883MACompass.h"
#include <Wire.h>
#include <Arduino.h>

// 16向罗盘方向字典
const char MMC5883MACompass::_bearings[16][3] = {
    {' ', ' ', 'N'}, {' ', 'N', 'N'}, {'N', 'N', 'E'}, {' ', 'N', 'E'},
    {'E', 'N', 'E'}, {' ', 'E', ' '}, {'E', 'S', 'E'}, {' ', 'S', 'E'},
    {'S', 'S', 'E'}, {' ', ' ', 'S'}, {'S', 'S', 'W'}, {' ', 'S', 'W'},
    {'W', 'S', 'W'}, {' ', ' ', 'W'}, {'W', 'N', 'W'}, {' ', 'N', 'W'}
};

MMC5883MACompass::MMC5883MACompass() {}

void MMC5883MACompass::init() {
    Wire.begin();
    _writeReg(0x09, 0x80);   // 软件复位
    delay(5);                // ≥5ms
    _writeReg(0x09, 0x02);   // 设置CR1: BW=400Hz
    _writeReg(0x08, 0x01);   // 设置CR0: 连续测量模式
    _performSet();           // SET脉冲
    _performReset();         // RESET脉冲
}

void MMC5883MACompass::setReset() {
    _writeReg(0x09, 0x80);
    delay(5);
}

void MMC5883MACompass::setADDR(byte addr) {
    _ADDR = addr;
}

void MMC5883MACompass::setMode(byte mode, byte odr) {
    mode &= 0x03;
    odr  &= 0x0C;
    _writeReg(0x09, odr);
    _writeReg(0x08, mode);
}

void MMC5883MACompass::setMagneticDeclination(int degrees, uint8_t minutes) {
    _magneticDeclinationDegrees = degrees + minutes / 60.0;
}

void MMC5883MACompass::setSmoothing(byte steps, bool adv) {
    _smoothUse = true;
    _smoothSteps = min(steps, (byte)10);
    _smoothAdvanced = adv;
}

void MMC5883MACompass::calibrate() {
  clearCalibration();
  long calibrationData[3][2] = {
      {65000, -65000}, {65000, -65000}, {65000, -65000}};
  
  // Prime the values
  read();
  long x = calibrationData[0][0] = calibrationData[0][1] = getX();
  long y = calibrationData[1][0] = calibrationData[1][1] = getY();
  long z = calibrationData[2][0] = calibrationData[2][1] = getZ();

  unsigned long startTime = millis();

  while ((millis() - startTime) < 10000) {
    read();

    x = getX();
    y = getY();
    z = getZ();

    if (x < calibrationData[0][0]) {
      calibrationData[0][0] = x;
    }
    if (x > calibrationData[0][1]) {
      calibrationData[0][1] = x;
    }

    if (y < calibrationData[1][0]) {
      calibrationData[1][0] = y;
    }
    if (y > calibrationData[1][1]) {
      calibrationData[1][1] = y;
    }

    if (z < calibrationData[2][0]) {
      calibrationData[2][0] = z;
    }
    if (z > calibrationData[2][1]) {
      calibrationData[2][1] = z;
    }
  }

  setCalibration(calibrationData[0][0], calibrationData[0][1],
                 calibrationData[1][0], calibrationData[1][1],
                 calibrationData[2][0], calibrationData[2][1]);
}

void MMC5883MACompass::setCalibration(int x_min, int x_max, int y_min, int y_max,
                                     int z_min, int z_max) {
  setCalibrationOffsets((x_min + x_max) / 2.0, (y_min + y_max) / 2.0,
                        (z_min + z_max) / 2.0);

  float x_avg_delta = (x_max - x_min) / 2.0;
  float y_avg_delta = (y_max - y_min) / 2.0;
  float z_avg_delta = (z_max - z_min) / 2.0;

  float avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3.0;

  setCalibrationScales(avg_delta / x_avg_delta, avg_delta / y_avg_delta,
                       avg_delta / z_avg_delta);
}

// (No change in logic)
void MMC5883MACompass::setCalibrationOffsets(float x_offset, float y_offset,
                                            float z_offset) {
  _offset[0] = x_offset;
  _offset[1] = y_offset;
  _offset[2] = z_offset;
}

// (No change in logic)
void MMC5883MACompass::setCalibrationScales(float x_scale, float y_scale,
                                           float z_scale) {
  _scale[0] = x_scale;
  _scale[1] = y_scale;
  _scale[2] = z_scale;
}

void MMC5883MACompass::clearCalibration() {
  setCalibrationOffsets(0., 0., 0.);
  setCalibrationScales(1., 1., 1.);
}

void MMC5883MACompass::read() {
    // 触发单次测量 TM_M=1
    _writeReg(0x08, 0x01);
    // 防止CR1丢失，重新写BW
    _writeReg(0x09, 0x02);
    delay(3);                // ≥2.5ms
    // 读取0x00-0x05寄存器
    Wire.beginTransmission(_ADDR);
    Wire.write((byte)0x00);
    if (Wire.endTransmission() == 0 && Wire.requestFrom(_ADDR, (byte)6) == 6) {
        _vRaw[0] = (int16_t)(Wire.read() | (Wire.read() << 8));
        _vRaw[1] = (int16_t)(Wire.read() | (Wire.read() << 8));
        _vRaw[2] = (int16_t)(Wire.read() | (Wire.read() << 8));
        _applyCalibration();
        if (_smoothUse) _smoothing();
    }
}

char MMC5883MACompass::chipID() {
    Wire.beginTransmission(_ADDR);
    Wire.write((byte)0x07);
    if (Wire.endTransmission() == 0) {
        Wire.requestFrom(_ADDR, (byte)1);
        return Wire.read();
    }
    return 0;
}

int MMC5883MACompass::getAzimuth() {
    float heading = atan2(_vCalibrated[0], -_vCalibrated[1]) * 180.0 / PI;
    heading += _magneticDeclinationDegrees;
    if (heading < 0) heading += 360;
    else if (heading >= 360) heading -= 360;
    heading = 360 - heading + 180;
    if (heading >= 360) heading -= 360;
    Serial.println("Heading: " + String(heading));
    return (int)heading;
}

byte MMC5883MACompass::getBearing(int azimuth) {
    return azimuth * 16 / 360;
}

void MMC5883MACompass::getDirection(char *buf, int azimuth) {
    byte b = getBearing(azimuth);
    memcpy(buf, _bearings[b], 3);
    buf[3] = '\0';
}

float MMC5883MACompass::getCalibrationOffset(byte idx) { return _offset[idx]; }
float MMC5883MACompass::getCalibrationScale(byte idx)  { return _scale[idx]; }
int   MMC5883MACompass::getX()  { return _vCalibrated[0]; }
int   MMC5883MACompass::getY()  { return _vCalibrated[1]; }
int   MMC5883MACompass::getZ()  { return _vCalibrated[2]; }

void MMC5883MACompass::_writeReg(byte r, byte v) {
    Wire.beginTransmission(_ADDR);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}

void MMC5883MACompass::_performSet() {
    _writeReg(0x08, 0x08);
    delay(1);
}

void MMC5883MACompass::_performReset() {
    _writeReg(0x08, 0x10);
    delay(1);
}

void MMC5883MACompass::_applyCalibration() {
    _vCalibrated[0] = (_vRaw[0] - (int)_offset[0]) * _scale[0];
    _vCalibrated[1] = (_vRaw[1] - (int)_offset[1]) * _scale[1];
    _vCalibrated[2] = (_vRaw[2] - (int)_offset[2]) * _scale[2];
}

void MMC5883MACompass::_smoothing() {
    if (_vScan >= _smoothSteps) _vScan = 0;
    for (int i = 0; i < 3; i++) _vHistory[_vScan][i] = _vCalibrated[i];
    _vScan++;
    memset(_vTotals, 0, sizeof(_vTotals));
    for (int j = 0; j < _smoothSteps; j++)
        for (int i = 0; i < 3; i++) _vTotals[i] += _vHistory[j][i];
    for (int i = 0; i < 3; i++) {
        byte mx = 0, mn = 0;
        for (byte j = 1; j < _smoothSteps; j++) {
            if (_vHistory[j][i] > _vHistory[mx][i]) mx = j;
            if (_vHistory[j][i] < _vHistory[mn][i]) mn = j;
        }
        if (_smoothAdvanced && _smoothSteps > 2)
            _vSmooth[i] = (_vTotals[i] - _vHistory[mx][i] - _vHistory[mn][i]) / (_smoothSteps - 2);
        else
            _vSmooth[i] = _vTotals[i] / _smoothSteps;
    }
}
