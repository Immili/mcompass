#include "Wire.h"
#include "MMC5883MACompass.h"

/*  为静态 constexpr 数组分配存储（必须有，否则链接报 undefined reference）  */
constexpr char MMC5883MACompass::_bearings[16][3];

/* ---------- 寄存器地址 (MMC5883MA) ---------- */
constexpr byte REG_DATA_X_L = 0x00;   // X_L, X_H, Y_L, Y_H, Z_L, Z_H
constexpr byte REG_STATUS   = 0x07;   // bit0 = Meas_Done
constexpr byte REG_CTRL0    = 0x08;   // TM_M / RESET / SET / BW
constexpr byte REG_CTRL1    = 0x09;   // 备用，保持默认
constexpr byte REG_CTRL2    = 0x0A;   // Soft‑reset
constexpr byte REG_PRODUCT  = 0x2F;   // 固定 0x0C

/* ---------- 位掩码 ---------- */
constexpr byte CTRL0_TM_M   = 0x01;
constexpr byte CTRL0_SET    = 0x08;
constexpr byte CTRL0_RESET  = 0x10;

/* ======================================================================
 *  类实现
 * ====================================================================*/
MMC5883MACompass::MMC5883MACompass() {}

/* ---------- 初始化 ---------- */
void MMC5883MACompass::init() {
    Wire.begin();

    /* Soft‑reset */
    _writeReg(REG_CTRL2, 0x80);
    delay(2);

    /* 先做一次 SET / RESET，保证磁芯退磁 */
    _writeReg(REG_CTRL0, CTRL0_SET);
    delayMicroseconds(50);
    _writeReg(REG_CTRL0, CTRL0_RESET);
    delayMicroseconds(50);

    clearCalibration();
}

/* 修改 I²C 地址（如有必要） */
void MMC5883MACompass::setADDR(byte b) { _ADDR = b; }

/* 读产品 ID（MMC5883MA 固定 0x0C） */
char MMC5883MACompass::chipID() {
    Wire.beginTransmission(_ADDR);
    Wire.write(REG_PRODUCT);
    if (Wire.endTransmission(false)) return 0;
    Wire.requestFrom(_ADDR, (byte)1);
    return Wire.read();
}

/* ---------- 单次测量并读取数据 ---------- */
void MMC5883MACompass::read() {
    /* 1. 触发单次测量 */
    _writeReg(REG_CTRL0, CTRL0_TM_M);

    /* 2. 等待测量完成 (STATUS.bit0 == 1) */
    uint8_t status;
    do {
        Wire.beginTransmission(_ADDR);
        Wire.write(REG_STATUS);
        Wire.endTransmission(false);
        Wire.requestFrom(_ADDR, (byte)1);
        status = Wire.read();
    } while (!(status & 0x01));

    /* 3. 读取 6 字节 XYZ 原始数据 */
    Wire.beginTransmission(_ADDR);
    Wire.write(REG_DATA_X_L);
    Wire.endTransmission(false);
    Wire.requestFrom(_ADDR, (byte)6);

    for (int i = 0; i < 3; ++i) {
        uint16_t lo = Wire.read();
        uint16_t hi = Wire.read();
        uint16_t raw = (hi << 8) | lo;   // 0‑65535
        _vRaw[i] = int(raw) - 32768;     // 转 ±32768（去掉 32768 偏移）
    }

    _applyCalibration();

    if (_useSmooth) _smoothing();
}

/* ---------- 校准与偏角 ---------- */
void MMC5883MACompass::setMagneticDeclination(int deg, uint8_t minutes) {
    _magDeclDeg = deg + minutes / 60.0f;
}

void MMC5883MACompass::setSmoothing(byte steps, bool adv) {
    _useSmooth = true;
    _smoothN   = (steps < 2) ? 2 : (steps > 10 ? 10 : steps);
    _smoothAdv = adv;
}

void MMC5883MACompass::calibrate() {
    clearCalibration();
    long minv[3] = { 65000, 65000, 65000 };
    long maxv[3] = {-65000,-65000,-65000};

    unsigned long t0 = millis();
    while (millis() - t0 < 10000) {  // 旋转 10 秒
        read();
        for (int i = 0; i < 3; ++i) {
            if (_vRaw[i] < minv[i]) minv[i] = _vRaw[i];
            if (_vRaw[i] > maxv[i]) maxv[i] = _vRaw[i];
        }
    }
    setCalibration(minv[0], maxv[0],
                   minv[1], maxv[1],
                   minv[2], maxv[2]);
}

void MMC5883MACompass::setCalibration(int xmin,int xmax,int ymin,int ymax,int zmin,int zmax){
    setCalibrationOffsets( (xmin + xmax) / 2.0f,
                           (ymin + ymax) / 2.0f,
                           (zmin + zmax) / 2.0f );
    float dx = (xmax - xmin) / 2.0f;
    float dy = (ymax - ymin) / 2.0f;
    float dz = (zmax - zmin) / 2.0f;
    float avg = (dx + dy + dz) / 3.0f;
    setCalibrationScales( avg / dx, avg / dy, avg / dz );
}

void MMC5883MACompass::setCalibrationOffsets(float x,float y,float z){
    _offset[0]=x; _offset[1]=y; _offset[2]=z;
}
void MMC5883MACompass::setCalibrationScales (float xs,float ys,float zs){
    _scale[0]=xs; _scale[1]=ys; _scale[2]=zs;
}

float MMC5883MACompass::getCalibrationOffset(uint8_t i){ return _offset[i]; }
float MMC5883MACompass::getCalibrationScale (uint8_t i){ return _scale [i]; }
void  MMC5883MACompass::clearCalibration(){
    setCalibrationOffsets(0,0,0);
    setCalibrationScales (1,1,1);
}

/* ---------- 平滑处理 ---------- */
void MMC5883MACompass::_smoothing(){
    if (_vScan >= _smoothN) _vScan = 0;
    for(int i=0;i<3;++i){
        _vTotals[i] -= _vHist[_vScan][i];
        _vHist  [_vScan][i] = _vCal[i];
        _vTotals[i] += _vHist[_vScan][i];

        if (_smoothAdv) {
            int min = _vHist[0][i], max = min;
            for (int j=1;j<_smoothN;++j){
                min = _vHist[j][i] < min ? _vHist[j][i] : min;
                max = _vHist[j][i] > max ? _vHist[j][i] : max;
            }
            _vSmooth[i] = (_vTotals[i] - min - max) / (_smoothN - 2);
        } else {
            _vSmooth[i] = _vTotals[i] / _smoothN;
        }
    }
    ++_vScan;
}

/* ---------- 校准应用 ---------- */
void MMC5883MACompass::_applyCalibration(){
    for(int i=0;i<3;++i){
        _vCal[i] = (_vRaw[i] - _offset[i]) * _scale[i];
    }
}

/* ---------- 数据接口 ---------- */
int MMC5883MACompass::_get(int i){
    return _useSmooth ? _vSmooth[i] : _vCal[i];
}
int MMC5883MACompass::getX(){ return _get(0); }
int MMC5883MACompass::getY(){ return _get(1); }
int MMC5883MACompass::getZ(){ return _get(2); }

int MMC5883MACompass::getAzimuth(){
    float ang = atan2(getY(), getX()) * 180.0f / PI + _magDeclDeg;
    if (ang < 0)  ang += 360;
    return int(ang + 0.5f) % 360;
}

byte MMC5883MACompass::getBearing(int az){
    int idx = int((az < 0 ? az + 360 : az) / 22.5f + 0.5f) & 0x0F;
    return idx;
}

void MMC5883MACompass::getDirection(char *buf, int az){
    byte idx = getBearing(az);
    buf[0] = _bearings[idx][0];
    buf[1] = _bearings[idx][1];
    buf[2] = _bearings[idx][2];
}

/* ---------- 私有寄存器写助手 ---------- */
void MMC5883MACompass
