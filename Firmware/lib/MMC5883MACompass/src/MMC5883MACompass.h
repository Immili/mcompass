#ifndef MMC5883MA_COMPASS
#define MMC5883MA_COMPASS

#include "Arduino.h"
#include "Wire.h"

/*
 * 兼容层：类名现改为 MMC5883MACompass
 * 内部针对 MMC5883MA（I²C 0x30、±8 G、0.25 mG/LSB）
 */
class MMC5883MACompass {
public:
    MMC5883MACompass();

    /* ---------- 初始化与配置 ---------- */
    void init();                          // I²C begin + 软件复位
    void setADDR(byte b);                 // 如需修改默认地址 0x30
    void setMagneticDeclination(int deg, uint8_t minutes = 0);

    /* ---------- 数据读取 ---------- */
    void read();                          // 触发一次测量并更新内部缓存
    int  getX();
    int  getY();
    int  getZ();
    int  getAzimuth();                    // 0‑359°，已加磁偏角
    byte getBearing(int azimuth);
    void getDirection(char *buf, int azimuth);

    /* ---------- 校准与滤波 ---------- */
    void calibrate();                     // 旋转 10 s 自动采集极值
    void setCalibration(int xmin,int xmax,int ymin,int ymax,int zmin,int zmax);
    void setCalibrationOffsets(float x_off,float y_off,float z_off);
    void setCalibrationScales(float x_s,float y_s,float z_s);
    float getCalibrationOffset(uint8_t idx);
    float getCalibrationScale(uint8_t idx);
    void  clearCalibration();
    void  setSmoothing(byte steps = 5, bool advanced = false);

    /* ---------- 调试 ---------- */
    char chipID();                        // 读 0x2F，应为 0x0C

private:
    /* 寄存器写助手 */
    void _writeReg(byte reg, byte val);

    /* 内部工具 */
    int  _get(int idx);
    void _smoothing();
    void _applyCalibration();

    /* 常量 & 状态 */
    static constexpr byte DEFAULT_ADDR = 0x30;   // MMC5883MA 7‑bit 地址
    byte _ADDR = DEFAULT_ADDR;

    /* 校准 & 滤波 */
    float _magDeclDeg = 0;
    bool  _useSmooth  = false;
    byte  _smoothN    = 5;
    bool  _smoothAdv  = false;
    float _offset[3]  = {0,0,0};
    float _scale [3]  = {1,1,1};

    /* 数据缓冲 */
    int   _vRaw [3]   = {0,0,0};
    int   _vCal [3]   = {0,0,0};
    int   _vSmooth[3] = {0,0,0};
    long  _vTotals[3] = {0,0,0};
    int   _vHist [10][3] {};
    byte  _vScan = 0;

    /* 16 方位字符表 */
    static constexpr char _bearings[16][3] = {
        {' ',' ','N'}, {'N','N','E'}, {' ','N','E'}, {'E','N','E'},
        {' ',' ','E'}, {'E','S','E'}, {' ','S','E'}, {'S','S','E'},
        {' ',' ','S'}, {'S','S','W'}, {' ','S','W'}, {'W','S','W'},
        {' ',' ','W'}, {'W','N','W'}, {' ','N','W'}, {'N','N','W'}
    };
};

#endif // MMC5883MA_COMPASS
