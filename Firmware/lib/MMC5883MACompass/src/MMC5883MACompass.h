// 文件: MMC5883MACompass.h
#ifndef MMC5883MA_COMPASS_H
#define MMC5883MA_COMPASS_H

#include "Arduino.h"
#include <Wire.h>

class MMC5883MACompass {
public:
    MMC5883MACompass();
    /** 初始化: 软件复位, 设置400Hz ODR, 连续测量模式, 执行SET/RESET */
    void init();
    /** 软件复位 */
    void setReset();
    /** 修改I2C地址 */
    void setADDR(byte addr);
    /** 设置模式和输出数据率 */
    void setMode(byte mode, byte odr);
    /** 设置磁偏角 */
    void setMagneticDeclination(int degrees, uint8_t minutes);
    /** 设置数据平滑 */
    void setSmoothing(byte steps, bool adv);
    /** 用户校准入口 */
    void calibrate();
    /** 直接设置校准范围 */
    void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
    /** 读取最新数据并更新 _vCalibrated */
    void read();
    /** 读取芯片ID */
    char chipID();
    /** 获取方位角（0-359） */
    int getAzimuth();
    /** 获取16向罗盘索引 */
    byte getBearing(int azimuth);
    /** 获取方向字符串 */
    void getDirection(char *buf, int azimuth);
    /** 获取校准偏移 */
    float getCalibrationOffset(byte idx);
    /** 获取校准比例 */
    float getCalibrationScale(byte idx);
    /** 获取X/Y/Z轴当前值 */
    int getX();
    int getY();
    int getZ();

private:
    byte _ADDR = 0x30;
    float _magneticDeclinationDegrees = 0;
    bool _smoothUse = false;
    byte _smoothSteps = 5;
    bool _smoothAdvanced = false;
    int _vRaw[3] = {0,0,0};
    int _vHistory[10][3];
    int _vScan = 0;
    long _vTotals[3] = {0,0,0};
    int _vSmooth[3] = {0,0,0};
    float _offset[3] = {0.0,0.0,0.0};
    float _scale[3]  = {1.0,1.0,1.0};
    int _vCalibrated[3] = {0,0,0};
    static const char _bearings[16][3];
    void _writeReg(byte reg, byte val);
    void _performSet();
    void _performReset();
    void _applyCalibration();
    void _smoothing();
};

#endif // MMC5883MA_COMPASS_H
