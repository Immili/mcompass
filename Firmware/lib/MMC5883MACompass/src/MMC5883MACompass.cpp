#include "MMC5883MACompass.h"
#include <Wire.h>
#include <math.h>
#include "esp_log.h"          // 为 ESP_LOGx 宏引入头文件

/* 为静态 constexpr 数组分配存储 */
constexpr char MMC5883MACompass::_bearings[16][3];

/* 寄存器地址 */
constexpr byte REG_DATA_X_L = 0x00; // X_L, X_H, Y_L, Y_H, Z_L, Z_H
constexpr byte REG_STATUS   = 0x07; // bit0 = Meas_Done
constexpr byte REG_CTRL0    = 0x08; // TM_M / RESET / SET / BW
constexpr byte REG_CTRL1    = 0x09; // BW
constexpr byte REG_CTRL2    = 0x0A; // Soft‑reset
constexpr byte REG_PRODUCT  = 0x2F; // 0x0C

/* 位掩码 */
constexpr byte CTRL0_TM_M  = 0x01;
constexpr byte CTRL0_SET   = 0x08;
constexpr byte CTRL0_RESET = 0x10;

/******************** 构造与初始化 ************************/ 
static const char *LOG_TAG = "MMC5883MA";

MMC5883MACompass::MMC5883MACompass(){}

void MMC5883MACompass::init(){
    Wire.begin();
    _writeReg(REG_CTRL2,0x80); // soft reset
    delay(3);
    _writeReg(REG_CTRL0,CTRL0_SET); delayMicroseconds(50);
    _writeReg(REG_CTRL0,CTRL0_RESET); delayMicroseconds(50);
    clearCalibration();
}

void MMC5883MACompass::setADDR(byte a){_ADDR=a;}

char MMC5883MACompass::chipID(){
    Wire.beginTransmission(_ADDR);
    Wire.write(REG_PRODUCT);
    if(Wire.endTransmission(false)) return 0;
    Wire.requestFrom(_ADDR,(byte)1);
    return Wire.read();
}

void MMC5883MACompass::setMode(byte mode, byte odr){
    _writeReg(REG_CTRL1, odr & 0x0C);      // 带宽位
    _writeReg(REG_CTRL0, mode & 0x03);     // 工作模式位
}

void MMC5883MACompass::setReset(){
    _writeReg(REG_CTRL2,0x80);
    delay(3);
}

/******************** 读取一次数据 ************************/
void MMC5883MACompass::read(){
    // 单次测量
    _writeReg(REG_CTRL0, CTRL0_TM_M);
    // 等待完成
    while(true){
        Wire.beginTransmission(_ADDR);
        Wire.write(REG_STATUS);
        Wire.endTransmission(false);
        Wire.requestFrom(_ADDR,(byte)1);
        if(Wire.read() & 0x01) break;
    }
    // 读 6 字节
    Wire.beginTransmission(_ADDR);
    Wire.write(REG_DATA_X_L);
    Wire.endTransmission(false);
    Wire.requestFrom(_ADDR,(byte)6);
    for(int i=0;i<3;++i){
        uint16_t lo=Wire.read(),hi=Wire.read();
        _vRaw[i]=int((hi<<8)|lo)-32768;
    }
    _applyCalibration();
    if(_useSmooth) _smoothing();
}

/******************** 校准/偏角/滤波 **********************/
void MMC5883MACompass::setMagneticDeclination(int d,uint8_t m){
    _magDeclDeg=d+m/60.0f;
}

void MMC5883MACompass::setSmoothing(byte n,bool adv){
    _useSmooth=true;
    _smoothN=n<2?2:(n>10?10:n);
    _smoothAdv=adv;
}

void MMC5883MACompass::calibrate(){
    clearCalibration();
    long minv[3]={ 65000,65000,65000};
    long maxv[3]={-65000,-65000,-65000};
    unsigned long t0=millis();
    while(millis()-t0<10000){
        read();
        for(int i=0;i<3;++i){
            if(_vRaw[i]<minv[i]) minv[i]=_vRaw[i];
            if(_vRaw[i]>maxv[i]) maxv[i]=_vRaw[i];
        }
        delay(50);
    }
    setCalibration(minv[0],maxv[0],minv[1],maxv[1],minv[2],maxv[2]);
}

void MMC5883MACompass::setCalibration(int xmin,int xmax,int ymin,int ymax,int zmin,int zmax){
    setCalibrationOffsets((xmin+xmax)/2.0f,(ymin+ymax)/2.0f,(zmin+zmax)/2.0f);
    float dx=(xmax-xmin)/2.0f, dy=(ymax-ymin)/2.0f, dz=(zmax-zmin)/2.0f;
    float avg=(dx+dy+dz)/3.0f;
    setCalibrationScales(avg/dx,avg/dy,avg/dz);
}
void MMC5883MACompass::setCalibrationOffsets(float x,float y,float z){_offset[0]=x;_offset[1]=y;_offset[2]=z;}
void MMC5883MACompass::setCalibrationScales (float xs,float ys,float zs){_scale[0]=xs;_scale[1]=ys;_scale[2]=zs;}
float MMC5883MACompass::getCalibrationOffset(uint8_t i){return _offset[i];}
float MMC5883MACompass::getCalibrationScale (uint8_t i){return _scale[i];}
void  MMC5883MACompass::clearCalibration(){setCalibrationOffsets(0,0,0);setCalibrationScales(1,1,1);} 

/******************** 平滑处理 ***************************/
void MMC5883MACompass::_smoothing(){
    if(_vScan>=_smoothN) _vScan=0;
    for(int i=0;i<3;++i){
        _vTotals[i]-=_vHist[_vScan][i];
        _vHist[_vScan][i]=_vCal[i];
        _vTotals[i]+=_vHist[_vScan][i];
        if(_smoothAdv){
            int min=_vHist[0][i],max=min;
            for(int j=1;j<_smoothN;++j){
                min=_vHist[j][i]<min?_vHist[j][i]:min;
                max=_vHist[j][i]>max?_vHist[j][i]:max;
            }
            _vSmooth[i]=(_vTotals[i]-min-max)/(_smoothN-2);
        }else{
            _vSmooth[i]=_vTotals[i]/_smoothN;
        }
    }
    ++_vScan;
}

/******************** 校准应用 ***************************/
void MMC5883MACompass::_applyCalibration(){
    for(int i=0;i<3;++i) _vCal[i]=(_vRaw[i]-_offset[i])*_scale[i];
}

/******************** 输出接口 ***************************/
int MMC5883MACompass::_get(int i){return _useSmooth?_vSmooth[i]:_vCal[i];}
int MMC5883MACompass::getX(){return _get(0);} 
int MMC5883MACompass::getY(){return _get(1);} 
int MMC5883MACompass::getZ(){return _get(2);} 



byte MMC5883MACompass::getBearing(int az){return byte(((az+11)/22)%16);} 

void MMC5883MACompass::getDirection(char*buf,int az){
    byte idx=getBearing(az);
    buf[0]=_bearings[idx][0];
    buf[1]=_bearings[idx][1];
    buf[2]=_bearings[idx][2];
}

/******************** I2C 写 *****************************/
void MMC5883MACompass::_writeReg(byte reg,byte val){
    Wire.beginTransmission(_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}
