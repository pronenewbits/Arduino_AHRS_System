/* Modified from  https://github.com/bolderflight/MPU9250/tree/master/src @2019-11-28 */

#ifndef SIMPLE_MPU9250_h
#define SIMPLE_MPU9250_h

#include "Arduino.h"
#include "Wire.h"       /* For I2C */

class SimpleMPU9250
{
public:
    enum GyroRange
    {
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G    
    };
    enum DlpfBandwidth
    {
      DLPF_BANDWIDTH_184HZ,
      DLPF_BANDWIDTH_92HZ,
      DLPF_BANDWIDTH_41HZ,
      DLPF_BANDWIDTH_20HZ,
      DLPF_BANDWIDTH_10HZ,
      DLPF_BANDWIDTH_5HZ
    };
    enum LpAccelOdr
    {
      LP_ACCEL_ODR_0_24HZ = 0,
      LP_ACCEL_ODR_0_49HZ = 1,
      LP_ACCEL_ODR_0_98HZ = 2,
      LP_ACCEL_ODR_1_95HZ = 3,
      LP_ACCEL_ODR_3_91HZ = 4,
      LP_ACCEL_ODR_7_81HZ = 5,
      LP_ACCEL_ODR_15_63HZ = 6,
      LP_ACCEL_ODR_31_25HZ = 7,
      LP_ACCEL_ODR_62_50HZ = 8,
      LP_ACCEL_ODR_125HZ = 9,
      LP_ACCEL_ODR_250HZ = 10,
      LP_ACCEL_ODR_500HZ = 11
    };
    
    SimpleMPU9250(TwoWire &_i2c, const uint8_t _address) {
        this->_i2c    = &_i2c;
        this->_address = _address;
    }
    
    int8_t begin(void) {
        _i2c->begin();
        _i2c->setClock(400000);          /* 400 kHz _i2c bus */
        
        /* select clock source to gyro */
        if (writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0) {
            return -1;
        }
        // enable _i2c master mode
        if(writeRegister(USER_CTRL,I2C_MST_EN) < 0) {
            return -2;
        }
        // set the _i2c bus speed to 400 kHz
        if(writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0) {
            return -3;
        }
        // set AK8963 to Power Down
        writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
        // reset the MPU9250
        writeRegister(PWR_MGMNT_1,PWR_RESET);
        // wait for MPU-9250 to come back up
        delay(1);
        // reset the AK8963
        writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
        // select clock source to gyro
        if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0) {
            return -4;
        }
        // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
        if((whoAmI() != 113)&&(whoAmI() != 115)) {
            return -5;
        }
        // enable accelerometer and gyro
        if(writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0) {
        return -6;
            }
        // setting accel range to 16G as default
        if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0) {
            return -7;
        }
        _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
        _accelRange = ACCEL_RANGE_16G;
        // setting the gyro range to 2000DPS as default
        if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0) {
            return -8;
        }
        _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
        _gyroRange = GYRO_RANGE_2000DPS;
        // setting bandwidth to 184Hz as default
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0) { 
            return -9;
        } 
        if(writeRegister(CONFIG,GYRO_DLPF_184) < 0) { // setting gyro bandwidth to 184Hz
            return -10;
        }
        _bandwidth = DLPF_BANDWIDTH_184HZ;
        // setting the sample rate divider to 0 as default
        if(writeRegister(SMPDIV,0x00) < 0) { 
            return -11;
        } 
        _srd = 0;
        // enable _i2c master mode
        if(writeRegister(USER_CTRL,I2C_MST_EN) < 0) {
            return -12;
        }
        // set the _i2c bus speed to 400 kHz
        if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0) {
            return -13;
        }
        // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
        if( whoAmIAK8963() != 72 ) {
            return -14;
        }
        /* get the magnetometer calibration */
        // set AK8963 to Power Down
        if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0) {
            return -15;
        }
        delay(100); // long wait between AK8963 mode changes
        // set AK8963 to FUSE ROM access
        if(writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0) {
            return -16;
        }
        delay(100); // long wait between AK8963 mode changes
        // read the AK8963 ASA registers and compute magnetometer scale factors
        readAK8963Registers(AK8963_ASA,3,_buffer);
        _magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
        _magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
        _magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 
        // set AK8963 to Power Down
        if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0) {
            return -17;
        }
        delay(100); // long wait between AK8963 mode changes  
        // set AK8963 to 16 bit resolution, 100 Hz update rate
        if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0) {
            return -18;
        }
        delay(100); // long wait between AK8963 mode changes
        // select clock source to gyro
        if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0) {
            return -19;
        }       
        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL,7,_buffer);
        // estimate gyro bias
        if (calibrateGyro() < 0) {
            return -20;
        }
        
        
        
            
        setAccelRange(ACCEL_RANGE_2G);
        setGyroRange(GYRO_RANGE_2000DPS);
        setDlpfBandwidth(DLPF_BANDWIDTH_184HZ);
        setSrd(0);
        // successful init, return 1
        return 1;
    }
    
    int8_t readSensor(void) {
        // grab the data from the MPU9250
        if (readRegisters(ACCEL_OUT, 21, _buffer) < 0) {
            return -1;
        }
        
        _axcounts = ((((int16_t)_buffer[0]) << 8) | _buffer[1]);  
        _aycounts = ((((int16_t)_buffer[2]) << 8) | _buffer[3]);
        _azcounts = ((((int16_t)_buffer[4]) << 8) | _buffer[5]);
        _gxcounts = ((((int16_t)_buffer[8]) << 8) | _buffer[9]);
        _gycounts = ((((int16_t)_buffer[10]) << 8) | _buffer[11]);
        _gzcounts = ((((int16_t)_buffer[12]) << 8) | _buffer[13]);
        _hxcounts = ((((int16_t)_buffer[15]) << 8) | _buffer[14]);
        _hycounts = ((((int16_t)_buffer[17]) << 8) | _buffer[16]);
        _hzcounts = ((((int16_t)_buffer[19]) << 8) | _buffer[18]);
        
        _ax = (float)(_axcounts) * _accelScale;
        _ay = (float)(_aycounts) * _accelScale;
        _az = (float)(_azcounts) * _accelScale;
        _gx = ((float)(_gxcounts) * _gyroScale) - _gxb;
        _gy = ((float)(_gycounts) * _gyroScale) - _gyb;
        _gz = ((float)(_gzcounts) * _gyroScale) - _gzb;
        
        /* Transform the magnetomer to align with the Accelerometer and Gyroscope Axis */
        float _hxAK8963 = ((float)_hxcounts) * _magScaleX;
        float _hyAK8963 = ((float)_hycounts) * _magScaleY;
        float _hzAK8963 = ((float)_hzcounts) * _magScaleZ;
        _hx =  _hyAK8963;
        _hy =  _hxAK8963;
        _hz = -_hzAK8963;
        
        return 1;
    }
    
    /* estimates the gyro biases */
    int calibrateGyro() {
        // set the range, bandwidth, and srd
        if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
            return -1;
        }
        if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
            return -2;
        }
        if (setSrd(19) < 0) {
            return -3;
        }

        // take samples and find bias
        _gxbD = 0;
        _gybD = 0;
        _gzbD = 0;
        for (size_t i=0; i < _numSamples; i++) {
            readSensor();
            _gxbD += (getGyroX_rads() + _gxb)/((double)_numSamples);
            _gybD += (getGyroY_rads() + _gyb)/((double)_numSamples);
            _gzbD += (getGyroZ_rads() + _gzb)/((double)_numSamples);
            delay(20);
        }
        _gxb = (float)_gxbD;
        _gyb = (float)_gybD;
        _gzb = (float)_gzbD;

        // set the range, bandwidth, and srd back to what they were
        if (setGyroRange(_gyroRange) < 0) {
            return -4;
        }
        if (setDlpfBandwidth(_bandwidth) < 0) {
            return -5;
        }
        if (setSrd(_srd) < 0) {
            return -6;
        }
        return 1;
    }
    
    /* sets the accelerometer full scale range to values other than default */
    int setAccelRange(AccelRange range) {
        switch(range) {
            case ACCEL_RANGE_2G: {
                // setting the accel range to 2G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0) {
                return -1;
                }
                _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
                break; 
            }
            case ACCEL_RANGE_4G: {
                // setting the accel range to 4G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0) {
                    return -1;
                }
                _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
                break;
            }
            case ACCEL_RANGE_8G: {
                // setting the accel range to 8G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0) {
                    return -1;
                }
                _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
                break;
            }
            case ACCEL_RANGE_16G: {
                // setting the accel range to 16G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0) {
                    return -1;
                }
                _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
                break;
            }
        }
        _accelRange = range;
        return 1;
    }

    /* sets the gyro full scale range to values other than default */
    int setGyroRange(GyroRange range) {
        switch(range) {
            case GYRO_RANGE_250DPS: {
                // setting the gyro range to 250DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0) {
                    return -1;
                }
                _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
                break;
            }
            case GYRO_RANGE_500DPS: {
                // setting the gyro range to 500DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0) {
                    return -1;
                }
                _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
                break;  
            }
            case GYRO_RANGE_1000DPS: {
                // setting the gyro range to 1000DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0) {
                    return -1;
                }
                _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
                break;
            }
            case GYRO_RANGE_2000DPS: {
                // setting the gyro range to 2000DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0) {
                    return -1;
                }
                _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
                break;
            }
        }
        _gyroRange = range;
        return 1;
    }

    /* sets the DLPF bandwidth to values other than default */
    int setDlpfBandwidth(DlpfBandwidth bandwidth) {
        switch(bandwidth) {
            case DLPF_BANDWIDTH_184HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0) { // setting accel bandwidth to 184Hz
                    return -1;
                } 
                if(writeRegister(CONFIG,GYRO_DLPF_184) < 0) { // setting gyro bandwidth to 184Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_92HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) < 0) { // setting accel bandwidth to 92Hz
                    return -1;
                } 
                if(writeRegister(CONFIG,GYRO_DLPF_92) < 0) { // setting gyro bandwidth to 92Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_41HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) < 0) { // setting accel bandwidth to 41Hz
                    return -1;
                } 
                if(writeRegister(CONFIG,GYRO_DLPF_41) < 0) { // setting gyro bandwidth to 41Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_20HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) < 0) { // setting accel bandwidth to 20Hz
                    return -1;
                } 
                if(writeRegister(CONFIG,GYRO_DLPF_20) < 0) { // setting gyro bandwidth to 20Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_10HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) < 0) { // setting accel bandwidth to 10Hz
                    return -1;
                } 
                if(writeRegister(CONFIG,GYRO_DLPF_10) < 0) { // setting gyro bandwidth to 10Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_5HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) < 0) { // setting accel bandwidth to 5Hz
                    return -1;
                } 
                if(writeRegister(CONFIG,GYRO_DLPF_5) < 0) { // setting gyro bandwidth to 5Hz
                    return -2;
                }
                break;
            }
        }
        _bandwidth = bandwidth;
        return 1;
    }

    /* sets the sample rate divider to values other than default */
    int setSrd(uint8_t srd) {
        /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
        if(writeRegister(SMPDIV,19) < 0) { // setting the sample rate divider
            return -1;
        }
        if(srd > 9) {
            // set AK8963 to Power Down
            if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0) {
                return -2;
            }
            delay(100); // long wait between AK8963 mode changes  
            // set AK8963 to 16 bit resolution, 8 Hz update rate
            if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0) {
                return -3;
            }
            delay(100); // long wait between AK8963 mode changes     
            // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
            readAK8963Registers(AK8963_HXL,7,_buffer);
        } else {
            // set AK8963 to Power Down
            if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0) {
                return -2;
            }
            delay(100); // long wait between AK8963 mode changes  
            // set AK8963 to 16 bit resolution, 100 Hz update rate
            if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0) {
                return -3;
            }
            delay(100); // long wait between AK8963 mode changes     
            // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
            readAK8963Registers(AK8963_HXL,7,_buffer);    
        } 
        /* setting the sample rate divider */
        if(writeRegister(SMPDIV,srd) < 0) { // setting the sample rate divider
            return -4;
        } 
        _srd = srd;
        return 1; 
    }

    /* returns the accelerometer measurement in the x direction, m/s/s */
    float getAccelX_mss() {
        return _ax;
    }

    /* returns the accelerometer measurement in the y direction, m/s/s */
    float getAccelY_mss() {
        return _ay;
    }

    /* returns the accelerometer measurement in the z direction, m/s/s */
    float getAccelZ_mss() {
        return _az;
    }

    /* returns the gyroscope measurement in the x direction, rad/s */
    float getGyroX_rads() {
        return _gx;
    }

    /* returns the gyroscope measurement in the y direction, rad/s */
    float getGyroY_rads() {
        return _gy;
    }

    /* returns the gyroscope measurement in the z direction, rad/s */
    float getGyroZ_rads() {
        return _gz;
    }

    /* returns the magnetometer measurement in the x direction, uT */
    float getMagX_uT() {
        return _hx;
    }

    /* returns the magnetometer measurement in the y direction, uT */
    float getMagY_uT() {
        return _hy;
    }

    /* returns the magnetometer measurement in the z direction, uT */
    float getMagZ_uT() {
        return _hz;
    }
    
    
private:
    uint8_t _address;
    TwoWire *_i2c;
    size_t _numBytes; // number of bytes received from I2C
    
    
    // track success of interacting with sensor
    int _status;
    // buffer for reading from sensor
    uint8_t _buffer[21];
    // data counts
    int16_t _axcounts,_aycounts,_azcounts;
    int16_t _gxcounts,_gycounts,_gzcounts;
    int16_t _hxcounts,_hycounts,_hzcounts;
    int16_t _tcounts;
    // data buffer
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _hx, _hy, _hz;
    float _t;
    // wake on motion
    uint8_t _womThreshold;
    // scale factors
    float _accelScale;
    float _gyroScale;
    float _magScaleX, _magScaleY, _magScaleZ;
    const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    DlpfBandwidth _bandwidth;
    uint8_t _srd;
    // gyro bias estimation
    size_t _numSamples = 100;
    double _gxbD, _gybD, _gzbD;
    float _gxb, _gyb, _gzb;
    // accel bias and scale factor estimation
    double _axbD, _aybD, _azbD;
    float _axmax, _aymax, _azmax;
    float _axmin, _aymin, _azmin;
    float _axb, _ayb, _azb;
    float _axs = 1.0f;
    float _ays = 1.0f;
    float _azs = 1.0f;
    // magnetometer bias and scale factor estimation
    uint16_t _maxCounts = 1000;
    float _deltaThresh = 0.3f;
    uint8_t _coeff = 8;
    uint16_t _counter;
    float _framedelta, _delta;
    float _hxfilt, _hyfilt, _hzfilt;
    float _hxmax, _hymax, _hzmax;
    float _hxmin, _hymin, _hzmin;
    float _hxb, _hyb, _hzb;
    float _hxs = 1.0f;
    float _hys = 1.0f;
    float _hzs = 1.0f;
    float _avgs;
    
    /* constants */
    const float G = 9.807f;
    const float _d2r = 3.14159265359f/180.0f;
    /* MPU9250 registers */
    const uint8_t ACCEL_OUT = 0x3B;
    const uint8_t GYRO_OUT = 0x43;
    const uint8_t TEMP_OUT = 0x41;
    const uint8_t EXT_SENS_DATA_00 = 0x49;
    const uint8_t ACCEL_CONFIG = 0x1C;
    const uint8_t ACCEL_FS_SEL_2G = 0x00;
    const uint8_t ACCEL_FS_SEL_4G = 0x08;
    const uint8_t ACCEL_FS_SEL_8G = 0x10;
    const uint8_t ACCEL_FS_SEL_16G = 0x18;
    const uint8_t GYRO_CONFIG = 0x1B;
    const uint8_t GYRO_FS_SEL_250DPS = 0x00;
    const uint8_t GYRO_FS_SEL_500DPS = 0x08;
    const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
    const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
    const uint8_t ACCEL_CONFIG2 = 0x1D;
    const uint8_t ACCEL_DLPF_184 = 0x01;
    const uint8_t ACCEL_DLPF_92 = 0x02;
    const uint8_t ACCEL_DLPF_41 = 0x03;
    const uint8_t ACCEL_DLPF_20 = 0x04;
    const uint8_t ACCEL_DLPF_10 = 0x05;
    const uint8_t ACCEL_DLPF_5 = 0x06;
    const uint8_t CONFIG = 0x1A;
    const uint8_t GYRO_DLPF_184 = 0x01;
    const uint8_t GYRO_DLPF_92 = 0x02;
    const uint8_t GYRO_DLPF_41 = 0x03;
    const uint8_t GYRO_DLPF_20 = 0x04;
    const uint8_t GYRO_DLPF_10 = 0x05;
    const uint8_t GYRO_DLPF_5 = 0x06;
    const uint8_t SMPDIV = 0x19;
    const uint8_t INT_PIN_CFG = 0x37;
    const uint8_t INT_ENABLE = 0x38;
    const uint8_t INT_DISABLE = 0x00;
    const uint8_t INT_PULSE_50US = 0x00;
    const uint8_t INT_WOM_EN = 0x40;
    const uint8_t INT_RAW_RDY_EN = 0x01;
    const uint8_t PWR_MGMNT_1 = 0x6B;
    const uint8_t PWR_CYCLE = 0x20;
    const uint8_t PWR_RESET = 0x80;
    const uint8_t CLOCK_SEL_PLL = 0x01;
    const uint8_t PWR_MGMNT_2 = 0x6C;
    const uint8_t SEN_ENABLE = 0x00;
    const uint8_t DIS_GYRO = 0x07;
    const uint8_t USER_CTRL = 0x6A;
    const uint8_t I2C_MST_EN = 0x20;
    const uint8_t I2C_MST_CLK = 0x0D;
    const uint8_t I2C_MST_CTRL = 0x24;
    const uint8_t I2C_SLV0_ADDR = 0x25;
    const uint8_t I2C_SLV0_REG = 0x26;
    const uint8_t I2C_SLV0_DO = 0x63;
    const uint8_t I2C_SLV0_CTRL = 0x27;
    const uint8_t I2C_SLV0_EN = 0x80;
    const uint8_t I2C_READ_FLAG = 0x80;
    const uint8_t MOT_DETECT_CTRL = 0x69;
    const uint8_t ACCEL_INTEL_EN = 0x80;
    const uint8_t ACCEL_INTEL_MODE = 0x40;
    const uint8_t LP_ACCEL_ODR = 0x1E;
    const uint8_t WOM_THR = 0x1F;
    const uint8_t WHO_AM_I = 0x75;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP = 0x80;
    const uint8_t FIFO_GYRO = 0x70;
    const uint8_t FIFO_ACCEL = 0x08;
    const uint8_t FIFO_MAG = 0x01;
    const uint8_t FIFO_COUNT = 0x72;
    const uint8_t FIFO_READ = 0x74;
    /* AK8963 registers */
    const uint8_t AK8963_I2C_ADDR = 0x0C;
    const uint8_t AK8963_HXL = 0x03; 
    const uint8_t AK8963_CNTL1 = 0x0A;
    const uint8_t AK8963_PWR_DOWN = 0x00;
    const uint8_t AK8963_CNT_MEAS1 = 0x12;
    const uint8_t AK8963_CNT_MEAS2 = 0x16;
    const uint8_t AK8963_FUSE_ROM = 0x0F;
    const uint8_t AK8963_CNTL2 = 0x0B;
    const uint8_t AK8963_RESET = 0x01;
    const uint8_t AK8963_ASA = 0x10;
    const uint8_t AK8963_WHO_AM_I = 0x00;
    
    
    /* writes a byte to MPU9250 register given a register address and data */
    int writeRegister(uint8_t subAddress, uint8_t data) {
        _i2c->beginTransmission(_address); // open the device
        _i2c->write(subAddress); // write the register address
        _i2c->write(data); // write the data
        _i2c->endTransmission();

        delay(10);

        /* read back the register */
        readRegisters(subAddress, 1, _buffer);
        /* check the read back register against the written register */
        if(_buffer[0] == data) {
            return 1;
        }
        else{
            return -1;
        }
    }

    /* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
        _i2c->beginTransmission(_address); // open the device
        _i2c->write(subAddress); // specify the starting register address
        _i2c->endTransmission(false);
        _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
        if (_numBytes == count) {
            for(uint8_t i = 0; i < count; i++) { 
                dest[i] = _i2c->read();
            }
            return 1;
        } else {
            return -1;
        }
    }

    /* writes a register to the AK8963 given a register address and data */
    int writeAK8963Register(uint8_t subAddress, uint8_t data) {
        // set slave 0 to the AK8963 and set for write
        if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
            return -1;
        }
        // set the register to the desired AK8963 sub address 
        if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
            return -2;
        }
        // store the data for write
        if (writeRegister(I2C_SLV0_DO,data) < 0) {
            return -3;
        }
        // enable _i2c and send 1 byte
        if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
            return -4;
        }
        // read the register and confirm
        if (readAK8963Registers(subAddress,1,_buffer) < 0) {
            return -5;
        }
        if(_buffer[0] == data) {
            return 1;
        } else{
            return -6;
        }
    }

    /* reads registers from the AK8963 */
    int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest) {
        // set slave 0 to the AK8963 and set for read
        if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
            return -1;
        }
        // set the register to the desired AK8963 sub address
        if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
            return -2;
        }
        // enable _i2c and request the bytes
        if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
            return -3;
        }
        delay(1); // takes some time for these registers to fill
        // read the bytes off the MPU9250 EXT_SENS_DATA registers
        _status = readRegisters(EXT_SENS_DATA_00,count,dest); 
        return _status;
    }

    /* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
    int whoAmI() {
        // read the WHO AM I register
        if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
            return -1;
        }
        // return the register value
        return _buffer[0];
    }

    /* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
    int whoAmIAK8963() {
        // read the WHO AM I register
        if (readAK8963Registers(AK8963_WHO_AM_I,1,_buffer) < 0) {
            return -1;
        }
        // return the register value
        return _buffer[0];
    }

};

#endif
