/************************************************************************************
Library for test and use in weather stations with Bosch i2c sensor bme280
by Igor Mkprog, mkprogigor@gmail.com

use examples:
https://github.com/AlexGyver/GyverLibs/blob/master/GyverBME280/
https://github.com/farmerkeith/BMP280-library/
https://github.com/farmerkeith/I2CScanner/

V1.0 from 01.06.2025
************************************************************************************/
#include "bme280.h"

uint8_t _i2c_address = 0x76;
uint8_t _reg_0xF2 = 0x07;	//	config regs osr_h[2:0]
uint8_t _reg_0xF4 = 0xFE;	//	config regs osr_t[7:5] osr_p[4:2] mode[1:0]
uint8_t _reg_0xF5 = 0x9C;	//	config regs t_sb[7:5] filter[4:2] spi3w_en[0]

uint8_t bme280::bme_check(void) {		// check is it conected bme280,
	//	if YES ret chip_code (0x60) then do software reset, if NO ret 0. 
	Wire.begin(); // start I2C interface
	uint8_t error;
    Wire.beginTransmission(_i2c_address);
    error = Wire.endTransmission();
    if (error == 0) {
        Wire.beginTransmission(_i2c_address);
        Wire.write(0xD0);   // Select register 0xD0 hex address of chip_id
        Wire.endTransmission();
        Wire.requestFrom(_i2c_address, 1);
        if (Wire.available() == 1)  return Wire.read();
		_f_reset();
    }
    else {
        Wire.beginTransmission(_i2c_address + 1);
        error = Wire.endTransmission();
        if (error == 0) {
            _i2c_address = _i2c_address + 1;    //  0x77
            Wire.beginTransmission(_i2c_address);
            Wire.write(0xD0);   // Select register 0xD0 hex address of chip_id
            Wire.endTransmission();
            Wire.requestFrom(_i2c_address, 1);
            if (Wire.available() == 1)  return Wire.read();
			_f_reset();
        }
    }
/*  fn return 0x58>BMP280, 0x60>BME280, 0x61>BME680. IF not present 0x76, 0x77 then return 0.
    i2c address 0x77 possible for BMP280 or BME280 or BME680 or MS5607,MS5611,MS5637
    note: address 0x77 may be BMP085, BMA180 and may not be MS5607 or MS5637 CHECK */
    return 0;
}

bool bme280::begin() {	// defaults are 16x; Normal mode; 0.5ms, no filter, I2C
	return begin(0x02, 0x04, 0x07, 0x07, 0x07, 0x07); // Forse mode, sleep 500ms, filter x16, t p h x16
  }

bool bme280::begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h) {	// init bme280
//	Wire.begin(); // start I2C interface
	bme280::_f_read_calibr_coeff();
	_reg_0xF2 = osrs_h;
	_reg_0xF4 = (osrs_t<<5) | (osrs_p<<2) | mode;
	_reg_0xF5 = (t_sb  <<5) | (filter<<2) | 0x00;
	bme280::_f_write_reg(0xF2, _reg_0xF2);   
	bme280::_f_write_reg(0xF4, _reg_0xF4);   
	bme280::_f_write_reg(0xF5, _reg_0xF5);
	return true;
};    

bool bme280::bme_is_meas(void)	{	// returns TRUE while bme280 is measuring								
	return (bool)((bme280::_f_read_reg(0xF3) & 0x08) >> 3);  	 // read status register & mask bit "measuring"
}

void bme280::bme_do1meas(void) {        // operating mode FORCED_MODE do 1 measuring
	bme280::_f_write_reg( 0xF4, ((_reg_0xF4 & 0xFC) | 0x02) );   
}

struct_tph bme280::bme_read_TPH(void) {
    #define BME280_S32_t int32_t
    #define BME280_U32_t uint32_t
    #define BME280_S64_t int64_t
    struct_tph lv_tph;

	uint8_t _lv_regs[8];
	Wire.beginTransmission(_i2c_address);   // addr of first byte raw data Humi
	Wire.write(0xF7);
	if (Wire.endTransmission() != 0) return { 0, 0, 0 };
	if (Wire.requestFrom(_i2c_address, 8) == 8) {    // reading 8 regs
		for (uint8_t i = 0; i < 8; i++) _lv_regs[i] = Wire.read();
	}
	int32_t  adc_T = (_lv_regs[3] << 16) | (_lv_regs[4] << 8) | _lv_regs[5];
	uint32_t adc_P = (_lv_regs[0] << 16) | (_lv_regs[1] << 8) | _lv_regs[2];
	int32_t  adc_H = (_lv_regs[6] << 8)  | _lv_regs[7];

    // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
    // t_fine carries fine temperature as global value
	BME280_S32_t lv_var1, lv_var2, t_fine;
	if (adc_T == 0x800000) {
		lv_tph.temp1 = 0;	// if the temperature module has been disabled return '0'
	}
	else {
		adc_T >>= 4;
		lv_var1 = ((((adc_T >> 3) - ((BME280_S32_t)_cal_data.dig_T1 << 1))) * ((BME280_S32_t)_cal_data.dig_T2)) >> 11;
		lv_var2 = (((((adc_T >> 4) - ((BME280_S32_t)_cal_data.dig_T1)) * ((adc_T >> 4) - ((BME280_S32_t)_cal_data.dig_T1))) >> 12)
			* ((BME280_S32_t)_cal_data.dig_T3)) >> 14;
		t_fine = lv_var1 + lv_var2;
		lv_tph.temp1  = (t_fine * 5 + 128) >> 8;
	}
  
    // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	BME280_S64_t var1, var2, p;
	if (adc_P == 0x800000) {
		lv_tph.pres1 = 0;	// If the pressure module has been disabled return '0'
	}
	else {
		adc_P >>= 4;	// Start pressure converting
		var1 = ((BME280_S64_t)t_fine) - 128000;
		var2 = var1 * var1 * (BME280_S64_t)_cal_data.dig_P6;
		var2 = var2 + ((var1 * (BME280_S64_t)_cal_data.dig_P5) << 17);
		var2 = var2 + (((BME280_S64_t)_cal_data.dig_P4) << 35);
		var1 = ((var1 * var1 * (BME280_S64_t)_cal_data.dig_P3) >> 8) + ((var1 * (BME280_S64_t)_cal_data.dig_P2) << 12);
		var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)_cal_data.dig_P1) >> 33;
		if (var1 == 0) {
			lv_tph.pres1 = 0;     // avoid exception caused by division by zero
		}
		else {
			p = 1048576 - adc_P;
			p = (((p << 31) - var2) * 3125) / var1;
			var1 = (((BME280_S64_t)_cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
			var2 = (((BME280_S64_t)_cal_data.dig_P8) * p) >> 19;
			p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)_cal_data.dig_P7) << 4);
			lv_tph.pres1 = (BME280_U32_t)((p * 100) / 256);
		}
	}
    
    // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
    // Output value of “47445” represents 47445/1024 = 46.333 %RH
	BME280_S32_t v_x1_u32r;
	if (adc_H == 0x8000) {
		lv_tph.humi1 = 0;	// If the humidity module has been disabled return '0'
	}
	else {
		v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
		v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)_cal_data.dig_H4) << 20) - (((BME280_S32_t)_cal_data.dig_H5) *
			v_x1_u32r)) + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *
				((BME280_S32_t)_cal_data.dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)_cal_data.dig_H3)) >> 11) +
					((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)_cal_data.dig_H2) + 8192) >> 14));
		v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)_cal_data.dig_H1)) >> 4));
		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
		lv_tph.humi1 = (BME280_U32_t)(((v_x1_u32r >> 12)*1000)/1024);
	}
    return lv_tph;
}

bme280::bme280() {};	// create an object of class bme280

//============================================
//    private metods (funcs)
//============================================
uint8_t bme280::_f_read_reg(uint8_t address) {    // read 1 byte bme280 register
	Wire.beginTransmission(_i2c_address);
	Wire.write(address);
	if (Wire.endTransmission() != 0) return 0;
	if (Wire.requestFrom(_i2c_address, 1) == 1) return Wire.read();
	else return 0;
}

bool bme280::_f_write_reg(uint8_t address , uint8_t data) {   //  write 1 byte to bme280 register
	Wire.beginTransmission(_i2c_address);
	Wire.write(address);
	Wire.write(data);
	if (Wire.endTransmission() == 0) return true;
	else return false;
}

bool bme280::_f_reset(void) {     //  software reset of bme280
	if (bme280::_f_write_reg(0x0E, 0xB6)) return true;
    else return false;
}

void bme280::_f_read_calibr_coeff(void) {
    uint8_t _tv_regs[25];
	
	Wire.beginTransmission(_i2c_address);   // first part request
	Wire.write(0x88);
	if (Wire.endTransmission() != 0) return;
	if (Wire.requestFrom(_i2c_address, 26) == 26) {    // reading 26 regs
        for (uint8_t i = 0; i < 25; i++) _tv_regs[i] = Wire.read();
	
        _cal_data.dig_T1 = _tv_regs[1]<<8 | _tv_regs[0];   // form struct
        _cal_data.dig_T2 = _tv_regs[3]<<8 | _tv_regs[2];
        _cal_data.dig_T3 = _tv_regs[5]<<8 | _tv_regs[4];
        _cal_data.dig_P1 = _tv_regs[7]<<8 | _tv_regs[6];
        _cal_data.dig_P2 = _tv_regs[9]<<8 | _tv_regs[8];
        _cal_data.dig_P3 = _tv_regs[11]<<8 | _tv_regs[10];
        _cal_data.dig_P4 = _tv_regs[13]<<8 | _tv_regs[12];
        _cal_data.dig_P5 = _tv_regs[15]<<8 | _tv_regs[14];
        _cal_data.dig_P6 = _tv_regs[17]<<8 | _tv_regs[16];
        _cal_data.dig_P7 = _tv_regs[19]<<8 | _tv_regs[18];
        _cal_data.dig_P8 = _tv_regs[21]<<8 | _tv_regs[20];
        _cal_data.dig_P9 = _tv_regs[23]<<8 | _tv_regs[22];
        _cal_data.dig_H1 = _tv_regs[25];
    }
	
	Wire.beginTransmission(_i2c_address);   // second part request
	Wire.write(0xE1);
	Wire.endTransmission();
	if (Wire.requestFrom(_i2c_address, 7) == 7) {   // reading
        for (uint8_t i = 0; i < 7; i++) _tv_regs[i] = Wire.read();
        _cal_data.dig_H2 = _tv_regs[1]<<8 | _tv_regs[0]; // (Wire.read() | (Wire.read() << 8));
        _cal_data.dig_H3 = _tv_regs[2];
        _cal_data.dig_H4 = ((int16_t)_tv_regs[3] << 4) | (_tv_regs[4] & 0x0F);
        _cal_data.dig_H5 = ((int16_t)_tv_regs[5] << 4) | ((_tv_regs[4] & 0xF0) >> 4);
        _cal_data.dig_H6 = _tv_regs[6];
    };
}