# bme280
Lightweight and short library for Bosch sensor BME280 for Arduino (in development).

It operate and return integer values of t,p,h at once.
Further you can convert to float in calling procedure.
By default, lib focused on one time measuring (forced mode, but normal available too) 
with max oversampling x16 of t,p,h, and filter x16. It's suitable for weather station.

Function => **uint8_t f_check_bme()**<BR>
check connection with bme280, return byte: 0 if not present bme280 or CHIP CODE, and makes SW reset.<br>
`/*  fn return 0x58=>BMP280, 0x60=>BME280, 0x61=>BME680. IF not present 0x76, 0x77 then return 0.
    i2c address 0x77 possible for BMP280 or BME280 or BME680 or MS5607,MS5611,MS5637
    note: address 0x77 may be BMP085, BMA180 and may not be MS5607 or MS5637 CHECK */`

Function => **bool begin()**<BR>
setup bme280 to FORCED MODE, with x16 oversampling for T,P,H and x16 filter. Return TRUE when is Ok. :-)

Function => **bool begin(uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h)**<BR>
init bme280 with Your parameters. Return TRUE when is Ok. :-)<BR>
*mode = NOR_MODE or FOR_MODE. <BR>
t_sb = standby (sleep) time, from SB_500US to SD_1000MS<BR>
filter  =  value of filterring - FIL_OFF, FIL_x2, FIL_x4, FIL_x8, FIL_x16<BR>
osrs_t, osrs_p, osrs_h = oversampling value for T,P,H - OS_OFF, OS_x1, OS_x2, OS_x4, OS_x8, OS_x16*

Function => **void f_do_1_meas(void)**<BR>
makes 1 measurement and goes to sleep. You should make delay for waitung result.
Acording to mode, sleep time, oversamlinhg value it can takes 200 ms. It's possible use vTaskDelay(200) in FreeRTOS.

Function => **bool f_bme_is_meas(void)**<BR>
returns TRUE while the bme280 IS measuring, otherwise FALSE.

Function => **struct_tph f_read_TPH(void)**<BR>
makes one measurement, wait for result, and reading value t,p,h for one I2C request and return structure:
```c++
struct struct_tph {
  int32_t   temp1;  //  real t = t * 100
  uint32_t  pres1;  //  real p = p * 100
  uint32_t  humi1;  //  real h = h * 1000
};
```
real values can get: T = ((float)(struct_tph.temp1)) / 100.

Functions don't use of delay, it should do it manually for benefits of using FreeRTOS.
Or usuing CPU resources for checkking results. Max measuring time takes about 200 mS, but You can check it.

I used oficial Bosch datasheet bme280 and examples for making this lib:<BR>
https://github.com/GyverLibs/GyverBME280<BR>
https://github.com/farmerkeith/BMP280-library/<BR>
https://github.com/farmerkeith/I2CScanner/<BR>

