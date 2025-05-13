/*
	Example of using mkprogigor/bme280
*/

#include <Arduino.h>
#include "bme280.h"

bme280 bme;
struct_tph result;

void setup() {
	Serial.begin(115200);

	uint8_t k = bme.f_check_bme();
	if (k == 0) {
		Serial.println("BME280 not found. Check wires conection.");	
	}
	else {
		Serial.print("BME280 is present. Chip code = ");
		Serial.println(k, HEX);
	}
	bme.begin();
}

void main() {
	bme.f_do_1_meas();
	result = bme.f_read_TPH();

	Serial.print("Temperature, C = ");
	Serial.println((float)(result.temp1)/100);

	Serial.print("Pressure, Pa = ");
	Serial.println((float)(result.pres1)/100);

	Serial.print("Humidity, % = ");
	Serial.println((float)(result.humi1)/1000);

	delay(3000);
}
