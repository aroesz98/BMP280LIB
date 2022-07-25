#include "bmp280.h"

/***************************************************************************************
** Description:             Object constructor.
***************************************************************************************/
BMP280_LIB::BMP280_LIB(I2C_HandleTypeDef &bus, uint8_t dev_address) {
	_bus = &bus;
	dev_addr = dev_address << 1;
}


/***************************************************************************************
** Description:             Object destructor.
***************************************************************************************/
BMP280_LIB::~BMP280_LIB() { ;}


/***************************************************************************************
** Description:             Read 16bit value from register
***************************************************************************************/
bool BMP280_LIB::readReg16(uint8_t addr, uint16_t *value) {
	uint8_t rx_buff[2];

	if (HAL_I2C_Mem_Read(_bus, dev_addr, addr, 1, rx_buff, 2, 1000) == HAL_OK) {
		*value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
		return true;
	}

	else return false;
}


/***************************************************************************************
** Description:             Read data from bmp280
***************************************************************************************/
inline int BMP280_LIB::readData(uint8_t addr, uint8_t *value, uint8_t len) {
	if (HAL_I2C_Mem_Read(_bus, dev_addr, addr, 1, value, len, 1000) == HAL_OK) return 0;
	else return 1;

}


/***************************************************************************************
** Description:             Read calibration data (Temperature & Pressure)
***************************************************************************************/
bool BMP280_LIB::readCalData(void) {

	if (readReg16(0x88, &T1)
			&& readReg16(0x8a, (uint16_t *) &T2)
			&& readReg16(0x8c, (uint16_t *) &T3)
			&& readReg16(0x8e, &P1)
			&& readReg16(0x90, (uint16_t *) &P2)
			&& readReg16(0x92, (uint16_t *) &P3)
			&& readReg16(0x94, (uint16_t *) &P4)
			&& readReg16(0x96, (uint16_t *) &P5)
			&& readReg16(0x98, (uint16_t *) &P6)
			&& readReg16(0x9a, (uint16_t *) &P7)
			&& readReg16(0x9c, (uint16_t *) &P8)
			&& readReg16(0x9e,
					(uint16_t *) &P9)) {

		return true;
	}

	return false;
}


/***************************************************************************************
** Description:             Read calibration data (Humidity)
***************************************************************************************/
bool BMP280_LIB::readHumCalData(void) {
	uint16_t h4, h5;

	if (!readData(0xa1, &H1, 1)
			&& readReg16(0xe1, (uint16_t *) &H2)
			&& !readData(0xe3, &H3, 1)
			&& readReg16(0xe4, &h4)
			&& readReg16(0xe5, &h5)
			&& !readData(0xe7, (uint8_t *) &H6, 1)) {
			H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
			H5 = h5 >> 4;

			return true;
	}

	return false;
}


/***************************************************************************************
** Description:             Write byte to register
***************************************************************************************/
int BMP280_LIB::writeReg8(uint8_t addr, uint8_t value) {
	if (HAL_I2C_Mem_Write(_bus, dev_addr, addr, 1, &value, 1, 10000) == HAL_OK) return false;
	else return true;
}


/***************************************************************************************
** Description:             Define default parameters
***************************************************************************************/
void BMP280_LIB::initDefaultParams(void) {
	parameters.mode = MODE_NORMAL;
	parameters.filter = FILTER_X16;
	parameters.oversampling_pressure = OVERSAMPLE_X8;
	parameters.oversampling_temperature = OVERSAMPLE_X8;
	parameters.oversampling_humidity = OVERSAMPLE_X8;
	parameters.standby = STANDBY_250;
	init(&parameters);
}


/***************************************************************************************
** Description:             Initialize BMX280 sensor
***************************************************************************************/
bool BMP280_LIB::init(BMP280_Params *params) {

	if (readData(REG_ID, &id, 1)) {
		return false;
	}

	if (id != BMP280_CHIP_ID && id != BME280_CHIP_ID) {

		return false;
	}

	if (writeReg8(REG_RESET, RESET_VALUE)) {
		return false;
	}

	// Wait until finished copying over the NVP data.
	while (1) {
		uint8_t status;
		if (!readData(REG_STATUS, &status, 1) && (status & 1) == 0)
			break;
	}

	if (!readCalData()) return false;

	if (id == BME280_CHIP_ID && !readHumCalData()) return false;

	uint8_t config = (params->standby << 5) | (params->filter << 2);
	if (writeReg8(REG_CONFIG, config)) return false;

	if (params->mode == MODE_FORCED) params->mode = MODE_SLEEP;

	uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | (params->mode);

	if (id == BME280_CHIP_ID) {
		// Write crtl hum reg first, only active after write to REG_CTRL.
		uint8_t ctrl_hum = params->oversampling_humidity;
		if (writeReg8(REG_CTRL_HUM, ctrl_hum)) return false;
	}

	if (writeReg8(REG_CTRL, ctrl)) return false;

	return true;
}


uint8_t BMP280_LIB::getID(void) {
	return id;
}


/***************************************************************************************
** Description:             Read sensor (forced mode)
***************************************************************************************/
bool BMP280_LIB::forceMeasurement(void) {
	uint8_t ctrl;

	if (readData(REG_CTRL, &ctrl, 1)) return false;

	ctrl &= ~0b11;  // clear two lower bits
	ctrl |= MODE_FORCED;

	if (writeReg8(REG_CTRL, ctrl)) return false;

	return true;
}


/***************************************************************************************
** Description:             Check if sensor is measuring
***************************************************************************************/
bool BMP280_LIB::isMeasuring(void) {
	uint8_t status;
	if (readData(REG_STATUS, &status, 1)) return false;
	if (status & (1 << 3)) return true;
	return false;
}


/***************************************************************************************
** Description:             Compensation algorithm is taken from BMX280 datasheet (Return C degrees)
***************************************************************************************/
inline int32_t BMP280_LIB::compensateTemperature(int32_t adc_temp, int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) T1 << 1))) * (int32_t) T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) T1)
			* ((adc_temp >> 4) - (int32_t) T1)) >> 12)
			* (int32_t) T3) >> 14;

	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}


/***************************************************************************************
** Description:             Compensation algorithm is taken from BMX280 datasheet (Return Pa)
***************************************************************************************/
inline uint32_t BMP280_LIB::compensatePressure(int32_t adc_press, int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) P6;
	var2 = var2 + ((var1 * (int64_t) P5) << 17);
	var2 = var2 + (((int64_t) P4) << 35);
	var1 = ((var1 * var1 * (int64_t) P3) >> 8) + ((var1 * (int64_t) P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) P7 << 4);
	return p;
}


/***************************************************************************************
** Description:             Compensation algorithm is taken from BMX280 datasheet
***************************************************************************************/
inline uint32_t BMP280_LIB::compensateHumidity(int32_t adc_hum, int32_t fine_temp) {
	int32_t v_x1_u32r;

	v_x1_u32r = fine_temp - (int32_t) 76800;
	v_x1_u32r = ((((adc_hum << 14) - ((int32_t) H4 << 20)
				- ((int32_t) H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
				* (((((((v_x1_u32r * (int32_t) H6) >> 10)
				* (((v_x1_u32r * (int32_t) H3) >> 11)
				+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
				* (int32_t) H2 + 8192) >> 14);
	v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (int32_t) H1) >> 4);
	v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
	v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;

	return v_x1_u32r >> 12;
}


/***************************************************************************************
** Description:             Read measurement from sensor
***************************************************************************************/
bool BMP280_LIB::readFixed(int32_t *temperature, uint32_t *pressure, uint32_t *humidity) {
	int32_t adcPress;
	int32_t adcTemp;
	uint8_t data[8];

	// Only the BME280 supports reading the humidity.
	if (id != BME280_CHIP_ID) {
		if (humidity) *humidity = 0;
		humidity = NULL;
	}

	// Need to read in one sequence to ensure they match.
	size_t size = humidity ? 8 : 6;
	if (readData(0xf7, data, size)) return false;

	adcPress = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adcTemp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	int32_t fine_temp;
	*temperature = compensateTemperature(adcTemp, &fine_temp);
	*pressure = compensatePressure(adcPress, fine_temp);

	if (humidity) {
		int32_t adcHum = data[6] << 8 | data[7];
		*humidity = compensateHumidity(adcHum, fine_temp);
	}

	return true;
}


/***************************************************************************************
** Description:             Read measurement from sensor (Floating point)
***************************************************************************************/
bool BMP280_LIB::readFloat(float *temperature, float *pressure, float *humidity) {
	int32_t fixedTemp;
	uint32_t fixedPress;
	uint32_t fixedHum;

	if (readFixed(&fixedTemp, &fixedPress, humidity ? &fixedHum : NULL)) {
		*temperature = (float) fixedTemp / 100;
		*pressure = (float) fixedPress / 256;

		if (humidity) *humidity = (float) fixedHum / 1024;
		return true;
	}

	return false;
}
