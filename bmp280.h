#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** Description:             Define i2c address of bmp sensor
***************************************************************************************/
typedef enum {
	I2C_ADDRESS_0	= 0x76,
	I2C_ADDRESS_1	= 0x77
} BMP280_Address;

/***************************************************************************************
** Description:             Define chip id's of bmp sensors
***************************************************************************************/
typedef enum {
	BMP280_CHIP_ID	= 0x58,		// BMP280 ==> 0x58
	BME280_CHIP_ID	= 0x60		// BME280 ==> 0x60
} BMP280_ChipID;

/***************************************************************************************
** Description:             Define registers
***************************************************************************************/
typedef enum {
	REG_TEMP_XLSB	= 0xFC,			// bits: 7-4
	REG_TEMP_LSB	= 0xFB,
	REG_TEMP_MSB	= 0xFA,
	REG_TEMP		= REG_TEMP_MSB,
	REG_PRESS_XLSB	= 0xF9,			// bits: 7-4
	REG_PRESS_LSB	= 0xF8,
	REG_PRESS_MSB	= 0xF7,
	REG_PRESSURE	= REG_PRESS_MSB,
	REG_CONFIG		= 0xF5,			// bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en
	REG_CTRL		= 0xF4,			//bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode
	REG_STATUS		= 0xF3,			//bits: 3 measuring; 0 im_update
	REG_CTRL_HUM	= 0xF2,			//bits: 2-0 osrs_h
	REG_RESET		= 0xE0,
	REG_ID			= 0xD0,
	REG_CALIB		= 0x88,
	REG_HUM_CALIB	= 0x88,
	RESET_VALUE		= 0xB6
} BMP280_Registers;


/***************************************************************************************
** Description:             Define type of work:
** 							- Forced - Measurement is initiated by user
** 							- Normal - Continous measurement
***************************************************************************************/
typedef enum {
    MODE_SLEEP = 0,
    MODE_FORCED = 1,
    MODE_NORMAL = 3
} BMP280_Mode;


/***************************************************************************************
** Description:             Filter settings for measurement
***************************************************************************************/
typedef enum {
    FILTER_OFF = 0,
    FILTER_X2 = 1,
    FILTER_X4 = 2,
    FILTER_X8 = 3,
    FILTER_X16 = 4
} BMP280_Filter;


/***************************************************************************************
** Description:             Oversampling settings for pressure measurement
***************************************************************************************/
typedef enum {
    SKIPPED_MEASUREMENTS = 0,          /* no measurement  */
    OVERSAMPLE_X1 = 1,  /* oversampling x1 */
	OVERSAMPLE_X2 = 2,        /* oversampling x2 */
	OVERSAMPLE_X4 = 3,         /* oversampling x4 */
	OVERSAMPLE_X8 = 4,         /* oversampling x8 */
	OVERSAMPLE_X16 = 5    /* oversampling x16 */
} BMP280_Oversampling;


/***************************************************************************************
** Description:             Time of standby between measurements
***************************************************************************************/
typedef enum {
    STANDBY_05		= 0,		// standby for 0.5ms
    STANDBY_62		= 1,		// standby for 62.5ms
    STANDBY_125		= 2,		// standby for 125ms
    STANDBY_250		= 3,		// standby for 250ms
    STANDBY_500		= 4,		// standby for 500ms
    STANDBY_1000	= 5,		// standby for 1s
    STANDBY_2000	= 6,		// standby for 2s BMP280, 10ms BME280
    STANDBY_4000	= 7,		// standby for 4s BMP280, 20ms BME280
} BMP280_StandbyTime;


/***************************************************************************************
** Description:             Configuration parameters for BMP module
***************************************************************************************/
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} BMP280_Params;


class BMP280_LIB {
	private:
		// Temperature & Pressure compensation
		uint16_t T1;
		int16_t  T2;
		int16_t  T3;
		uint16_t P1;
		int16_t  P2;
		int16_t  P3;
		int16_t  P4;
		int16_t  P5;
		int16_t  P6;
		int16_t  P7;
		int16_t  P8;
		int16_t  P9;

		// Humidity compensation for BME280
		uint8_t  H1;
		int16_t  H2;
		uint8_t  H3;
		int16_t  H4;
		int16_t  H5;
		int8_t   H6;

		uint16_t dev_addr;

		I2C_HandleTypeDef* _bus;

		BMP280_Params parameters;

		uint8_t  id;

		bool readReg16(uint8_t addr, uint16_t *value);
		int writeReg8(uint8_t addr, uint8_t value);
		inline int readData(uint8_t addr, uint8_t *value, uint8_t len);
		bool readCalData();
		bool readHumCalData();
		inline int32_t compensateTemperature(int32_t adc_temp, int32_t *fine_temp);
		inline uint32_t compensatePressure(int32_t adc_press, int32_t fine_temp);
		inline uint32_t compensateHumidity(int32_t adc_hum, int32_t fine_temp);
	public:

		// Object constructor.
		BMP280_LIB(I2C_HandleTypeDef &bus, uint8_t address);

		// Object destructor.
		~BMP280_LIB();

		/***************************************************************************************
		** Description:             Initialize BMP sensor with default parameters:
		** 							- Normal mode operation,
		** 							- Filtering x16,
		** 							- Pressure oversampling x8,
		** 							- Temperature oversampling x8,
		** 							- Standby for 250ms,
		***************************************************************************************/
		void initDefaultParams(void);

		// Initialize BMP sensor with custom parameters.
		bool init(BMP280_Params *params);

		uint8_t getID(void);

		// Start measurement in Forced Mode.
		bool forceMeasurement(void);

		// Check if BMP sensor is busy - return true if is.
		bool isMeasuring(void);

		// Read data from BMP sensor - return 32bit values.
		bool readFixed(int32_t *temperature, uint32_t *pressure, uint32_t *humidity);

		// Read data from BMP sensor - return float values.
		bool readFloat(float *temperature, float *pressure, float *humidity);
};
