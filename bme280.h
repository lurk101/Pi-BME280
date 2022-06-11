
#ifndef BME280_H_
#define BME280_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/********************************************************/

#ifndef BME280_64BIT_ENABLE /*< Check if 64-bit integer (using BME280_64BIT_ENABLE) is enabled */
#ifndef BME280_32BIT_ENABLE /*< Check if 32-bit integer (using BME280_32BIT_ENABLE) is enabled */
#ifndef BME280_FLOAT_ENABLE /*< If any of the integer data types not enabled then enable           \
                               BME280_FLOAT_ENABLE */
#define BME280_FLOAT_ENABLE
#endif
#endif
#endif

/**
 * BME280_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the
 * build system.
 */
#ifndef BME280_INTF_RET_TYPE
#define BME280_INTF_RET_TYPE int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BME280_INTF_RET_SUCCESS
#define BME280_INTF_RET_SUCCESS 0
#endif

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM 0x76
#define BME280_I2C_ADDR_SEC 0x77

/**\name BME280 chip identifier */
#define BME280_CHIP_ID 0x60

/**\name Register Address */
#define BME280_CHIP_ID_ADDR 0xD0
#define BME280_RESET_ADDR 0xE0
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR 0x88
#define BME280_HUMIDITY_CALIB_DATA_ADDR 0xE1
#define BME280_PWR_CTRL_ADDR 0xF4
#define BME280_CTRL_HUM_ADDR 0xF2
#define BME280_CTRL_MEAS_ADDR 0xF4
#define BME280_CONFIG_ADDR 0xF5
#define BME280_DATA_ADDR 0xF7

/**\name API success code */
#define BME280_OK 0

/**\name API error codes */
#define BME280_E_DEV_NOT_FOUND -2
#define BME280_E_INVALID_LEN -3
#define BME280_E_COMM_FAIL -4
#define BME280_E_SLEEP_MODE_FAIL -5
#define BME280_E_NVM_COPY_FAILED -6

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO 1

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN 26
#define BME280_HUMIDITY_CALIB_DATA_LEN 7
#define BME280_P_T_H_DATA_LEN 8

/**\name Sensor power modes */
#define BME280_SLEEP_MODE 0x00
#define BME280_FORCED_MODE 0x01
#define BME280_NORMAL_MODE 0x03

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data)                                                   \
    ((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data)                                             \
    ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname) ((reg_data & (bitname##_MSK)) >> (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

/**\name Macros for bit masking */
#define BME280_SENSOR_MODE_MSK 0x03
#define BME280_SENSOR_MODE_POS 0x00

#define BME280_CTRL_HUM_MSK 0x07
#define BME280_CTRL_HUM_POS 0x00

#define BME280_CTRL_PRESS_MSK 0x1C
#define BME280_CTRL_PRESS_POS 0x02

#define BME280_CTRL_TEMP_MSK 0xE0
#define BME280_CTRL_TEMP_POS 0x05

#define BME280_FILTER_MSK 0x1C
#define BME280_FILTER_POS 0x02

#define BME280_STANDBY_MSK 0xE0
#define BME280_STANDBY_POS 0x05

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.
 */
#define BME280_PRESS 1
#define BME280_TEMP (1 << 1)
#define BME280_HUM (1 << 2)
#define BME280_ALL 0x07

/**\name Settings selection macros */
#define BME280_OSR_PRESS_SEL 1
#define BME280_OSR_TEMP_SEL (1 << 1)
#define BME280_OSR_HUM_SEL (1 << 2)
#define BME280_FILTER_SEL (1 << 3)
#define BME280_STANDBY_SEL (1 << 4)
#define BME280_ALL_SETTINGS_SEL 0x1F

/**\name Oversampling macros */
#define BME280_NO_OVERSAMPLING 0x00
#define BME280_OVERSAMPLING_1X 0x01
#define BME280_OVERSAMPLING_2X 0x02
#define BME280_OVERSAMPLING_4X 0x03
#define BME280_OVERSAMPLING_8X 0x04
#define BME280_OVERSAMPLING_16X 0x05

/**\name Measurement delay calculation macros  */
#define BME280_MEAS_OFFSET 1250
#define BME280_MEAS_DUR 2300
#define BME280_PRES_HUM_MEAS_OFFSET 575
#define BME280_MEAS_SCALING_FACTOR 1000

/**\name Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS 0x00
#define BME280_STANDBY_TIME_62_5_MS 0x01
#define BME280_STANDBY_TIME_125_MS 0x02
#define BME280_STANDBY_TIME_250_MS 0x03
#define BME280_STANDBY_TIME_500_MS 0x04
#define BME280_STANDBY_TIME_1000_MS 0x05
#define BME280_STANDBY_TIME_10_MS 0x06
#define BME280_STANDBY_TIME_20_MS 0x07

/**\name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF 0x00
#define BME280_FILTER_COEFF_2 0x01
#define BME280_FILTER_COEFF_4 0x02
#define BME280_FILTER_COEFF_8 0x03
#define BME280_FILTER_COEFF_16 0x04

#define BME280_STATUS_REG_ADDR 0xF3
#define BME280_SOFT_RESET_COMMAND 0xB6
#define BME280_STATUS_IM_UPDATE 0x01

/**
 * Type definitions
 */

/**
 * Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * param[in] reg_addr       : Register address from which data is read.
 * param[out] reg_data     : Pointer to data buffer where read data is stored.
 * param[in] len            : Number of bytes of data to be read.
 * param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 * retval   0 -> Success.
 * retval Non zero value -> Fail.
 *
 */
typedef BME280_INTF_RET_TYPE (*bme280_read_fptr_t)(uint8_t reg_addr, uint8_t* reg_data,
                                                   uint32_t len, void* intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef BME280_INTF_RET_TYPE (*bme280_write_fptr_t)(uint8_t reg_addr, const uint8_t* reg_data,
                                                    uint32_t len, void* intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bme280_delay_us_fptr_t)(uint32_t period, void* intf_ptr);

/*!
 * @brief Calibration data
 */
struct bme280_calib_data {
    /*< Calibration coefficient for the temperature sensor */
    uint16_t dig_t1;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t2;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t3;

    /*< Calibration coefficient for the pressure sensor */
    uint16_t dig_p1;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p2;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p3;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p4;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p5;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p6;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p7;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p8;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p9;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h1;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h2;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h3;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h4;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h5;

    /*< Calibration coefficient for the humidity sensor */
    int8_t dig_h6;

    /*< Variable to store the intermediate temperature coefficient */
    int32_t t_fine;
};

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
#ifdef BME280_FLOAT_ENABLE
struct bme280_data {
    /*< Compensated pressure */
    double pressure;

    /*< Compensated temperature */
    double temperature;

    /*< Compensated humidity */
    double humidity;
};
#else
struct bme280_data {
    /*< Compensated pressure */
    uint32_t pressure;

    /*< Compensated temperature */
    int32_t temperature;

    /*< Compensated humidity */
    uint32_t humidity;
};
#endif /*! BME280_USE_FLOATING_POINT */

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
struct bme280_uncomp_data {
    /*< un-compensated pressure */
    uint32_t pressure;

    /*< un-compensated temperature */
    uint32_t temperature;

    /*< un-compensated humidity */
    uint32_t humidity;
};

/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */
struct bme280_settings {
    /*< pressure oversampling */
    uint8_t osr_p;

    /*< temperature oversampling */
    uint8_t osr_t;

    /*< humidity oversampling */
    uint8_t osr_h;

    /*< filter coefficient */
    uint8_t filter;

    /*< standby time */
    uint8_t standby_time;
};

/*!
 * @brief bme280 device structure
 */
struct bme280_dev {
    /* Interface function pointer used to enable the device address for I2C and chip selection for
     * SPI */
    void* intf_ptr;

    /*< Read function pointer */
    bme280_read_fptr_t read;

    /*< Write function pointer */
    bme280_write_fptr_t write;

    /*< Delay function pointer */
    bme280_delay_us_fptr_t delay_us;

    /*< Trim data */
    struct bme280_calib_data calib_data;

    /*< Sensor settings */
    struct bme280_settings settings;

    /*< Variable to store result of read/write function */
    BME280_INTF_RET_TYPE intf_rslt;
};

/**
 * Initialize the sensor and device structure
 */

/**
 * This API reads the chip-id of the sensor which is the first step to
 * verify the sensor and also calibrates the sensor
 * As this API is the entry point, call this API before using other APIs.
 *
 * param[in,out] dev : Structure instance of bme280_dev
 *
 * return Result of API execution status.
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_init(struct bme280_dev* dev);

/**
 * Generic API for accessing sensor registers
 */

/**
 * This API writes the given data to the register address of the sensor
 *
 * param[in] reg_addr : Register addresses to where the data is to be written
 * param[in] reg_data : Pointer to data buffer which is to be written
 *                       in the reg_addr of sensor.
 * param[in] len      : No of bytes of data to write
 * param[in,out] dev  : Structure instance of bme280_dev
 *
 * return Result of API execution status.
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_set_regs(uint8_t* reg_addr, const uint8_t* reg_data, uint8_t len,
                       struct bme280_dev* dev);

/**
 * This API reads the data from the given register address of sensor.
 *
 * param[in] reg_addr  : Register address from where the data to be read
 * param[out] reg_data : Pointer to data buffer to store the read data.
 * param[in] len       : No of bytes of data to be read.
 * param[in,out] dev   : Structure instance of bme280_dev.
 *
 * return Result of API execution status.
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t* reg_data, uint16_t len, struct bme280_dev* dev);

/**
 * Generic API for accessing sensor settings
 */

/**
 * This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 *
 * param[in] dev : Structure instance of bme280_dev.
 * param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros                 |   Functionality
 * -----------------------|----------------------------------------------
 * BME280_OSR_PRESS_SEL   |   To set pressure oversampling.
 * BME280_OSR_TEMP_SEL    |   To set temperature oversampling.
 * BME280_OSR_HUM_SEL     |   To set humidity oversampling.
 * BME280_FILTER_SEL      |   To set filter setting.
 * BME280_STANDBY_SEL     |   To set standby duration setting.
 *
 * return Result of API execution status
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_set_sensor_settings(uint8_t desired_settings, struct bme280_dev* dev);

/**
 * This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 *
 * param[in,out] dev : Structure instance of bme280_dev.
 *
 * return Result of API execution status
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_settings(struct bme280_dev* dev);

/**
 * Generic API for configuring sensor power mode
 */

/**
 * This API sets the power mode of the sensor.
 *
 * param[in] dev : Structure instance of bme280_dev.
 * param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 * sensor_mode          |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * return Result of API execution status
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, struct bme280_dev* dev);

/**
 * This API gets the power mode of the sensor.
 *
 * param[in] dev : Structure instance of bme280_dev.
 * param[out] sensor_mode : Pointer variable to store the power mode.
 *
 * sensor_mode          |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * return Result of API execution status
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_mode(uint8_t* sensor_mode, struct bme280_dev* dev);

/**
 * API that performs system-level operations
 */

/**
 * This API soft-resets the sensor.
 *
 * param[in,out] dev : Structure instance of bme280_dev.
 *
 * return Result of API execution status.
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_soft_reset(struct bme280_dev* dev);

/**
 * Data processing of sensor
 */

/**
 * This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 *
 * param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BME280_PRESS
 *     2       | BME280_TEMP
 *     4       | BME280_HUM
 *     7       | BME280_ALL
 *
 * param[out] comp_data : Structure instance of bme280_data.
 * param[in] dev : Structure instance of bme280_dev.
 *
 * return Result of API execution status
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data* comp_data,
                              struct bme280_dev* dev);

/**
 *  This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 *
 *  param[in] reg_data     : Contains register data which needs to be parsed
 *  param[out] uncomp_data : Contains the uncompensated pressure, temperature
 *  and humidity data.
 *
 */
void bme280_parse_sensor_data(const uint8_t* reg_data, struct bme280_uncomp_data* uncomp_data);

/**
 * This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected by the
 * user.
 *
 * param[in] sensor_comp : Used to select pressure and/or temperature and/or
 * humidity.
 * param[in] uncomp_data : Contains the uncompensated pressure, temperature and
 * humidity data.
 * param[out] comp_data : Contains the compensated pressure and/or temperature
 * and/or humidity data.
 * param[in] calib_data : Pointer to the calibration data structure.
 *
 * return Result of API execution status.
 *
 * retval   0 -> Success.
 * retval > 0 -> Warning.
 * retval < 0 -> Fail.
 *
 */
int8_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data* uncomp_data,
                              struct bme280_data* comp_data, struct bme280_calib_data* calib_data);

#ifdef __cplusplus
}
#endif

#endif /* BME280_H_ */
