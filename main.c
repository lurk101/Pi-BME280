
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "bme280.h"

/* Structure that contains identifier details used in example */
struct identifier {
    /* Variable to hold device address */
    uint8_t dev_addr;
    /* Variable that contains file descriptor */
    int8_t fd;
};

/**
 * Function that creates a mandatory delay required in some of the APIs.
 *
 * param[in] period              : Delay in microseconds.
 * param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 * return void.
 *
 */
void user_delay_us(uint32_t period, void* intf_ptr);

/**
 * Function for print the temperature, humidity and pressure data.
 *
 * param[out] comp_data    :   Structure instance of bme280_data
 *
 * note Sensor data whose can be read
 *
 * sens_list
 * --------------
 * Pressure
 * Temperature
 * Humidity
 *
 */
void print_sensor_data(struct bme280_data* comp_data);

/**
 *  Function for reading the sensor's registers through I2C bus.
 *
 *  param[in] reg_addr       : Register address.
 *  param[out] data          : Pointer to the data buffer to store the read data.
 *  param[in] len            : No of bytes to read.
 *  param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  return Status of execution
 *
 *  retval 0 -> Success
 *  retval > 0 -> Failure Info
 *
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t* data, uint32_t len, void* intf_ptr);

/**
 *  Function for writing the sensor's registers through I2C bus.
 *
 *  param[in] reg_addr       : Register address.
 *  param[in] data           : Pointer to the data buffer whose value is to be written.
 *  param[in] len            : No of bytes to write.
 *  param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 *  return Status of execution
 *
 *  retval BME280_OK -> Success
 *  retval BME280_E_COMM_FAIL -> Communication failure.
 *
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t* data, uint32_t len, void* intf_ptr);

/**
 * Function reads temperature, humidity and pressure data in forced mode.
 *
 * param[in] dev   :   Structure instance of bme280_dev.
 *
 * return Result of API execution status
 *
 * retval BME280_OK - Success.
 * retval BME280_E_NULL_PTR - Error: Null pointer error
 * retval BME280_E_COMM_FAIL - Error: Communication fail error
 * retval BME280_E_NVM_COPY_FAILED - Error: NVM copy failed
 *
 */
int8_t stream_sensor_data_forced_mode(struct bme280_dev* dev);

/**
 * This function starts execution of the program.
 */
int main(int argc, char* argv[]) {
    struct bme280_dev dev;
    struct identifier id;

    /* Variable to define the result */
    int8_t rslt = BME280_OK;

    if (argc < 2) {
        fprintf(stderr, "Missing argument for i2c bus.\n");
        exit(1);
    }

    if ((id.fd = open(argv[1], O_RDWR)) < 0) {
        fprintf(stderr, "Failed to open the i2c bus %s\n", argv[1]);
        exit(1);
    }

    /* Make sure to select BME280_I2C_ADDR_PRIM or BME280_I2C_ADDR_SEC as needed */
    id.dev_addr = BME280_I2C_ADDR_PRIM;

    if (ioctl(id.fd, I2C_SLAVE, id.dev_addr) < 0) {
        fprintf(stderr, "Failed to acquire bus access and/or talk to slave.\n");
        exit(1);
    }

    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;

    /* Update interface pointer with the structure that contains both device address and file
     * descriptor */
    dev.intf_ptr = &id;

    /* Initialize the bme280 */
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
        exit(1);
    }

    rslt = stream_sensor_data_forced_mode(&dev);
    if (rslt != BME280_OK) {
        fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
        exit(1);
    }

    return 0;
}

/**
 * This function reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t* data, uint32_t len, void* intf_ptr) {
    struct identifier id;

    id = *((struct identifier*)intf_ptr);

    write(id.fd, &reg_addr, 1);
    read(id.fd, data, len);

    return BME280_INTF_RET_SUCCESS;
}

/**
 * This function provides the delay for required time (Microseconds) as per the input
 * provided in some of the APIs
 */
void user_delay_us(uint32_t period, void* intf_ptr) { usleep(period); }

/**
 * This function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t* data, uint32_t len, void* intf_ptr) {
    uint8_t* buf;
    struct identifier id;

    id = *((struct identifier*)intf_ptr);

    buf = malloc(len + 1);
    buf[0] = reg_addr;
    memcpy(buf + 1, data, len);
    if (write(id.fd, buf, len + 1) < (uint16_t)len)
        return BME280_E_COMM_FAIL;

    free(buf);

    return BME280_OK;
}

/**
 * This API used to print the sensor temperature, pressure and humidity data.
 */
void print_sensor_data(struct bme280_data* comp_data) {
    float temp, press, hum;

    temp = (comp_data->temperature / 100.0) * 1.8 + 32;
    press = comp_data->pressure / 10000.0;
    hum = comp_data->humidity / 1024.0;
    printf("%0.2lf deg F, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
}

/**
 * This API reads the sensor temperature, pressure and humidity data in forced mode.
 */
int8_t stream_sensor_data_forced_mode(struct bme280_dev* dev) {
    /* Variable to define the result */
    int8_t rslt = BME280_OK;

    /* Variable to define the selecting sensors */
    uint8_t settings_sel = 0;

    /* Structure to get the pressure, temperature and humidity values */
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_1X;
    dev->settings.osr_t = BME280_OVERSAMPLING_1X;
    dev->settings.filter = BME280_FILTER_COEFF_OFF;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL;

    /* Set the sensor settings */
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_OK) {
        fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);
        return rslt;
    }

    printf("Temperature, Pressure, Humidity\n");

    /*Calculate the minimum delay required between consecutive measurement based upon the sensor
     * enabled and the oversampling configuration. */

    /* Continuously stream sensor data */
    while (1) {
        /* Set the sensor to forced mode */
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        if (rslt != BME280_OK) {
            fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
            break;
        }

        /* Wait for the measurement to complete and print data */
        dev->delay_us(5000000, dev->intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if (rslt != BME280_OK) {
            fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
            break;
        }

        print_sensor_data(&comp_data);
    }

    return rslt;
}
