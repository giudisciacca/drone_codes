# include <stdio.h>
# include "pico/stdlib.h"
# include "hardware/spi.h"
# include "hardware/i2c.h"
# include "hardware/dma.h"
# include "hardware/pio.h"
# include "hardware/interp.h"
# include "hardware/timer.h"
# include "hardware/watchdog.h"
# include "hardware/clocks.h"
# include "hardware/uart.h"
# include "blink.pio.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5


struct gyro_data {
    double x;
    double y;
    double z;
};

struct accel_data {
    double x;
    double y;
    double z;
};

struct magnetometer_data {
    double x;
    double y;
    double z;
};

struct imu_data {
    struct gyro_data gyro;
    struct accel_data accel;
};

struct imc29048_data {
    uint32_t timestamp; // Timestamp in milliseconds
    struct imu_data imu; // Gyroscope data
    struct accel_data accel; // Accelerometer data
    struct magnetometer_data mag; // Magnetometer data
};


double scale_gyro = 0.015267*( M_PI / 180.0); 
double scale_accel = 0.000061* 9.80665;
double scale_mag = 0.15;

class IMU {
public:
    IMU() {
        // Constructor: Initialize the IMU
        // Example: Read IMU data over I2C and print values
        initialise_imu();
    };

    const uint8_t IMU_ADDR = 0x68; // ICM-29048 default I2C address (check your IMU's datasheet)
    const uint8_t PWR_MGMT_1_REG = 0x06; // Power management register (verify with your IMU's datasheet)
    uint8_t pwr_mgmt_data[2] = {PWR_MGMT_1_REG, 0x01}; // Example: set clock source, disable sleep
    uint8_t accel_gyro_reg = 0x2D; // Starting register for accel/gyro data (verify for your IMU)
    uint8_t imu_data[12] = {0};    // 6 axes, 2 bytes each

    
    ~IMU() {
        // Destructor: Cleanup if needed
    }

    void initialise_imu() {
        // This function initializes the IMU over I2C.
        // Implement initialization logic here
        printf("Initialising IMU\n");
    }   
    
    struct imu_data to_si_units(struct imu_data data) {
        // Convert raw IMU data to SI units (e.g., m/s^2 for acceleration, rad/s for angular velocity)
        // Implement conversion logic here
        struct imu_data si_data = {};
        si_data.accel.x = data.accel.x * scale_accel; // Example conversion factor
        si_data.accel.y = data.accel.y * scale_accel;
        si_data.accel.z = data.accel.z * scale_accel;
        si_data.gyro.x = data.gyro.x * scale_gyro; // Example conversion factor
        si_data.gyro.y = data.gyro.y * scale_gyro;
        si_data.gyro.z = data.gyro.z * scale_gyro;
        return si_data;
    }


    struct imu_data read_data() {
        // Read data from the IMU
        // Implement reading logic here
        // Wake up the IMU
        struct imu_data out_imu_data = {};
        int init_result = i2c_write_blocking(I2C_PORT, IMU_ADDR, pwr_mgmt_data, 2, false);

        if (init_result < 0) {
            printf("Failed to initialize ICM-29048\n");
        } else {
            printf("ICM-29048 initialized\n");
            // Read accelerometer and gyroscope data registers (example addresses, check your IMU's datasheet)

            // Write register address to IMU, then read data
            int write_result = i2c_write_blocking(I2C_PORT, IMU_ADDR, &accel_gyro_reg, 1, true);

            if (write_result < 0) {
                printf("I2C write to IMU data register failed\n");
            } else {
                int read_result = i2c_read_blocking(I2C_PORT, IMU_ADDR, imu_data, 12, false);
                if (read_result < 0) {
                    printf("I2C read from IMU failed\n");
                } else {
                    // Convert raw bytes to signed 16-bit values
                    int16_t accel_x = (imu_data[0] << 8) | imu_data[1];
                    int16_t accel_y = (imu_data[2] << 8) | imu_data[3];
                    int16_t accel_z = (imu_data[4] << 8) | imu_data[5];
                    int16_t gyro_x  = (imu_data[6] << 8) | imu_data[7];
                    int16_t gyro_y  = (imu_data[8] << 8) | imu_data[9];
                    int16_t gyro_z  = (imu_data[10] << 8) | imu_data[11];

                    
                    out_imu_data.gyro.x = static_cast<double>(gyro_x);
                    out_imu_data.gyro.y = static_cast<double>(gyro_y);
                    out_imu_data.gyro.z = static_cast<double>(gyro_z);
                    out_imu_data.accel.x = static_cast<double>(accel_x);
                    out_imu_data.accel.y = static_cast<double>(accel_y);
                    out_imu_data.accel.z = static_cast<double>(accel_z);
                }
            }
        }
        return to_si_units(out_imu_data);
    };

    void print_data(){
        struct imu_data out_imu_data = read_data();
        
        printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n",
               out_imu_data.accel.x, out_imu_data.accel.y, out_imu_data.accel.z,
               out_imu_data.gyro.x, out_imu_data.gyro.y, out_imu_data.gyro.z);
    };

};

class Magnetometer {
public:
    const uint8_t IMU_ADDR = 0x68; // ICM-29048 default I2C address (check your IMU's datasheet)
    const uint8_t MAG_ADDR = 0x0C; // Magnetometer address (
    Magnetometer() {
        // Constructor: Initialize the magnetometer
        initialise_magnetometer();
    }

    ~Magnetometer() {
        // Destructor: Cleanup if needed
    }
    void initialise_magnetometer() {
        // This function initializes the magnetometer over I2C.
        // Implement initialization logic here
        printf("Initialising Magnetometer\n");
    }
    struct magnetometer_data read_data() {
        // Read data from the magnetometer
        // Implement reading logic here
        // Switch to USER BANK 0
        struct magnetometer_data raw_data = {};
        uint8_t bank0[2] = {0x7F, 0x00};
        i2c_write_blocking(I2C_PORT, IMU_ADDR, bank0, 2, false);

        // Disable I2C master mode in USER_CTRL (0x03)
        uint8_t user_ctrl[2] = {0x03, 0x00};
        i2c_write_blocking(I2C_PORT, IMU_ADDR, user_ctrl, 2, false);

        // Enable BYPASS_EN in INT_PIN_CFG (0x0F)
        uint8_t int_pin_cfg[2] = {0x0F, 0x02};
        i2c_write_blocking(I2C_PORT, IMU_ADDR, int_pin_cfg, 2, false);

        sleep_ms(10); // Give time for bypass to take effect

        // Now you can try to access the magnetometer directly at address 0x0C
        const uint8_t MAG_ADDR = 0x0C;

        // Power down magnetometer
        uint8_t mag_ctrl2[2] = {0x31, 0x00};
        i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_ctrl2, 2, false);
        sleep_ms(10);

        // Power up magnetometer in single measurement mode (0x01)
        mag_ctrl2[1] = 0x01;
        i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_ctrl2, 2, false);
        sleep_ms(10);

        // Read ST1 register to check if data is ready
        uint8_t mag_st1_reg = 0x10; // ST1 register for AK09916
        uint8_t mag_st1 = 0;
        int mag_st1_write = i2c_write_blocking(I2C_PORT, MAG_ADDR, &mag_st1_reg, 1, true);
        if (mag_st1_write < 0) {
            printf("I2C write to magnetometer ST1 reg failed\n");
        } else {
            int mag_st1_read = i2c_read_blocking(I2C_PORT, MAG_ADDR, &mag_st1, 1, false);
            if (mag_st1_read < 0) {
                printf("I2C read from magnetometer ST1 failed\n");
            } else if (mag_st1 & 0x01) { // Data ready
                uint8_t mag_data_reg = 0x11; // HXL register
                uint8_t mag_data[6] = {0};
                int mag_data_write = i2c_write_blocking(I2C_PORT, MAG_ADDR, &mag_data_reg, 1, true);
                if (mag_data_write < 0) {
                    printf("I2C write to magnetometer data reg failed\n");
                } else {
                    int mag_data_read = i2c_read_blocking(I2C_PORT, MAG_ADDR, mag_data, 6, false);
                    if (mag_data_read < 0) {
                        printf("I2C read from magnetometer data failed\n");
                    } else {
                        raw_data.x = static_cast<double>( (mag_data[1] << 8) | mag_data[0]);
                        raw_data.y = static_cast<double>((mag_data[3] << 8) | mag_data[2]);
                        raw_data.z = static_cast<double>((mag_data[5] << 8) | mag_data[4]);
                        printf("Mag: X=%f Y=%f Z=%f\n", raw_data.x, raw_data.y, raw_data.z);
                    }
                }
            } else {
                printf("Magnetometer data not ready\n");
            }
        }
        
        
        return to_si_units(raw_data);
    }

    void print_data() {
        // Print magnetometer data to console
        // Implement printing logic here
    }
    struct magnetometer_data to_si_units(struct magnetometer_data raw_data) {
        // Convert raw magnetometer data to SI units (e.g., microteslas)
        // Implement conversion logic here
        struct magnetometer_data si_data = {};
        si_data.x = raw_data.x * scale_mag; // Example conversion factor
        si_data.y = raw_data.y * scale_mag;
        si_data.z = raw_data.z * scale_mag;
        return si_data;
    };
};

class IMC29048 {
public: 
    IMC29048() {
        // Constructor: Initialize the IMC-29048
    }

    Magnetometer magnetometer; // Instance of Magnetometer class
    IMU imu; // Instance of IMU class


    ~IMC29048() {
        // Destructor: Cleanup if needed
    }

    void set_pins(){
        stdio_init_all();
        sleep_ms(2000); 
        printf("Initialise IMU\n");
        gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA);
        gpio_pull_up(I2C_SCL);
        i2c_init(I2C_PORT, 400*1000);
    }
    void initialise_sensors(){
        // This function initializes the ICM-20948) over I2C.
        set_pins();
        magnetometer.initialise_magnetometer();
        imu.initialise_imu();
    }

    struct imc29048_data read_data() {
        // Read data from the IMC-29048
        // Implement reading logic here
        struct imc29048_data out_imc29048_data = {};

        out_imc29048_data.timestamp = to_ms_since_boot(get_absolute_time());
        out_imc29048_data.imu = imu.read_data();
        out_imc29048_data.mag = magnetometer.read_data();

        return out_imc29048_data;
    }

    void print_data() {
        struct imc29048_data data = read_data();
        printf("Timestamp: %u ms\n", data.timestamp);  
        printf("IMU Data - Gyro: X=%f Y=%f Z=%f | Accel: X=%f Y=%f Z=%f\n",
               data.imu.gyro.x, data.imu.gyro.y, data.imu.gyro.z,
               data.imu.accel.x, data.imu.accel.y, data.imu.accel.z);
        printf("Magnetometer Data - X=%f Y=%f Z=%f\n",
               data.mag.x, data.mag.y, data.mag.z);

    }
};




