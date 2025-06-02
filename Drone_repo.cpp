#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

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

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}


int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}



// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5



int main()
{   int i;
    int flag;

    stdio_init_all();
    sleep_ms(2000); 
    /** 
    
    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi
    */

    // I2C Initialisation. Using it at 400Khz.
    printf("Before I2C init\n");
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    i2c_init(I2C_PORT, 400*1000);
    
    printf("After I2C init\n");
    
    // Example: Read IMU data over I2C and print values
    const uint8_t IMU_ADDR = 0x68; // ICM-29048 default I2C address (check your IMU's datasheet)
    const uint8_t PWR_MGMT_1_REG = 0x06; // Power management register (verify with your IMU's datasheet)
    uint8_t pwr_mgmt_data[2] = {PWR_MGMT_1_REG, 0x01}; // Example: set clock source, disable sleep

    
    
    //int init_result = 0;
    while (true) {
    // Wake up the IMU
    int init_result = i2c_write_blocking(I2C_PORT, IMU_ADDR, pwr_mgmt_data, 2, false);

    if (init_result < 0) {
        printf("Failed to initialize ICM-29048\n");
    } else {
        printf("ICM-29048 initialized\n");
        // Read accelerometer and gyroscope data registers (example addresses, check your IMU's datasheet)
        uint8_t accel_gyro_reg = 0x2D; // Starting register for accel/gyro data (verify for your IMU)
        uint8_t imu_data[12] = {0};    // 6 axes, 2 bytes each

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

            printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n",
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
            }
        }

        // Magnetometer (example: AK09916, check your IMU's datasheet for address and registers)
        const uint8_t MAG_ADDR = 0x0C; // Example I2C address for magnetometer
        uint8_t mag_reg = 0x10;        // Example: starting register for mag data
        uint8_t mag_data[6] = {0};

        //int mag_write = i2c_write_blocking(I2C_PORT, MAG_ADDR, &mag_reg, 1, true);
        //if (mag_write < 0) {
        //    printf("I2C write to magnetometer failed\n");
        //} else {
        //    int mag_read = i2c_read_blocking(I2C_PORT, MAG_ADDR, mag_data, 6, false);
        //    if (mag_read < 0) {
        //    printf("I2C read from magnetometer failed\n");
        //    } else {
        //    int16_t mag_x = (mag_data[1] << 8) | mag_data[0];
        //    int16_t mag_y = (mag_data[3] << 8) | mag_data[2];
        //    int16_t mag_z = (mag_data[5] << 8) | mag_data[4];
        //    printf("Mag: X=%d Y=%d Z=%d\n", mag_x, mag_y, mag_z);
        //    }
        //}

        // Barometer (example: BMP280, check your sensor's datasheet for address and registers)
        //const uint8_t BARO_ADDR = 0x76; // Example I2C address for barometer
        //uint8_t baro_reg = 0xF7;        // Starting register for pressure/temp data
        //uint8_t baro_data[6] = {0};
//
        //int baro_write = i2c_write_blocking(I2C_PORT, BARO_ADDR, &baro_reg, 1, true);
        //if (baro_write < 0) {
        //    printf("I2C write to barometer failed\n");
        //} else {
        //    int baro_read = i2c_read_blocking(I2C_PORT, BARO_ADDR, baro_data, 6, false);
        //    if (baro_read < 0) {
        //    printf("I2C read from barometer failed\n");
        //    } else {
        //    // Combine bytes for raw pressure and temperature (20 bits each, see BMP280 datasheet)
        //    int32_t press_raw = ((baro_data[0] << 12) | (baro_data[1] << 4) | (baro_data[2] >> 4));
        //    int32_t temp_raw  = ((baro_data[3] << 12) | (baro_data[4] << 4) | (baro_data[5] >> 4));
        //    printf("Baro: Pressure(raw)=%ld Temp(raw)=%ld\n", press_raw, temp_raw);
        //    }
        //}
    }
    /*
    // Read WHO_AM_I register (replace with correct register for your IMU)
    const uint8_t WHO_AM_I_REG = 0x75;
    uint8_t reg = WHO_AM_I_REG;
    uint8_t value = 0;

    printf("Reading IMU WHO_AM_I register over I2C...\n");
    
    int result = i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg, 1, true);
    if (result < 0) {
        printf("I2C write to WHO_AM_I failed\n");
    } else {
        result = i2c_read_blocking(I2C_PORT, IMU_ADDR, &value, 1, false);
        if (result < 0) {
            printf("I2C read from WHO_AM_I failed\n");
        } else {
            printf("IMU WHO_AM_I: 0x%02x\n", value);
        }
}
*/
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    /** 
    // Get a free channel, panic() if there are none
    int chan = dma_claim_unused_channel(true);
    
    // 8 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.
    
    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    
    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        dst,           // The initial write address
        src,           // The initial read address
        count_of(src), // Number of transfers; in this case each is 1 byte.
        true           // Start immediately.
    );
    
    // We could choose to go and do something else whilst the DMA is doing its
    // thing. In this case the processor has nothing else to do, so we just
    // wait for the DMA to finish.
    dma_channel_wait_for_finish_blocking(chan);
    
    // The DMA has now copied our text from the transmit buffer (src) to the
    // receive buffer (dst), so we can print it out from there.
    puts(dst);

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // Interpolator example code
    interp_config cfg = interp_default_config();
    // Now use the various interpolator library functions for your use case
    // e.g. interp_config_clamp(&cfg, true);
    //      interp_config_shift(&cfg, 2);
    // Then set the config 
    interp_set_config(interp0, 0, &cfg);
    // For examples of interpolator use see https://github.com/raspberrypi/pico-examples/tree/master/interp

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    // Watchdog example code
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // Whatever action you may take if a watchdog caused a reboot
    }
    
    // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(100, 1);
    
    // You need to call this function at least more often than the 100ms in the enable call to prevent a reboot
    watchdog_update();

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART
    
    // Send out a string, with CR/LF conversions
    uart_puts(UART_ID, " Hello, UART!\n");
    
    // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart
    */
    
        //printf("Hello, world!\n");
        sleep_ms(5);
    }
}
