#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "def.h"

#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

enum HtpaRegister
{
    Configuration = 0x01,
    Status = 0x02,
    Trim1 = 0x03,
    Trim2 = 0x04,
    Trim3 = 0x05,
    Trim4 = 0x06,
    Trim5 = 0x07,
    Trim6 = 0x08,
    Trim7 = 0x09,
    ReadTop = 0x0A,
    ReadBottom = 0x0B
};

enum Block
{
    Block0 = 0b00,
    Block1 = 0b01,
    Block2 = 0b10,
    Block3 = 0b11,
};

uint8_t get_sensor_status();
void print_sensor_status();
void read_block(Block block, uint8_t *top_data, size_t top_len, uint8_t *bottom_data, size_t bottom_len);
void read_all(uint8_t *data, size_t len);

// STRUCT WITH ALL SENSOR CHARACTERISTICS
struct characteristics
{
    unsigned short NumberOfPixel;
    unsigned char NumberOfBlocks;
    unsigned char RowPerBlock;
    unsigned short PixelPerBlock;
    unsigned short PixelPerColumn;
    unsigned short PixelPerRow;
    unsigned char AllowedDeadPix;
    unsigned short TableNumber;
    unsigned short TableOffset;
    unsigned char PTATPos;
    unsigned char VDDPos;
    unsigned char PTATVDDSwitch;
    unsigned char CyclopsActive;
    unsigned char CyclopsPos;
    unsigned char DataPos;
};

characteristics DevConst = {
    NUMBER_OF_PIXEL,
    NUMBER_OF_BLOCKS,
    ROW_PER_BLOCK,
    PIXEL_PER_BLOCK,
    PIXEL_PER_COLUMN,
    PIXEL_PER_ROW,
    ALLOWED_DEADPIX,
    TABLENUMBER,
    TABLEOFFSET,
    PTAT_POS,
    VDD_POS,
    PTAT_VDD_SWITCH,
    ATC_ACTIVE,
    ATC_POS,
    DATA_POS,
};

// EEPROM data
uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, mbit_user, bias_user, clk_user, bpa_user, pu_user;
uint8_t gradscale, vddscgrad, vddscoff, epsilon;

int8_t globaloff;

uint8_t globalgain;

float pixcmin, pixcmax;
// use a heap allocated memory to store the pixc instead of a nxm array
// ^thats not true anymore
unsigned long pixc2_0[NUMBER_OF_PIXEL * 4]; // start address of the allocated heap memory
unsigned long *pixc2;                       // increasing address pointer

// PROGRAM CONTROL
bool ReadingRoutineEnable = 1;

// OTHER
uint32_t gradscale_div, vddscgrad_div, vddscoff_div;

unsigned NewDataAvailable = 1;
unsigned short timert;

uint8_t read_EEPROM_byte(uint16_t address)
{
    uint8_t rdata = 0;
    uint8_t memory_address[2] = {
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xFF),
    };

    i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, memory_address, 2, true);
    i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, &rdata, 1, false);

    return rdata;
}

void read_eeprom()
{
    int m = 0;
    int n = 0;
    uint8_t b[4];
    mbit_calib = read_EEPROM_byte(E_MBIT_CALIB);
    bias_calib = read_EEPROM_byte(E_BIAS_CALIB);
    clk_calib = read_EEPROM_byte(E_CLK_CALIB);
    bpa_calib = read_EEPROM_byte(E_BPA_CALIB);
    pu_calib = read_EEPROM_byte(E_PU_CALIB);
    mbit_user = read_EEPROM_byte(E_MBIT_USER);
    bias_user = read_EEPROM_byte(E_BIAS_USER);
    clk_user = read_EEPROM_byte(E_CLK_USER);
    bpa_user = read_EEPROM_byte(E_BPA_USER);
    pu_user = read_EEPROM_byte(E_PU_USER);

    vddscgrad = read_EEPROM_byte(E_VDDSCGRAD);
    vddscoff = read_EEPROM_byte(E_VDDSCOFF);

    gradscale = read_EEPROM_byte(E_GRADSCALE);

    b[0] = read_EEPROM_byte(E_PIXCMIN_1);
    b[1] = read_EEPROM_byte(E_PIXCMIN_2);
    b[2] = read_EEPROM_byte(E_PIXCMIN_3);
    b[3] = read_EEPROM_byte(E_PIXCMIN_4);
    pixcmin = *(float *)b;
    b[0] = read_EEPROM_byte(E_PIXCMAX_1);
    b[1] = read_EEPROM_byte(E_PIXCMAX_2);
    b[2] = read_EEPROM_byte(E_PIXCMAX_3);
    b[3] = read_EEPROM_byte(E_PIXCMAX_4);
    pixcmax = *(float *)b;
    epsilon = read_EEPROM_byte(E_EPSILON);
    globaloff = read_EEPROM_byte(E_GLOBALOFF);
    globalgain = read_EEPROM_byte(E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(E_GLOBALGAIN_1);

    // --- Thgrad_ij, ThOffset_ij and P_ij ---
    m = 0;
    n = 0;
    pixc2 = pixc2_0; // set pointer to start address of the allocated heap // reset pointer to initial address
    // top half
    for (int i = 0; i < (unsigned short)(DevConst.NumberOfPixel / 2); i++)
    {
        // thgrad[m][n] = read_EEPROM_byte(E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_THGRAD + 2 * i);
        // thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
        *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
        n++;
        if (n == DevConst.PixelPerRow)
        {
            n = 0;
            m++; // !!!! forwards !!!!
        }
    }
    // bottom half
    m = (unsigned char)(DevConst.PixelPerColumn - 1);
    n = 0;
    for (int i = (unsigned short)(DevConst.NumberOfPixel / 2); i < (unsigned short)(DevConst.NumberOfPixel); i++)
    {
        // thgrad[m][n] = read_EEPROM_byte(E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_THGRAD + 2 * i);
        // thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
        *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
        n++;

        if (n == DevConst.PixelPerRow)
        {
            n = 0;
            m--; // !!!! backwards !!!!
        }
    }
}

void write_sensor_byte(uint8_t device_address, uint8_t register_address, uint8_t input)
{
    uint8_t cmd[2] = {register_address, input};
    i2c_write_blocking(I2C_PORT, device_address, cmd, 2, false);
}

void write_calibration_settings_to_sensor()
{
    write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_calib);
    sleep_ms(5);
    write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_calib);
    sleep_ms(5);
    write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_calib);
    sleep_ms(5);
    write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_calib);
    sleep_ms(5);
    write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_calib);
    sleep_ms(5);
    write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_calib);
    sleep_ms(5);
    write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_calib);
    sleep_ms(5);
}

void calcPixC()
{
    /* uses the formula from datasheet:

                       PixC_uns[m][n]*(PixCmax-PixCmin)               epsilon   GlobalGain
        PixC[m][n] = ( -------------------------------- + PixCmin ) * ------- * ----------
                                    65535                               100        1000
    */
    double pixcij;
    pixc2 = pixc2_0; // set pointer to start address of the allocated heap

    for (int m = 0; m < DevConst.PixelPerColumn; m++)
    {
        for (int n = 0; n < DevConst.PixelPerRow; n++)
        {
            pixcij = (double)pixcmax;
            pixcij -= (double)pixcmin;
            pixcij /= (double)65535.0;
            pixcij *= (double)*pixc2;
            pixcij += (double)pixcmin;
            pixcij /= (double)100.0;
            pixcij *= (double)epsilon;
            pixcij /= (double)10000.0;
            pixcij *= (double)globalgain;
            pixcij += 0.5;

            *pixc2 = (unsigned long)pixcij;
            pixc2++;
        }
    }
}

uint16_t calc_timert(uint8_t clk, uint8_t mbit)
{
    float a;
    uint16_t calculated_timer_duration;

    float Fclk_float = 12000000.0 / 63.0 * (float)clk + 1000000.0; // calc clk in Hz
    a = 32.0 * ((float)pow(2, (unsigned char)(mbit & 0b00001111)) + 4.0) / Fclk_float;

    calculated_timer_duration = (unsigned short)(0.98 * a * 1000000); // c in s | timer_duration in µs
    return calculated_timer_duration;
}

void timer_callback()
{
    if (ReadingRoutineEnable)
    {
        NewDataAvailable = 1;
    }
}

int main()
{
    stdio_init_all();
    getchar(); // blocking program so i can open serial monitor and get output

    // TODO: CHECK IF 400KHz WORKS FOR BOTH DEVICES
    i2c_init(I2C_PORT, CLOCK_SENSOR);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // reading whole EEPROM
    // TODO: TEST THAT SHIT
    read_eeprom();

    // to wake up sensor set configuration register to 0x01
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);
    sleep_ms(30);

    // write the calibration settings into the trim registers
    write_calibration_settings_to_sensor();

    // to start sensor set configuration register to 0x09
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);
    sleep_ms(30);
    printf("HTPAd is ready\n");

    // do bigger calculations before main loop
    gradscale_div = pow(2, gradscale);
    vddscgrad_div = pow(2, vddscgrad);
    vddscoff_div = pow(2, vddscoff);
    calcPixC();

    // timer initialization
    timert = calc_timert(clk_calib, mbit_calib);
    repeating_timer_t timer;
    add_repeating_timer_us(-timert, [](repeating_timer_t *t)
                           {
                               timer_callback();
                               return true; // Return true to keep the timer repeating
                           },
                           nullptr, &timer);

    // Loopin time!!
    while (true)
    {
    }

    // Htpa32x32d configuration
    {
        uint8_t cmd[2];
        uint8_t trims[] = {0x0C, 0x0C, 0x0C, 0x14, 0x0C, 0x0C, 0x88};
        for (size_t i = 0; i < 7; i++)
        {
            cmd[0] = HtpaRegister::Trim1 + i;
            cmd[1] = trims[i];
            i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, cmd, 2, false);
            sleep_ms(5);
        }
    }

    // Checking sensor status
    print_sensor_status();

    while (true)
    {
        char ch;
        ch = getchar();
        if (ch == 'q')
            break;

        // Start procedure
        {
            uint8_t cmd[2] = {HtpaRegister::Configuration, 0x09};
            i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, cmd, 2, false);
            sleep_ms(30);
        }

        // Checking sensor status
        print_sensor_status(); // should be 0x01

        // Reading data
        uint8_t data[2048] = {0};
        read_all(data, 2048);

        for (size_t i = 0; i < 2048; i++)
        {
            if (i % 64 == 0 && i != 0)
                printf("\n");
            printf("%02X", data[i]);
        }
        printf("\n");
    }

    // Sleep sensor
    uint8_t cmd[2] = {HtpaRegister::Configuration, 0x00};
    i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, cmd, 2, false);
    sleep_ms(30);
    printf("\nSensor sleeped\n");
}

uint8_t get_sensor_status()
{
    uint8_t cmd = HtpaRegister::Status;
    uint8_t status;
    i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, &cmd, 1, true);
    i2c_read_blocking(I2C_PORT, HTPA_I2C_ADDR, &status, 1, false);
    sleep_ms(5);

    return status;
}

void print_sensor_status()
{
    printf("Sensor status: 0x%02X\n", get_sensor_status());
}

void read_block(Block block, uint8_t *top_data, size_t top_len, uint8_t *bottom_data, size_t bottom_len)
{
    if (top_len < 258 || bottom_len < 258)
        return;

    // setting up block
    {
        uint8_t cmd_data = (block << 4) | 0x09u;
        uint8_t cmd[2] = {HtpaRegister::Configuration, cmd_data};
        i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, cmd, 2, false);
        sleep_ms(30);
    }

    uint8_t top_cmd = HtpaRegister::ReadTop;
    uint8_t bottom_cmd = HtpaRegister::ReadBottom;
    i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, &top_cmd, 1, true);
    i2c_read_blocking(I2C_PORT, HTPA_I2C_ADDR, top_data, 258, false);
    i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, &bottom_cmd, 1, true);
    i2c_read_blocking(I2C_PORT, HTPA_I2C_ADDR, bottom_data, 258, false);
}

void cpy_array(uint8_t *src, uint8_t *dst, size_t len)
{
    for (size_t i = 0; i < len; i++)
        dst[i] = src[i];
}

void read_all(uint8_t *data, size_t len)
{
    if (len < 2048)
        return;

    uint8_t blk0top[258] = {0};
    uint8_t blk1top[258] = {0};
    uint8_t blk2top[258] = {0};
    uint8_t blk3top[258] = {0};
    uint8_t blk3bottom[258] = {0};
    uint8_t blk2bottom[258] = {0};
    uint8_t blk1bottom[258] = {0};
    uint8_t blk0bottom[258] = {0};

    uint8_t *data_raw[8] = {
        blk0top,
        blk1top,
        blk2top,
        blk3top,
        blk3bottom,
        blk2bottom,
        blk1bottom,
        blk0bottom,
    };

    read_block(Block::Block0, blk0top, 258, blk0bottom, 258);
    read_block(Block::Block1, blk1top, 258, blk1bottom, 258);
    read_block(Block::Block2, blk2top, 258, blk2bottom, 258);
    read_block(Block::Block3, blk3top, 258, blk3bottom, 258);

    cpy_array(blk0top + 2, data, 256);
    cpy_array(blk1top + 2, data + 256, 256);
    cpy_array(blk2top + 2, data + 512, 256);
    cpy_array(blk3top + 2, data + 768, 256);

    cpy_array(blk3bottom + 2 + 192, data + 1024, 64);
    cpy_array(blk3bottom + 2 + 128, data + 1024 + 64, 64);
    cpy_array(blk3bottom + 2 + 64, data + 1024 + 128, 64);
    cpy_array(blk3bottom + 2, data + 1024 + 192, 64);

    cpy_array(blk2bottom + 2 + 192, data + 1280, 64);
    cpy_array(blk2bottom + 2 + 128, data + 1280 + 64, 64);
    cpy_array(blk2bottom + 2 + 64, data + 1280 + 128, 64);
    cpy_array(blk2bottom + 2, data + 1280 + 192, 64);

    cpy_array(blk1bottom + 2 + 192, data + 1536, 64);
    cpy_array(blk1bottom + 2 + 128, data + 1536 + 64, 64);
    cpy_array(blk1bottom + 2 + 64, data + 1536 + 128, 64);
    cpy_array(blk1bottom + 2, data + 1536 + 192, 64);

    cpy_array(blk0bottom + 2 + 192, data + 1792, 64);
    cpy_array(blk0bottom + 2 + 128, data + 1792 + 64, 64);
    cpy_array(blk0bottom + 2 + 64, data + 1792 + 128, 64);
    cpy_array(blk0bottom + 2, data + 1792 + 192, 64);
}
