#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

#define HTPA_I2C_ADDR _u(0x1A)
#define HTPA_I2C_CLK 400000

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

int main()
{
    stdio_init_all();
    getchar(); // blocking program so i can open serial monitor and get output

    i2c_init(I2C_PORT, HTPA_I2C_CLK);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Htpa32x32d wakeup
    {
        uint8_t cmd[2] = {HtpaRegister::Configuration, 0x01};
        i2c_write_blocking(I2C_PORT, HTPA_I2C_ADDR, cmd, 2, false);
        sleep_ms(30);
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
