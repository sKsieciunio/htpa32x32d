#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "def.h"

#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

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

uint8_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, globalgain;

signed short thgrad[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
signed short thoffset[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
signed short vddcompgrad[ROW_PER_BLOCK * 2][PIXEL_PER_ROW]; // TODO:
signed short vddcompoff[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];  // TODO:

float ptatgr_float, ptatoff_float, pixcmin, pixcmax;
// use a heap allocated memory to store the pixc instead of a nxm array
// ^thats not true anymore
unsigned long pixc2_0[NUMBER_OF_PIXEL]; // start address of the allocated heap memory
unsigned long *pixc2;                   // increasing address pointer

// SENSOR DATA
uint16_t data_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
uint8_t RAMoutput[2 * NUMBER_OF_BLOCKS + 2][BLOCK_LENGTH];
/*
  RAMoutput is the place where the raw values are saved

  example, order for 80x64:
  RAMoutput[0][]... data from block 0 top
  RAMoutput[1][]... data from block 1 top
  RAMoutput[2][]... data from block 2 top
  RAMoutput[3][]... data from block 3 top
  RAMutput[4][]... electrical offset top
  RAMoutput[5][]... electrical offset bottom
  RAMoutput[6][]... data from block 3 bottom
  RAMoutput[7][]... data from block 2 bottom
  RAMoutput[8][]... data from block 1 bottom
  RAMoutput[9][]... data from block 0 bottom

*/
uint16_t eloffset[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
uint8_t statusreg;
uint16_t Ta, ptat_av_uint16, vdd_av_uint16;

// BUFFER for PTAT, VDD, and elOffsets
// PTAT:
uint16_t ptat_buffer[PTAT_BUFFER_SIZE];
uint8_t use_ptat_buffer = 0;
uint8_t ptat_i = 0;
// VDD:
uint16_t vdd_buffer[VDD_BUFFER_SIZE];
uint8_t use_vdd_buffer = 0;
uint8_t vdd_i = 0;
// electrical offsets:
uint8_t use_eloffsets_buffer = 0;
uint8_t new_offsets = 1;

// PROGRAM CONTROL
bool switch_ptat_vdd = 0;
uint16_t picnum = 0;
uint8_t state = 0;
uint8_t read_block_num = START_WITH_BLOCK;
uint8_t read_eloffset_next_pic = 0;
bool ReadingRoutineEnable = 1;

// OTHER
uint32_t gradscale_div, vddscgrad_div, vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;
uint8_t print_state = 0;

unsigned NewDataAvailable = 1;
unsigned short timert;
repeating_timer_t timer;

uint8_t get_sensor_status()
{
    uint8_t cmd = STATUS_REGISTER;
    uint8_t status;
    i2c_write_blocking(I2C_PORT, SENSOR_ADDRESS, &cmd, 1, true);
    i2c_read_blocking(I2C_PORT, SENSOR_ADDRESS, &status, 1, false);

    return status;
}

void print_sensor_status()
{
    printf("Sensor status: 0x%02X\n", get_sensor_status());
}

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
    vddth1 = read_EEPROM_byte(E_VDDTH1_2) << 8 | read_EEPROM_byte(E_VDDTH1_1);
    vddth2 = read_EEPROM_byte(E_VDDTH2_2) << 8 | read_EEPROM_byte(E_VDDTH2_1);
    vddscgrad = read_EEPROM_byte(E_VDDSCGRAD);
    vddscoff = read_EEPROM_byte(E_VDDSCOFF);
    ptatth1 = read_EEPROM_byte(E_PTATTH1_2) << 8 | read_EEPROM_byte(E_PTATTH1_1);
    ptatth2 = read_EEPROM_byte(E_PTATTH2_2) << 8 | read_EEPROM_byte(E_PTATTH2_1);

    gradscale = read_EEPROM_byte(E_GRADSCALE);
    tablenumber = read_EEPROM_byte(E_TABLENUMBER2) << 8 | read_EEPROM_byte(E_TABLENUMBER1);

    b[0] = read_EEPROM_byte(E_PTATGR_1);
    b[1] = read_EEPROM_byte(E_PTATGR_2);
    b[2] = read_EEPROM_byte(E_PTATGR_3);
    b[3] = read_EEPROM_byte(E_PTATGR_4);
    ptatgr_float = *(float *)b;
    b[0] = read_EEPROM_byte(E_PTATOFF_1);
    b[1] = read_EEPROM_byte(E_PTATOFF_2);
    b[2] = read_EEPROM_byte(E_PTATOFF_3);
    b[3] = read_EEPROM_byte(E_PTATOFF_4);
    ptatoff_float = *(float *)b;
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
        thgrad[m][n] = read_EEPROM_byte(E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_THGRAD + 2 * i);
        thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
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
        thgrad[m][n] = read_EEPROM_byte(E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_THGRAD + 2 * i);
        thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(E_THOFFSET + 2 * i);
        *(pixc2 + m * DevConst.PixelPerRow + n) = read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
        n++;

        if (n == DevConst.PixelPerRow)
        {
            n = 0;
            m--; // !!!! backwards !!!!
        }
    }

    //---VddCompGrad and VddCompOff---
    // top half
    m = 0;
    n = 0;
    // top half
    for (int i = 0; i < (unsigned short)(DevConst.PixelPerBlock); i++)
    {
        vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
        vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
        n++;
        if (n == DevConst.PixelPerRow)
        {
            n = 0;
            m++; // !!!! forwards !!!!
        }
    }
    // bottom half
    m = (unsigned char)(DevConst.RowPerBlock * 2 - 1);
    n = 0;
    for (int i = (unsigned short)(DevConst.PixelPerBlock); i < (unsigned short)(DevConst.PixelPerBlock * 2); i++)
    {
        vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
        vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
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

bool timer_callback(repeating_timer_t *rt)
{
    static int count = 0;
    // printf("timer fired! count: %d\n", ++count);
    if (ReadingRoutineEnable)
    {
        NewDataAvailable = 1;
    }
    return true;
}

void read_sensor_register(uint8_t addr, uint8_t *dest, uint16_t n)
{
    i2c_write_blocking(I2C_PORT, SENSOR_ADDRESS, &addr, 1, true);
    i2c_read_blocking(I2C_PORT, SENSOR_ADDRESS, dest, n, false);
}

void readblockinterrupt()
{
    unsigned char bottomblock;

    ReadingRoutineEnable = 0;
    cancel_repeating_timer(&timer);

    // check EOC bit
    read_sensor_register(STATUS_REGISTER, &statusreg, 1);
    while (statusreg & 0x01 == 0)
    {
        read_sensor_register(STATUS_REGISTER, &statusreg, 1);
    }

    read_sensor_register(TOP_HALF, (uint8_t *)&RAMoutput[read_block_num], BLOCK_LENGTH);
    bottomblock = (uint8_t)((uint8_t)(NUMBER_OF_BLOCKS + 1) * 2 - read_block_num - 1);
    read_sensor_register(BOTTOM_HALF, (uint8_t *)&RAMoutput[bottomblock], BLOCK_LENGTH);

    read_block_num++;

    if (read_block_num < NUMBER_OF_BLOCKS)
    {
        write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (uint8_t)(0x09 + (0x10 * read_block_num) + (0x04 * switch_ptat_vdd)));
    }
    else
    {
        if (read_eloffset_next_pic)
        {
            read_eloffset_next_pic = 0;

            // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
            // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
            write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x0B + (0x04 * switch_ptat_vdd)));
            new_offsets = 1;
        }
        else
        {
            if (picnum > 1)
                state = 1;
            picnum++;

            if ((uint8_t)(picnum % READ_ELOFFSET_EVERYX) == 0)
                read_eloffset_next_pic = 1;

            if (DevConst.PTATVDDSwitch)
                switch_ptat_vdd ^= 1;

            read_block_num = 0;

            // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
            // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
            write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, (unsigned char)(0x09 + (0x04 * switch_ptat_vdd)));
        }
    }

    add_repeating_timer_us(-timert, timer_callback, nullptr, &timer);
    ReadingRoutineEnable = 1;
}

void sort_data()
{

    unsigned long sum = 0;
    unsigned short pos = 0;

    for (int m = 0; m < DevConst.RowPerBlock; m++)
    {
        for (int n = 0; n < DevConst.PixelPerRow; n++)
        {

            /*
               for example: a normal line of RAMoutput for HTPAd80x64 looks like:
               RAMoutput[0][] = [ PTAT(MSB), PTAT(LSB), DATA0[MSB], DATA0[LSB], DATA1[MSB], DATA1[LSB], ... , DATA640[MSB], DATA640LSB];
                                                            |
                                                            |-- DATA_Pos = 2 (first data byte)
            */
            pos = (unsigned short)(2 * n + DevConst.DataPos + m * 2 * DevConst.PixelPerRow);

            /******************************************************************************************************************
              new PIXEL values
            ******************************************************************************************************************/
            for (int i = 0; i < DevConst.NumberOfBlocks; i++)
            {
                // top half
                data_pixel[m + i * DevConst.RowPerBlock][n] =
                    (unsigned short)(RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1]);
                // bottom half
                data_pixel[DevConst.PixelPerColumn - 1 - m - i * DevConst.RowPerBlock][n] =
                    (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos + 1]);
            }

            /******************************************************************************************************************
              new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
            ******************************************************************************************************************/
            if (picnum % ELOFFSETS_BUFFER_SIZE == 1)
            {
                if ((!eloffset[m][n]) || (picnum < ELOFFSETS_FILTER_START_DELAY))
                {
                    // top half
                    eloffset[m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
                    // bottom half
                    eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
                    use_eloffsets_buffer = 1;
                }
                else
                {
                    // use a moving average filter
                    // top half
                    sum = (unsigned long)eloffset[m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
                    sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
                    eloffset[m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
                    // bottom half
                    sum = (unsigned long)eloffset[2 * DevConst.RowPerBlock - 1 - m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
                    sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
                    eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
                }
            }
        }
    }

    /******************************************************************************************************************
      new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
    ******************************************************************************************************************/
    if (switch_ptat_vdd == 1)
    {
        sum = 0;
        // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
        for (int i = 0; i < DevConst.NumberOfBlocks; i++)
        {
            // block top half
            sum += (unsigned short)(RAMoutput[i][DevConst.PTATPos] << 8 | RAMoutput[i][DevConst.PTATPos + 1]);
            // block bottom half
            sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos + 1]);
        }
        ptat_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));
        Ta = (unsigned short)((unsigned short)ptat_av_uint16 * (float)ptatgr_float + (float)ptatoff_float);

        ptat_buffer[ptat_i] = ptat_av_uint16;
        ptat_i++;
        if (ptat_i == PTAT_BUFFER_SIZE)
        {
            if (use_ptat_buffer == 0)
            {
                // Serial.print(" | PTAT buffer complete");
                use_ptat_buffer = 1;
            }
            ptat_i = 0;
        }

        if (use_ptat_buffer)
        {
            // now overwrite the old ptat average
            sum = 0;
            for (int i = 0; i < PTAT_BUFFER_SIZE; i++)
            {
                sum += ptat_buffer[i];
            }
            ptat_av_uint16 = (uint16_t)((float)sum / PTAT_BUFFER_SIZE);
        }
    }

    /******************************************************************************************************************
      new VDD values (store them in VDD buffer and calculate the average for pixel compensation
    ******************************************************************************************************************/
    if (switch_ptat_vdd == 0)
    {
        sum = 0;
        // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
        for (int i = 0; i < DevConst.NumberOfBlocks; i++)
        {
            // block top half
            sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]);
            // block bottom half
            sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos + 1]);
        }
        vdd_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));

        // write into vdd buffer
        vdd_buffer[vdd_i] = vdd_av_uint16;
        vdd_i++;
        if (vdd_i == VDD_BUFFER_SIZE)
        {
            if (use_vdd_buffer == 0)
            {
                // Serial.print(" | VDD buffer complete");
                use_vdd_buffer = 1;
            }
            vdd_i = 0;
        }
        if (use_vdd_buffer)
        {
            sum = 0;
            for (int i = 0; i < VDD_BUFFER_SIZE; i++)
            {
                sum += vdd_buffer[i];
            }
            // now overwrite the old vdd average
            vdd_av_uint16 = (uint16_t)((float)sum / VDD_BUFFER_SIZE);
        }
    }
}

void calculate_pixel_temp()
{

    int64_t vij_pixc_and_pcscaleval;
    int64_t pixcij;
    int64_t vdd_calc_steps;
    uint16_t table_row, table_col;
    int32_t vx, vy, ydist, dta;
    signed long pixel;
    pixc2 = pixc2_0; // set pointer to start address of the allocated heap

    /******************************************************************************************************************
      step 0: find column of lookup table
    ******************************************************************************************************************/
    for (int i = 0; i < NROFTAELEMENTS; i++)
    {
        if (Ta > XTATemps[i])
        {
            table_col = i;
        }
    }
    dta = Ta - XTATemps[table_col];
    ydist = (int32_t)ADEQUIDISTANCE;

    for (int m = 0; m < DevConst.PixelPerColumn; m++)
    {
        for (int n = 0; n < DevConst.PixelPerRow; n++)
        {

            /******************************************************************************************************************
               step 1: use a variable with bigger data format for the compensation steps
             ******************************************************************************************************************/
            pixel = (signed long)data_pixel[m][n];

            /******************************************************************************************************************
               step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
             ******************************************************************************************************************/
            pixel -= (int32_t)(((int32_t)thgrad[m][n] * (int32_t)ptat_av_uint16) / (int32_t)gradscale_div);
            pixel -= (int32_t)thoffset[m][n];

            /******************************************************************************************************************
               step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
             ******************************************************************************************************************/
            if (m < DevConst.PixelPerColumn / 2)
            { // top half
                pixel -= eloffset[m % DevConst.RowPerBlock][n];
            }
            else
            { // bottom half
                pixel -= eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
            }

            /******************************************************************************************************************
               step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
             ******************************************************************************************************************/
            // first select VddCompGrad and VddCompOff for pixel m,n:
            if (m < DevConst.PixelPerColumn / 2)
            { // top half
                vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock][n];
                vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock][n];
            }
            else
            { // bottom half
                vddcompgrad_n = vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
                vddcompoff_n = vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
            }
            // now do the vdd calculation
            vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
            vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
            vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
            vdd_calc_steps = vdd_calc_steps * (vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16 - ptatth1));
            vdd_calc_steps = vdd_calc_steps / vddscoff_div;
            pixel -= vdd_calc_steps;

            /******************************************************************************************************************
               step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
             ******************************************************************************************************************/
            vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
            pixel = (int32_t)(vij_pixc_and_pcscaleval / *pixc2);
            pixc2++;
            /******************************************************************************************************************
               step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
             ******************************************************************************************************************/
            table_row = pixel + TABLEOFFSET;
            table_row = table_row >> ADEXPBITS;
            // bilinear interpolation
            vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
            vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
            pixel = (uint32_t)((vy - vx) * ((int32_t)(pixel + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

            /******************************************************************************************************************
               step 7: add GlobalOffset (stored as signed char)
             ******************************************************************************************************************/
            pixel += globaloff;

            /******************************************************************************************************************
              step 8: overwrite the uncompensate pixel with the new calculated compensated value
            ******************************************************************************************************************/
            data_pixel[m][n] = (unsigned short)pixel;
        }
    }

    /******************************************************************************************************************
      step 8: overwrite the uncompensate pixel with the new calculated compensated value
    ******************************************************************************************************************/
    // TODO:
    // pixel_masking();
}

void print_final_array(void)
{
    printf("\n\n---pixel data---\n");
    for (int m = 0; m < DevConst.PixelPerColumn; m++)
    {
        for (int n = 0; n < DevConst.PixelPerRow; n++)
        {
            printf("%04X", data_pixel[m][n]);
        }
        printf("\n");
    }
}

void print_RAM_array(void)
{
    printf("\n\n---pixel data ---\n");
    for (int m = 0; m < (2 * NUMBER_OF_BLOCKS + 2); m++)
    {
        for (int n = 0; n < BLOCK_LENGTH; n++)
        {
            printf("%02X", RAMoutput[m][n]);
        }
        printf("\n");
    }
}

void dump_whole_eeprom()
{
    printf("EEPROM from 0x0000 to 0x1F3F (useful data end)\n");
    for (uint16_t address = 0x0000; address < 0x1F40; address += 0x0010)
    {
        uint8_t rdata[16] = {0};
        uint8_t memory_address[2] = {
            static_cast<uint8_t>(address >> 8),
            static_cast<uint8_t>(address & 0xFF),
        };

        i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, memory_address, 2, true);
        i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, rdata, 16, false);

        for (int i = 0; i < 16; i++)
        {
            printf("%02X", rdata[i]);
        }
        printf("\n");
    }
}

int main()
{
    stdio_init_all();
    getchar(); // blocking program so i can open serial monitor and get output

    i2c_init(I2C_PORT, CLOCK_SENSOR);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // reading whole EEPROM
    // read_eeprom();
    dump_whole_eeprom();

    return 0;

    // to wake up sensor set configuration register to 0x01
    // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
    // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
    write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);
    sleep_ms(30);

    // write the calibration settings into the trim registers
    // write_calibration_settings_to_sensor();

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

    print_state = 1;

    // timer initialization
    timert = calc_timert(clk_calib, mbit_calib); // chyba około 25 ms
    printf("timert: %d\n", timert);
    add_repeating_timer_us(-timert, timer_callback, nullptr, &timer);

    // Loopin time!!
    while (true)
    {
        if (NewDataAvailable)
        {
            readblockinterrupt();
            NewDataAvailable = 0;
        }
        else
        {
            sleep_ms(1);
        }

        if (state)
        {
            sort_data();
            state = 0;

            if (print_state == 1)
            {
                calculate_pixel_temp();
                print_final_array();
            }

            if (print_state == 2)
            {
                print_RAM_array();
            }
        }
        else
        {
            sleep_ms(1);
        }
    }
}