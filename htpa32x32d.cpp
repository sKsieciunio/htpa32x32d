#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "htpa32x32d.h"

void Htpa32x32d::send_cmd(uint8_t cmd, uint8_t data)
{
	uint8_t buf[2] = {cmd, data};
	i2c_write_blocking(i2c, addr_sensor, buf, 2, false);
}

void Htpa32x32d::wake_up_and_write_config()
{
	send_cmd(Register::Configuration, 0x01);
	sleep_ms(30);
	send_cmd(Register::Trim1, 0x0C);
	sleep_ms(5);
	send_cmd(Register::Trim2, 0x0C);
	sleep_ms(5);
	send_cmd(Register::Trim3, 0x0C);
	sleep_ms(5);
	send_cmd(Register::Trim4, 0x14);
	sleep_ms(5);
	send_cmd(Register::Trim5, 0x0C);
	sleep_ms(5);
	send_cmd(Register::Trim6, 0x0C);
	sleep_ms(5);
	send_cmd(Register::Trim7, 0x88);
	sleep_ms(5);
}

bool Htpa32x32d::is_woken_up()
{
	return woken_up;
}

uint8_t Htpa32x32d::get_status()
{
	uint8_t write_buf = 0x02;
	uint8_t read_buf;
	i2c_write_blocking(i2c, addr_sensor, &write_buf, 1, true);
	i2c_read_blocking(i2c, addr_sensor, &read_buf, 1, false);
	return read_buf;
}
