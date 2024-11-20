#include "htpa32x32d.h"

Htpa32x32d::Htpa32x32d() : Htpa32x32d(i2c0, 0x1A, 0x50) {}

Htpa32x32d::Htpa32x32d(i2c_inst_t *i2c, uint8_t addr_sensor, uint8_t addr_eeprom)
	: i2c{i2c}, addr_sensor{addr_sensor}, addr_eeprom{addr_eeprom}
{
}

bool Htpa32x32d::is_woken_up() const
{
	return woken_up;
}

int Htpa32x32d::sleep()
{
	woken_up = false;
	uint8_t cmd[] = {Register::Configuration, static_cast<uint8_t>(woken_up)};

	if (i2c_write_blocking(i2c, addr_sensor, cmd, 2, false) < 0)
		return -1;

	return 0;
}

int Htpa32x32d::wakeup_and_write_config()
{
	woken_up = true;

	uint8_t cmd[2];
	cmd[0] = Register::Configuration;
	cmd[1] = woken_up;

	if (i2c_write_blocking(i2c, addr_sensor, cmd, 2, false) < 0)
		return -1;
	sleep_ms(30);

	uint8_t trims[] = {0x0C, 0x0C, 0x0C, 0x14, 0x0C, 0x0C, 0x88};
	for (size_t i = 0; i < 7; ++i)
	{
		cmd[0] = Register::Trim1 + i;
		cmd[1] = trims[i];
		if (i2c_write_blocking(i2c, addr_sensor, cmd, 2, false) < 0)
			return -1;
		sleep_ms(5);
	}

	return 0;
}

int Htpa32x32d::get_sensor_status(uint8_t &status)
{
	uint8_t cmd = Register::Status;

	if (i2c_write_blocking(i2c, addr_sensor, &cmd, 1, true) < 0)
		return -1;

	if (i2c_read_blocking(i2c, addr_sensor, &status, 1, false) < 0)
		return -1;
}
