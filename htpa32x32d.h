#ifndef HTPA32X32D_H
#define HTPA32X32D_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

class Htpa32x32d
{
private:
	enum Register
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

public:
	Htpa32x32d();
	Htpa32x32d(i2c_inst_t *i2c, uint8_t addr_sensor, uint8_t addr_eeprom);

	bool is_woken_up() const;
	int sleep();
	int wakeup_and_write_config();
	int get_sensor_status(uint8_t &status);
	// void start_measurement(uint8_t measurement);
	// void check_measurement_ready(uint8_t measurement, bool &ready);
	// void get_measurement_data(SensorHalf half, uint8_t *data, size_t size);

private:
	i2c_inst_t *i2c;
	uint8_t addr_sensor;
	uint8_t addr_eeprom;
	bool woken_up;
};

#endif