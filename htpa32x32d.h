class Htpa32x32d
{
private:
	i2c_inst_t *i2c = i2c0;
	uint8_t addr_sensor = 0x1A;
	uint8_t addr_eeprom = 0x50;
	bool woken_up = false;

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
		ReadBottom = 0x0B,
	};

	enum SensorHalf
	{
		Top = 0x0A,
		Bottom = 0x0B,
	};

	void send_cmd(uint8_t cmd, uint8_t data);

public:
	Htpa32x32d() {}
	Htpa32x32d(i2c_inst_t *i2c, uint8_t addr_sensor, uint8_t addr_eeprom)
		: i2c{i2c}, addr_sensor{addr_sensor}, addr_eeprom{addr_eeprom} {}

	uint8_t get_status();
	bool is_woken_up();
	void wake_up_and_write_config();
	void get_measurement_data(SensorHalf half, uint8_t data[258]);
};