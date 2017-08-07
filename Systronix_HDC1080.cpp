/****************************************************************************
	Systronix_HDC1080.cpp

Be sure to set stop "true" to release the bus or it will send a restart
So to read three bytes, do a requestFrom(address, 2, true). << insists the first param is int.
Compiler has major whines if called as shown in the online Wire reference.

	Author: bboyes

 Based on 2015-2016 TI data sheet

 Revisions

***************************************************************************/

/****************************************************************************
	DEVICE I2C BASE ADDRESS

 I2C has a 7-bit device address with bit 0 actually the R(H)/W(L) bit
 Right-shifting to not include the R/W bit, the range of base addresses is
 0b0001 1xxx which is 0x18-0x1F which is how Arduino refers to it.

 Some include the R/W bit which, IMO, makes the addresses more confusing,
 since each actual address has both a "read" and "write" version.

 NOTES ABOUT WIRE

Wire.endTransmission() seems to be only intended for use with a master write.
Wire.requestFrom(address, quantity, stop) is used to get bytes from a slave, with read().
 NOTE: We are actually using i2c_t3 optimized for Teensy 3.x but should
 still work with Wire on Teensy or other Arduinos

***************************************************************************/


#include <Systronix_HDC1080.h>


#define _DEBUG 0

//---------------------------< D E F A U L T   C O N S R U C T O R >------------------------------------------
//
// default constructor
//

Systronix_HDC1080::Systronix_HDC1080 (void)
	{
	error.total_error_count = 0;				// clear the error counter
	}


//---------------------------< D E S T R U C T O R >----------------------------------------------------------
//
// destructor
//

Systronix_HDC1080::~Systronix_HDC1080 (void)
{
	// Anything to do here? Leave I2C as master? Set flag?
}


//---------------------------< S E T U P >--------------------------------------------------------------------
/*!
    @brief  Instantiates a new HDC1080 class to use the given base address

	@param wire instance of i2c_t3 'object' Wire, Wire1, Wire2, Wire3
	@param name is a string such as Wire, Wire1, etc used for debug output,
		also saved as wire_name
	@TODO merge with begin()? This function doesn't actually do anything,
	it just sets some private values. It's redundant and some params must
	be effectively specified again in begin (Wire net and pins are not independent).
	@TODO there is no default of this using just the base, so the example program
	PCA9557_test_all doesn't work with this library version.
*/

uint8_t Systronix_HDC1080::setup(i2c_t3 wire, char* name)
	{
	_wire = wire;
	_wire_name = wire_name = name;		// protected and public
	return SUCCESS;
	}


/**************************************************************************/
/*!
    @brief  Join the I2C bus as a master, call this once in setup()

    @param pins must be from i2c_t3 enum i2c_pins
    @param rate speed in KHz, won't end up being exact. Use i2c_t3 enum i2c_rate
	@returns nothing, since Wire.begin() doesn't return anything

*/
/**************************************************************************/
void Systronix_HDC1080::begin(i2c_pins pins, i2c_rate rate)
	{
	_wire.begin(I2C_MASTER, 0x00, pins, I2C_PULLUP_EXT, rate);	// join I2C as master
	_wire.setDefaultTimeout(200000); // 200ms
	}


/**
	@ brief Void version of begin for backwards compatability, assumes Wire net

*/
void Systronix_HDC1080::begin(void)
	{
	_wire.begin();	// join I2C as master
	_wire.setDefaultTimeout(200000); // 200ms
	}


//---------------------------< B A S E _ G E T >--------------------------------------------------------------
//
//	returns the I2C base address for this instance
//
uint8_t Systronix_HDC1080::base_get(void)
	{
	return _base;
	}


//---------------------------< S E N S O R _ D A T A _ S E T >------------------------------------------------
//
// writes little endian arm uint16_t into _sensor_data union in big endian form used by HDC1080
//

void Systronix_HDC1080::sensor_data_set (uint16_t data)
	{
	_sensor_data.as_u16 = __builtin_bswap16 (data);
	}


//---------------------------< S E N S O R _ D A T A _ G E T >------------------------------------------------
//
// reads big endian uint16_t sensor data form used by HDC1080 from _sensor_data union and returns little endian arm uint16_t
//

uint16_t Systronix_HDC1080::sensor_data_get (void)
	{
	return __builtin_bswap16 (_sensor_data.as_u16);
	}


//---------------------------< I N I T >----------------------------------------------------------------------
/**
 *  @brief Initialize the HDC1080 to a given state. Can be called as often as needed.
 *
 *  Call after a hardware reset, if a reset can be caused programatically.
 *  @param config_reg settings
 *
 *  @return SUCCESS/0 if OK, ABSENT if could not even address the part, or FAIL if we could address it
 *  but then could not subsequently load its registers
 *
 *  @TODO maybe take out the inversion of config? To be consistent with the data sheet
 */

 uint8_t Systronix_HDC1080::init (uint16_t config, uint8_t initial_measure)
	{
	uint8_t ret_val;
	error.exists = true;					// so we can use config_write; we'll find out later if device does not exist

	Serial.printf("HDC1080 Lib init %s at base 0x%.2X\r\n", _wire_name, _base);

	ret_val = config_write (config);		// if successful this means we got two ACKs from slave device
	if (SUCCESS != ret_val)
		{
		Serial.printf("9557 lib init failed with %s (0x%.2X)\r\n", status_text[error.error_val], error.error_val);
		error.exists = false;			// only place error.exists is set false
		return ABSENT;
		}

	if (_config_reg & MODE_T_AND_H)			// if set for dual acquisition
		pointer_write (TRIGGER_T_H);		// trigger both temperature and humidity measurements
	else if (TRIGGER_T == initial_measure)	// individual measurements
		pointer_write (TRIGGER_T);			// do temperature
	else
		pointer_write (TRIGGER_H);			// do humidity

	return SUCCESS;
	}


//---------------------------< R E S E T _ B U S >------------------------------------------------------------
/**
	Invoke resetBus of whichever Wire net this class instance is using
	@return nothing
*/
void Systronix_HDC1080::reset_bus (void)
	{
	_wire.resetBus();
	}


//---------------------------< R E S E T _ B U S _ C O U N T _ R E A D >--------------------------------------
/**
	Return the resetBusCount of whichever Wire net this class instance is using
	@return number of Wire net resets, clips at UINT32_MAX
*/
uint32_t Systronix_HDC1080::reset_bus_count_read(void)
	{
	return _wire.resetBusCountRead();
	}


//---------------------------< T A L L Y _ T R A N S A C T I O N >--------------------------------------------
/**
Here we tally errors.  This does not answer the what-to-do-in-the-event-of-these-errors question; it just
counts them.

TODO: we should decide if the correct thing to do when slave does not ack, or arbitration is lost, or
timeout occurs, or auto reset fails (states 2, 5 and 4, 7 ??? state numbers may have changed since this
comment originally added) is to declare these addresses as non-existent.

We need to decide what to do when those conditions occur if we do not declare the device non-existent.
When a device is declared non-existent, what do we do then? (this last is more a question for the
application than this library).  The questions in this TODO apply equally to other i2c libraries that tally
these errors.

Don't set error.exists = false here! These errors are likely recoverable. bab & wsk 170612

This is the only place we set error.error_val()

TODO use i2c_t3 error or status enumeration here in the switch/case
*/

void Systronix_HDC1080::tally_transaction (uint8_t value)
	{
	if (value && (error.total_error_count < UINT64_MAX))
		error.total_error_count++; 			// every time here incr total error count

	error.error_val = value;

	switch (value)
		{
		case SUCCESS:
			if (error.successful_count < UINT64_MAX)
				error.successful_count++;
			break;
		case 1:								// i2c_t3 and Wire: data too long from endTransmission() (rx/tx buffers are 259 bytes - slave addr + 2 cmd bytes + 256 data)
			error.data_len_error_count++;
			break;
#if defined I2C_T3_H
		case I2C_TIMEOUT:
			error.timeout_count++;			// 4 from i2c_t3; timeout from call to status() (read)
#else
		case 4:
			error.other_error_count++;		// i2c_t3 and Wire: from endTransmission() "other error"
#endif
			break;
		case 2:								// i2c_t3 and Wire: from endTransmission()
		case I2C_ADDR_NAK:					// 5 from i2c_t3
			error.rcv_addr_nack_count++;
			break;
		case 3:								// i2c_t3 and Wire: from endTransmission()
		case I2C_DATA_NAK:					// 6 from i2c_t3
			error.rcv_data_nack_count++;
			break;
		case I2C_ARB_LOST:					// 7 from i2c_t3; arbitration lost from call to status() (read)
			error.arbitration_lost_count++;
			break;
		case I2C_BUF_OVF:
			error.buffer_overflow_count++;
			break;
		case I2C_SLAVE_TX:
		case I2C_SLAVE_RX:
			error.other_error_count++;		// 9 & 10 from i2c_t3; these are not errors, I think
			break;
		case WR_INCOMPLETE:					// 11; Wire.write failed to write all of the data to tx_buffer
			error.incomplete_write_count++;
			break;
		case SILLY_PROGRAMMER:				// 12
			error.silly_programmer_error++;
			break;
		default:
			error.unknown_error_count++;
			break;
		}
	}


//---------------------------< P O I N T E R _ W R I T E >----------------------------------------------------
//
// Write to the pointer register. This is the register accessed after the HDC1080 receives a valid slave
// address and ACKs it.  The next data written by the master sets the pointer register.  Note that the control
// register itself does not have a "register address".

// 0x00 read-only temperature measurement register
// 0x01 read-only humidiy measurement register
//
// 0x02 read/write configuration register
//
// 0xFB read-only serial ID (bytes 4 and 3)
// 0xFC read-only serial ID (bytes 2 and 1)
// 0xFD read-only serial ID (byte 0 and 0x00 in the LS byte)
// 0xFE read-only serial ID (bytes 4 and 3)
// 0xFF read-only serial ID (bytes 4 and 3)
//
//	All this function does is set the pointer register
//
//	@param target_register the HDC1080 register that we want to access
//	@return SUCCESS, FAIL, or ABSENT.
//	@side_effect if SUCCESS we update _pointer_reg
//

uint8_t Systronix_HDC1080::pointer_write (uint8_t target_register)
	{
	uint8_t ret_val;

	if (!error.exists)							// exit immediately if device does not exist
		return ABSENT;

	if ((target_register > HDC1080_CONFIG_REG) && (target_register < HDC1080_ID_43_REG))
		tally_transaction(SILLY_PROGRAMMER);	// target_register does not specify a valid register

	_wire.beginTransmission (_base);
	ret_val = _wire.write (target_register);	// returns # of bytes written to i2c_t3 buffer
	if (1 != ret_val)
		{
		tally_transaction (WR_INCOMPLETE);		// increment the appropriate counter
		return FAIL;							// calling function decides what to do with the error
		}

	ret_val = _wire.endTransmission ();			// endTransmission() returns 0 if successful
  	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);			// increment the appropriate counter
		return FAIL;							// calling function decides what to do with the error
		}

	_pointer_reg = target_register;				// remember where the pointer register is pointing

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< C O N F I G _ W R I T E >------------------------------------------------------
//
//	Write configuration to the configuration register
//	@param data
//
//	returns SUCCESS, FAIL, or ABSENT
//

uint8_t Systronix_HDC1080::config_write (uint16_t data)
	{
	uint8_t ret_val;

	if (!error.exists)									// exit immediately if device does not exist
		return ABSENT;

	sensor_data_set (data);								// reorder bytes from arm little endian to HDC1080 big endian

	_wire.beginTransmission (_base);
	ret_val = _wire.write (HDC1080_CONFIG_REG);			// write pointer register value
	ret_val += _wire.write (_sensor_data.as_array, 2);	// and write the data to the tx_buffer
	if (3 != ret_val)
		{
		tally_transaction (WR_INCOMPLETE);				// increment the appropriate counter
		return FAIL;									// calling function decides what to do with the error
		}

	ret_val = _wire.endTransmission();
  	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);					// increment the appropriate counter
		return FAIL;									// calling function decides what to do with the error
		}

	_pointer_reg = HDC1080_CONFIG_REG;					// remember where the pointer register is pointing

	_config_reg = data;									// remember how we have set the configuration register

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< R E G I S T E R _ R E A D >------------------------------------------------------
//
// Read a uint16_t from target register.  For reading a single register only; use get_data() to read both
// temperature and humidity in a single operation.
//
// local register and register contents tracking variables are updated in the functions called from here, so we
// don't need to do that here. So are any error values.
//
// @note after a register_read you can do additional default_reads at the same target_register.
//
// @param target_register
// @param data_ptr
//
// returns SUCCESS, FAIL, or ABSENT
//

uint8_t Systronix_HDC1080::register_read (uint8_t target_register, uint16_t* data_ptr)
{

	if (!error.exists)							// exit immediately if device does not exist
		return ABSENT;

	if (pointer_write (target_register))		// put address of target_register into the pointer register
		return FAIL;

	if (default_read (data_ptr))				// read the target_register
		return FAIL;

	// the actual functions above update successful_count so don't duplicate that here
	return SUCCESS;
}


//---------------------------< D E F A U L T _ R E A D >------------------------------------------------------
//
// Enter this function with _pointer_reg already set to desired measurement, config or ID register
//
// Reads one or two uint16_t values from whatever register is currently pointed to by the pointer register, the
// value of which is held in our local private _pointer_reg.
//
// When MODE bit is 1, and h_data_ptr is not NULL, and _pointer_reg points to HDC1080_TEMP_REG, this function
// reads two uint16_t values from HDC1080.  The first is always temperature, the second always humidity.
//
// When MODE bit is 0, h_data_ptr is ignored (defaults to NULL)
//
// @param *data_ptr is pointer to a uint16_t where the read data will be written
//
//	returns SUCCESS, FAIL, SILLY_PROGRAMMER, or ABSENT
//

uint8_t Systronix_HDC1080::default_read (uint16_t* data_ptr, uint16_t* h_data_ptr)
	{
	uint8_t ret_val;
	size_t	read_len;

	if (!error.exists)									// exit immediately if device does not exist
		return ABSENT;

	if ((_config_reg & MODE_T_AND_H) && _pointer_reg == HDC1080_TEMP_REG)	// both temp and humidity acquisition mode and reading measurements
		{
		if (h_data_ptr)									// must point somewhere other than NULL
			read_len = sizeof (uint16_t) << 1;			// read both measurements in one operation
		else											// forgot to set h_data_ptr?
			{
			tally_transaction (SILLY_PROGRAMMER);
			return FAIL;
			}
		}
	else												// here when reading only one of the measurement registers
		read_len = sizeof (uint16_t);					// or a non-measurement register (selected by preceding pointer write)

	if (read_len != _wire.requestFrom (_base, read_len, I2C_STOP))
		{
		ret_val = _wire.status();						// to get error value
		tally_transaction (ret_val);					// increment the appropriate counter
		return FAIL;
		}

	for (uint8_t i=0; i<sizeof(uint16_t); i++)			// get temp (if dual) or selected measurement (if individual) or register
		_sensor_data.as_array[i] = _wire.readByte();	// and write to _sensor_data union

	*data_ptr = sensor_data_get ();						// reorder bytes from HDC1080 big endian to arm little endian

	if (read_len > sizeof (uint16_t))
		{
		for (uint8_t i=sizeof(uint16_t), j=0; i<read_len; i++, j++)	// get humidity into _sensor_data union for byte reordering
			_sensor_data.as_array[j] = _wire.readByte();

		*h_data_ptr = sensor_data_get ();				// reordered bytes from HDC1080 big endian to arm little endian
		}

	if (HDC1080_TEMP_REG == _pointer_reg)				// update our remembered reg value
		{												// don't bother remembering the serial ID, manufacturer, device ID
		_temp_reg = *data_ptr;							// temperature
		if ((_config_reg & MODE_T_AND_H))				// and if dual acquisition mode
			_humidity_reg = *h_data_ptr;				// humidity
		}
	else if (HDC1080_HUMIDITY_REG == _pointer_reg)		// just humidity
		_humidity_reg = *data_ptr;
	else if (HDC1080_CONFIG_REG == _pointer_reg)
		_config_reg = *data_ptr;

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< R A W _ 1 4 _ T O _ R H >------------------------------------------------------
//
// Convert raw14 (uint16_t) relative humidity measurement to float
//

float Systronix_HDC1080::raw14_to_rh (uint16_t raw14)
	{
	return (raw14/65536.0) * 100.0;
	}


//---------------------------< R A W _ 1 4 _ T O _ C >--------------------------------------------------------
//
// Convert raw14 uint16_t to float as degrees C
//

float Systronix_HDC1080::raw14_to_c (uint16_t raw14)
	{
	return ((raw14/65536.0) * 165.0) -40.0;
	}


//---------------------------< R A W 1 4 _ T O _ F >----------------------------------------------------------
//
// Convert raw 14-bit temperature to degrees Fahrenheit.
//

float Systronix_HDC1080::raw14_to_f (uint16_t raw14)
	{
	return (raw14_to_c (raw14) * 1.8) + 32.0;
	}


//---------------------------< G E T _ T _ H _ D A T A >------------------------------------------------------
//
// Get current temperature and humidity measurement data and fill the data struct with the various info.  To get
// a single measurement, use register_read().  If the MODE bit is not right for dual measurement reading, this
// function returns FAULT;
//
// This function expects that the pointer register was previously set to HDC1080_TEMP_REG or TRIGGER_T_H (they
// are the same thing) because setting pointer register triggers a measurement cycle.  While a measurement cycle
// is in process data reads are NACKed (?)  Not really clear in the datasheet which reads:
//		A read operation will return a NACK if the contents of the registers have not been updated
//
// Does that mean measurement registers become 'not updated' when a trigger occurs?
//
// If pointer register is not pointing at HDC1080_TEMP_REG, this function will point it, causing a trigger, and
// return WAIT to the calling function which must then decide what to do.
//

uint8_t Systronix_HDC1080::get_t_h_data (void)
	{
	uint8_t	ret_val;

	if (_config_reg & MODE_T_AND_H)
		{
		if (_pointer_reg)								// if not pointed at temperature register
			{
			ret_val = pointer_write (TRIGGER_T_H);
			if (SUCCESS != ret_val)						// attempt to point it (triggers a measurement cycle)
				return ret_val;							// attempt failed; quit
			else
				return WAIT;							// attempt succeeded but now we must wait for measurement cycle completion
			}
		}
	else
		return FAULT;									// MODE bit wrong for two-measurement acquisition

	ret_val = default_read (&data.raw_temp, &data.raw_rh);	// attempt to read the temperature & humidity
	if (SUCCESS != ret_val)
		return ret_val;									// attempt failed; quit

	data.t_high = max(data.raw_temp, data.t_high);		// keep track of min/max raw temperatures
	data.t_low = min(data.t_low, data.raw_temp);

	data.deg_c = raw14_to_c (data.raw_temp);			// convert to human-readable forms
	data.deg_f = raw14_to_f (data.raw_temp);

	data.rh_high = max(data.raw_rh, data.rh_high);		// keep track of min/max raw relative humidities
	data.rh_low = min(data.rh_low, data.raw_rh);

	data.rh = raw14_to_rh (data.raw_rh);				// convert to human-readable form

	data.fresh = true;									// identify the current data set as new and fresh

	return pointer_write (TRIGGER_T_H);					// attempt to point it (triggers a measurement cycle)
	}
