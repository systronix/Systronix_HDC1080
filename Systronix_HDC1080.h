/**
 HDC1080.h
 Revisions


 */


// include this only one time
#ifndef HDC1080_H_
#define HDC1080_H_


//---------------------------< I N C L U D E S >--------------------------------------------------------------

#include <Arduino.h>

// Include the lowest level I2C library
#if defined (__MK20DX256__) || defined (__MK20DX128__) 	// Teensy 3.1 or 3.2 || Teensy 3.0
#include <i2c_t3.h>
#else
#include <Wire.h>	// for AVR I2C library
#endif


//---------------------------< D E F I N E S >----------------------------------------------------------------

#define		SUCCESS				0
#define		FAIL				(~SUCCESS)
#define		ABSENT				0xFD
#define		WAIT				0xFC		// get_t_h_data () caused a trigger so no data available to get until measurement cycle complete
#define		FAULT				0xFB		// MODE bit wrong for get_t_h_data()

#define		HDC1080_BASE	 	0x40		// 7-bit address not including R/W bit

#define		WR_INCOMPLETE		11
#define		SILLY_PROGRAMMER	12


//----------< P O I N T E R   R E G I S T E R >----------
//
// this register selects which of the other registers in the device are read/written.  Set only by an i2c write
// this 8-bit register can hold the following values:
//

#define	HDC1080_TEMP_REG		0x00	// point to 16-bit read-only temperature measurement register

#define	HDC1080_HUMIDITY_REG	0x01	// point to 16-bit read-only humidity measurement register

#define	HDC1080_CONFIG_REG		0x02	// point to 16-bit configuration and status register

#define	HDC1080_ID_43_REG		0xFB	// point to 16-bit read-only serial ID bits 39..24 (bytes 4..3)
#define	HDC1080_ID_21_REG		0xFC	// point to 16-bit read-only serial ID bits 23..8 (bytes 2..1)
#define	HDC1080_ID_0X_REG		0xFD	// point to 16-bit read-only serial ID bits 7..0 (byte 0; in upper byte; lower byte = 0x00)
#define	HDC1080_MANUF_ID_REG	0xFE	// point to 16-bit read-only manufacturer ID (0x5449)
#define	HDC1080_DEVICE_ID_REG	0xFF	// point to 16-bit read-only device ID (0x1050)


// measurement triggers

#define	TRIGGER_T_H				0x00	// alias of HDC1080_TEMP_REG, triggers a measurement of both temperature and humidity when MODE bit = 1
#define	TRIGGER_T				0x00	// alias of HDC1080_TEMP_REG, triggers a temperature measurement when MODE bit = 0
#define	TRIGGER_H				0x01	// alias of HDC1080_TEMP_REG, triggers a humidity measurement when MODE bit = 0


//----------< C O N F I G   R E G I S T E R   B I T S >----------
//
// Bits 14, 7..0 are reserved and must be 0x00
//

// bits 9..8 humidity resolution
#define	HRES_14_BIT				0		// bits 9..8 both 0 for 14-bit resolution
#define	HRES_11_BIT				(1<<8)	// 11-bit resolution
#define	HRES_8_BIT				(1<<9)	// 8-bit resolution

// bit 10 temperature resolution
#define	TRES_14_BIT				0		// 14 bit
#define	TRES_11_BIT				(1<<10)	// 11 bit

// bit 11 battery status (read only)
#define	BTST					(1<<11)	// read only; 0: Vbat > 2.8V; 1: Vbat <2.8V

//bit 12 mode of acquisition
#define	MODE_INDIVIDUAL			0		// single value
#define	MODE_T_AND_H			(1<<12)	// temperature and humidity; temperature first

//bit 13
#define	HEAT_DISABLE			0
#define	HEAT_ENABLE				(1<<13)

// bit 15
#define	RST						(1<<15)	// software reset; this bit self-clears


class Systronix_HDC1080
{
	protected:
		// Instance-specific properties; protected so that they aren't trampled by outside forces
		const uint8_t	_base = HDC1080_BASE;			// base address, only one; assign it here and be done

		uint8_t			_pointer_reg = 0;				// write-only; must be written as part of any other register write
		uint16_t		_temp_reg = 0;					// reset state; or data last read
		uint16_t		_humidity_reg = 0;				// reset state; or data last read
		uint16_t		_config_reg = 0x1000;			// reset state; or last setting of the configuration register

		char* 			_wire_name = (char*)"empty";
		i2c_t3			_wire = Wire;					// why is this assigned value = Wire? [bab]

		union int16_data
			{
			struct
				{										// data stored here with set_sensor_data(0x1234) gives this result:
				uint8_t		high;						// 0x12
				uint8_t		low;						// 0x34
				} as_struct;
			uint16_t		as_u16;						// 0x3412
			uint8_t			as_array[2];				// [0]: 0x12; [1]: 0x34
			} _sensor_data;

		void		tally_transaction (uint8_t);

	public:

		/**
		Array of Wire.status() extended return code strings, 11 as of 29Dec16 i2c_t3 release
		index into this with the value of status.
		There is an array of constant text: const status_text[11]
		char * makes the decl an array of char pointers, each pointing to constant text
		the first const means that array of char pointers can't change.
		We can access this with a const char * text_ptr which means point to char(s) which happen to be const
		Note each literal string has a null terminator added by C compiler.
		See NAP_UI_key_defs.h for similar

		TODO A problem is that SUCCESS returns 0 and gets put into error_val, so
		we can't tell the difference between SUCCESS and I2C_WAITING
		Since requestFrom is blocking, only "I2C message is over" status can occur.
		In Writing, with endTransmission, it is blocking, so only end of message errors can exist.
		*/
#if defined I2C_T3_H
		const char * const status_text[13] =
		{
			"I2C_WAITING", 		// first four are not errors but status; first eleven taken from i2c_t3.h
			"I2C_SENDING",
			"I2C_SEND_ADDR",
			"I2C_RECEIVING",
			"I2C_TIMEOUT", 		// start of 5 errors, status==4
			"I2C_ADDR_NAK",
			"I2C_DATA_NAK",
			"I2C_ARB_LOST",
			"I2C_BUF_OVF",
			"I2C_SLAVE_TX", 	// slave status; not errors
			"I2C_SLAVE_RX",
			"WR_INCOMPLETE",
			"SILLY_PROGRAMMER"	// Doh. Slap forehead.
		};
#else
		// Wire.h returns from endTransmission
		// 0=success, 1=data too long, 2=recv addr NACK, 3=recv data NACK, 4=other error
		const char * const status_text[5] =
		{
			"Success",
			"Data length",
			"Receive addr NAK",
			"Receive data NAK",
			"Other error"
		};
#endif

		struct data_t
			{
			uint16_t	raw_temp;				// most recent
			uint16_t	t_high = 0x0000;		// historical (since reset) high temp (preset here to device max low (-40 C) raw14 format)
			uint16_t	t_low = 0x7FFC; 		// historical low temp (preset here to device max high (125 C) raw14 format)
			float		deg_c;
			float		deg_f;
			uint16_t	raw_rh;
			uint16_t	rh_high = 0x0000;		// historical (since reset) high relative humidity (preset here to device max low (0%) raw14 format)
			uint16_t	rh_low = 0x7FFC; 		// historical low rh (preset here to device max high (100%) raw14 format)
			float		rh;
			bool		fresh;					// data is good and fresh TODO: how does one know that the data are not 'fresh'?
			} data;





		/** error stucture
		Note that this can be written by a library user, so it could be cleared if desired as part of
		some error recovery or logging operation. It could also be inadvertenly erased...

		successful_count overflowed at 258.5 hours. Making this a 64-bit unsigned (long long) allows
		for 2**32 times as many hours. So not likely to ever wrap wrap.
		*/
		struct
			{
			boolean		exists;							// set false after an unsuccessful i2c transaction
			uint8_t		error_val;						// the most recent error value, not just SUCCESS or FAIL
			uint32_t	incomplete_write_count;			// Wire.write failed to write all of the data to tx_buffer
			uint32_t	data_len_error_count;			// data too long
			uint32_t	timeout_count;					// slave response took too long
			uint32_t	rcv_addr_nack_count;			// slave did not ack address
			uint32_t	rcv_data_nack_count;			// slave did not ack data
			uint32_t	arbitration_lost_count;
			uint32_t	buffer_overflow_count;
			uint32_t	other_error_count;				// from endTransmission there is "other" error
			uint32_t	unknown_error_count;
			uint32_t	data_value_error_count;			// I2C message OK but value read was wrong; how can this be?
			uint32_t	silly_programmer_error;			// I2C address to big or something else that "should never happen"
			uint64_t	total_error_count;				// quick check to see if any have happened
			uint64_t	successful_count;				// successful access cycle
			} error;


		char*		wire_name;							// name of Wire, Wire1, etc in use

		Systronix_HDC1080 (void);							// constructor
		~Systronix_HDC1080 (void);							// destructor

		uint8_t		setup (i2c_t3 wire, char* name);	// initialize
		void		setup ()							// defaults to Wire net
					{setup (Wire, (char*) "Wire");};
		void 		begin(i2c_pins pins, i2c_rate rate);	// with pins and rate
		void		begin(void);
		uint8_t		init (uint16_t config, uint8_t initial_measure = TRIGGER_T_H);	// sets config reg & triggers first measurement

		uint8_t		pointer_write (uint8_t target_register);
		uint8_t		config_write (uint16_t data);
		
		uint8_t 	register_read (uint8_t target_register, uint16_t* data_ptr);
		uint8_t		default_read (uint16_t* data_ptr, uint16_t* h_data_ptr = NULL);
		uint8_t		base_get(void);
		void		sensor_data_set (uint16_t data);
		uint16_t	sensor_data_get (void);
		
		float		raw14_to_rh (uint16_t raw14);
		float		raw14_to_c (uint16_t raw14);
		float		raw14_to_f (uint16_t raw14);

		uint8_t		get_t_h_data (void);
		uint8_t		get_data ()											// an alias that may be useful
						{return get_t_h_data ();};

		void		reset_bus (void);				// invoke Wire[x] resetBus()
		uint32_t	reset_bus_count_read(void);		// read resetBusCount for Wire[x]
	private:

};

#endif	// PCA9557_H_
