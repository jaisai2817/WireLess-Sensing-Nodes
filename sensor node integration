#ifndef Header_file
#define Header_file

#define BAUD 9600
#define F_CPU 8000000
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)
#define NUM_ADC_READINGS 10

volatile uint16_t adc_values[NUM_ADC_READINGS];
volatile uint8_t adc_index = 0;
//#define BMP280_ADDR 0x77
#define BMP280_ADDR 0x76
// Calibration parameters
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

int32_t t_fine;


//TSL2561 Address Set
#define TSL2561_ADDR 0x39
#define READ_ADDR 0x0C
#define COMMAND_REG 0x80
#define TIMING_REG 0x81
#define CH1_LOW_REG 0x8C
#define CH1_HIGH_REG 0x8E

//Functions for USART0
void init_usart() {
	// Set baud rate
	UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
	UBRR0L = (unsigned char)UBRR_VALUE;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}

void usart_transmit(uint8_t data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));

	// Put data into buffer, sends the data
	UDR0 = data;
}

void USART0_init() {
	// Set baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)(UBRR_VALUE);
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

void USART0_transmit_char(char data) {

	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

unsigned char USART_Receive() {
	// Wait for data to be received
	while (!(UCSR0A & (1<<RXC0)));
	// Get and return received data from buffer
	return UDR0;
}


//For BMP280
void USART_send(uint8_t data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));
	// Put data into buffer, sends the data
	UDR0 = data;
}

void USART_putstring(const char *str) {
	// Sends the characters from the string one by one to USART
	while (*str) {
		USART_send(*str++);
	}
}

//Functions for HIH4602 Sensor
void ADC_init() {
	DDRF &= ~((1 << PF0)|(1 << PF1));
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
	ADMUX |= (1 << REFS0);
}

uint16_t ADC_read() {
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

//Functions for TSL2561 Sensor
void init_i2c() {
	// Set prescaler to 0, clear prescaler bits
	TWSR = 0;
	// Set bit rate
	TWBR = ((F_CPU / 100000UL) - 16) / 2; // Set TWI clock to 100kHz
	// Enable TWI
	TWCR = (1 << TWEN);
}

void i2c_start() {
	// Send start condition
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	// Wait for TWINT flag set
	while (!(TWCR & (1 << TWINT)));
}

void i2c_stop() {
	// Send stop condition
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	// Wait for TWSTO flag to clear
	while (TWCR & (1 << TWSTO));
}

void i2c_write(uint8_t data) {
	// Load data into TWI data register
	TWDR = data;
	// Clear TWINT flag by writing 1 to it
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack() {
	// Clear TWINT flag by writing 1 to it
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	// Wait for TWINT flag set
	while (!(TWCR & (1 << TWINT)));
	// Return received data
	return TWDR;
}

uint8_t i2c_read_nack() {
	// Clear TWINT flag by writing 1 to it
	TWCR = (1 << TWINT) | (1 << TWEN);
	// Wait for TWINT flag set
	while (!(TWCR & (1 << TWINT)));
	// Return received data
	return TWDR;
}

//For BMP280
void I2C_init() {
	// Set SCL frequency to 100kHz
	TWBR = 32;
}

void I2C_start() {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void I2C_stop() {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	_delay_us(50); // Wait for stop condition to complete
}

void I2C_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_read_ack() {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t I2C_read_nack() {
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

void tsl2561_init() {
	
	i2c_start();
	i2c_write(TSL2561_ADDR << 1);
	i2c_write(0x01);
	i2c_write(0x12);
	i2c_start();
	i2c_write(TSL2561_ADDR << 1);
	i2c_write(0x00);
	i2c_write(0x03);
	i2c_stop();
	//_delay_ms(15); // Delay to allow sensor to power up
}

void tsl2561_read_channel0(uint8_t *msb, uint8_t *lsb) {

	i2c_start();
	i2c_write(TSL2561_ADDR << 1);
	i2c_write(COMMAND_REG | READ_ADDR);
	i2c_start();
	i2c_write((TSL2561_ADDR << 1) | 1);
	*msb = i2c_read_ack();
	*lsb = i2c_read_nack();
	i2c_stop();
}

void tsl2561_read_channel1(uint8_t *msb, uint8_t *lsb) {

	i2c_start();
	i2c_write(TSL2561_ADDR << 1);
	i2c_write(COMMAND_REG | 0x0E);
	i2c_start();
	i2c_write((TSL2561_ADDR << 1) | 1);
	*msb = i2c_read_ack();
	*lsb = i2c_read_nack();
	i2c_stop();
}

float calculate_lux(uint16_t ch0, uint16_t ch1) {
	float d0, d1;
	d0=ch0;
	d1=ch1;
	//d0*=16;
	//d1*=16;
	float ratio = (float)d1 / (float)d0;
	if (ratio > 0 && ratio <= 0.50)
	return 0.0304 * d0 - 0.062 * d0 * pow(ratio, 1.4);
	else if (ratio > 0.50 && ratio <= 0.61)
	return 0.0224 * d0 - 0.031 * d1;
	else if (ratio > 0.61 && ratio <= 0.80)
	return 0.0128 * d0 - 0.0153 * d1;
	else if (ratio > 0.80 && ratio <= 1.30)
	return 0.00146 * d0 - 0.00112 * d1;
	else if (ratio > 1.30)
	return 0;
	else
	return 0; // Error condition
}


//For BMP Calculations
void read_calibration_data() {
	I2C_start();
	I2C_write(BMP280_ADDR << 1); // Write BMP280 address with write bit
	I2C_write(0x88); // Start reading calibration data from register 0x88
	I2C_start(); // Repeated start
	I2C_write((BMP280_ADDR << 1) | 0x01); // Write BMP280 address with read bit
	dig_T1 = (uint16_t)I2C_read_ack() | ((uint16_t)I2C_read_ack() << 8); // Read dig_T1 (16 bits, little-endian)
	dig_T2 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_T2 (16 bits, little-endian)
	dig_T3 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_T3 (16 bits, little-endian)
	dig_P1 = (uint16_t)I2C_read_ack() | ((uint16_t)I2C_read_ack() << 8); // Read dig_P1 (16 bits, little-endian)
	dig_P2 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P2 (16 bits, little-endian)
	dig_P3 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P3 (16 bits, little-endian)
	dig_P4 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P4 (16 bits, little-endian)
	dig_P5 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P5 (16 bits, little-endian)
	dig_P6 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P6 (16 bits, little-endian)
	dig_P7 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P7 (16 bits, little-endian)
	dig_P8 = (int16_t)I2C_read_ack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P8 (16 bits, little-endian)
	dig_P9 = (int16_t)I2C_read_nack() | ((int16_t)I2C_read_ack() << 8); // Read dig_P9 (16 bits, little-endian)
	I2C_stop();
}

int32_t read_raw_temperature() {
	uint8_t msb, lsb, xlsb;
	int32_t raw_temperature;

	// Start I2C communication
	I2C_start();
	// Write BMP280 address with write bit
	I2C_write(BMP280_ADDR << 1);
	// Write temperature register address
	I2C_write(0xFA);
	// Repeated start
	I2C_start();
	// Write BMP280 address with read bit
	I2C_write((BMP280_ADDR << 1) | 0x01);
	// Read MSB with ACK
	msb = I2C_read_ack();
	// Read LSB with ACK
	lsb = I2C_read_ack();
	// Read XLSB without ACK
	xlsb = I2C_read_nack();
	// Stop I2C communication
	I2C_stop();

	// Combine 20-bit raw temperature data
	raw_temperature = ((int32_t)msb << 12) | ((int32_t)lsb << 4) | ((int32_t)xlsb >> 4);

	return raw_temperature;
}

uint32_t read_raw_pressure() {
	uint8_t msb, lsb, xlsb;
	uint32_t raw_pressure;

	// Start I2C communication
	I2C_start();
	// Write BMP280 address with write bit
	I2C_write(BMP280_ADDR << 1);
	// Write pressure register address
	I2C_write(0xF7);
	// Repeated start
	I2C_start();
	// Write BMP280 address with read bit
	I2C_write((BMP280_ADDR << 1) | 0x01);
	// Read MSB with ACK
	msb = I2C_read_ack();
	// Read LSB with ACK
	lsb = I2C_read_ack();
	// Read XLSB without ACK
	xlsb = I2C_read_nack();
	// Stop I2C communication
	I2C_stop();

	// Combine 20-bit raw pressure data
	raw_pressure = ((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | ((uint32_t)xlsb >> 4);

	return raw_pressure;
}

void compensate_temperature(int32_t adc_T) {
	double var1, var2;
	var1 = ((((adc_T>>3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = (int32_t)(var1 + var2);
}

double compensate_pressure(uint32_t adc_P) {
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * ((int64_t)dig_P6);
	var2 =var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8) + ((var1 * (int64_t) dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+ var1)) * ((int64_t)dig_P1)>>33;
	if (var1 == 0.0) {
		return 0;
	}
	p = 1048576.0 - adc_P;
	p = (((p << 31) - var2) * 3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return p;
}

//Timer Initialisation Function
void Timer1_init() {
	// Set Timer1 mode to CTC (Clear Timer on Compare Match)
	TCCR1B |= (1 << WGM12);
	//TCCR1B |= (1 << CS11) | (1 << CS10);
	TCCR1B |= (1 << CS12); //Setting prescaler to 256
	// Set compare value for 1 second interrupt at 8MHz CPU frequency and 256 prescaler
	OCR1A = 15624;
	TIMSK |= (1 << OCIE1A);
}


#endif
