// Software description:
/*
  in this code AVR Atmega 328P interfacing with ADXL345 module via SPI
*/

// Preprocessor directives - software

// F_CPU macrodefinition
#define F_CPU 8000000

// UART macrodefinition

#define USART_BAUDRATE 38400
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// Preprocessor directives - software
#define SPI_DDR DDRB // Port directions for SPI interface (Input/Output)
#define SPI_PORT PORTB // Port state for SPI interface
#define CS 2
#define SCLK 5
#define MISO 4
#define MOSI 3


// ADXL 345 macrodefinitions:

// BW_Rate reg
#define BW_Rate_address 0x2C
#define Output_rate_100Hz 0b00001010
#define Output_rate_50Hz 0b00001001
#define Output_rate_25Hz 0b00001000
#define BW_Rate_conf 0b00000000

// PWR_Control reg
#define PWR_Control_address 0x2D
#define PWR_Control_conf 0b00001000
//D7 - D6 by definition 0
//D5 - Link bit - 0
//D4 - Auto Sleep - Disabled - 0
//D3 - Measurement Mode - 1
//D2 - Sleep - 0 (no sleeping)
//D1-D0 - Wakeup bits (don't care) 0

// INT_ENABLE reg
#define INT_ENABLE_address 0x2E

//  INT_MAP reg
#define INT_MAP_address 0x2F

//Data Format reg
#define Data_Format_address 0x31
#define Full_res 0b00001000
#define Fix10bit_res 0b00000000
#define Range2g 0b00000000
#define Range4g 0b00000001
#define Range8g 0b00000010
#define Range16g 0b00000011
#define Data_Format_conf 0b00000100

// FIFO_CTL reg
#define FIFO_CTL_address 0x38
#define FIFO_mode_Bypass 0b00000000
#define FIFO_mode_FIFO 0b01000000
#define FIFO_mode_Stream 0b10000000
#define FIFO_mode_Trigger 0b11000000
#define FIFO_Trigger_bit_INT1 0b00000000
#define FIFO_Trigger_bit_INT2 0b00100000
#define FIFO_Samples 16 //(in range between 1 to 32 (dec) or equal value in hex)

// Include section
#include <avr/io.h>
#include <stdio.h> //! konieczna do UARTU
#include <util/delay.h>
#include <avr/interrupt.h>


// Global Variable
volatile uint16_t timer_var=0;

// Function declaration
void USART0_initialization(void);
int USART0SendByte(char u8Data, FILE *stream);
void SPI_conf();
void ADXL345_conf();
void External_Interrupt0_config();
uint8_t SPI_Read(uint8_t address);
void SPI_Write(uint8_t address, uint8_t dane);
uint8_t ADXL345_DevID();
uint8_t ADXL345_read_conf(uint8_t address);
int16_t	ADXL345_ReadAxis_fix10bit_resolution(uint8_t address);
void Timer0_Normal_conf();

// File handlers
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);

// ISR - Interrupt Service Routine
ISR(INT0_vect){
	
}


// MAIN function
int main()
{
	// INITIALIZATION AND CONFIGURATION SECTION:
  // External Interrupt config 
	External_Interrupt0_config();
	// UART
	USART0_initialization();
	stdout = &usart0_str;
	//SPI
	SPI_conf();
	// Timery:
	Timer0_Normal_conf();
	
	//HOW TO READ PROPERLY INITIAL CONFIGURATION!
	/*
	1. Put your atmega into reset state by connecting RESET pin with GND
	2. Disconnect Vcc and GND wires of ADXL345 from power and wait 10s.
	3. Conncet ADXL345 wires to Vcc and GND respectively
	4. Run Serial Monitor (i.e. Arduino IDE, Putty, etc.)
	5. Put your atmega into normal mode by disconnect RESET pin with GND.
	6. In Serial Monitor you should find full intial and user configuration
	*/
	// KONFIGURACJA POCZATKOWA ADXL345:
	
	_delay_ms(1);
	printf("!!!\n");
	printf("!!!\n");
	printf("!!!\n");
	printf("Dev ID: %d\n",ADXL345_DevID());
	_delay_ms(1);
  
  // INITIAL CONFIGURATION OF ADXL345
	printf("INITIAL ADXL345 conf:\n");
	_delay_ms(1);
	printf("ADXL345 BW_Rate: %d\n",ADXL345_read_conf(BW_Rate_address));
	_delay_ms(1);
	printf("ADXL345 POWER_CTL: %d\n",ADXL345_read_conf(PWR_Control_address));
	_delay_ms(1);
	printf("ADXL345 DataFormat: %d\n",ADXL345_read_conf(Data_Format_address));
	_delay_ms(1);
	
	//ADXL345 conf;
	ADXL345_conf();

	// USER CONFIGURATION OF ADXL345:
	printf("!!!\n");
	printf("!!!\n");
	printf("!!!\n");
	printf("ADXL345 User conf:\n");
	printf("ADXL345 BW_Rate: %d\n",ADXL345_read_conf(BW_Rate_address));
	_delay_ms(1);
	printf("ADXL345 POWER_CTL: %d\n",ADXL345_read_conf(PWR_Control_address));
	_delay_ms(1);
	printf("ADXL345 DataFormat: %d\n",ADXL345_read_conf(Data_Format_address));
	_delay_ms(1000);
	
	// Globalne przerwania:
	sei();
	
	// variables in main

	while(1){
		// nothing to do
	}
}


// Function definition:

void USART0_initialization(void)
{
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}
int USART0SendByte(char u8Data, FILE *stream)
{
	if(u8Data == '\n')
	{
		USART0SendByte('\r', stream);
	}
	while(!(UCSR0A&(1<<UDRE0))){};
	UDR0 = u8Data;
	return 0;
}

void SPI_conf(){
	
	//!!! FOR SPI REMEMBER ABOUT PORT DIRECTION REGISTER!!!!!!!!
	SPI_DDR |= (1<<SCLK)|(1<<CS)|(1<<MOSI);
	SPI_DDR &= ~(1<<MISO); 	
	SPI_PORT |= (1<<CS); // High on Chip Select Pin -> SPI transmition disabled!
	
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR1);
	// SPIE = 0
	// SPE = 1
	// DORD = 0
	// MSTR = 1
	// CPOL = 1
	// CPHA = 1
	// OFFTOP: f_osc = 8MHZ
	// SPR1 = 1
	
	SPSR = 0;
	// nothing to set

	
}

uint8_t SPI_Read(uint8_t address){
	uint8_t return_value;
	SPI_PORT &= ~(1<<CS);
	//_delay_ms(1); // might be used if necessary
	SPDR = (address&0x3F)|0x80;
	while(!(SPSR &(1<<SPIF))); 
	SPDR = 0xFF; 
	while(!(SPSR &(1<<SPIF)));
	return_value = SPDR;
	//_delay_ms(1); //might be used if necessary
	SPI_PORT |= (1<<CS);
	return return_value;
	
}

void SPI_Write(uint8_t address, uint8_t dane){
	SPI_PORT &= ~(1<<CS);
  //_delay_ms(1); //might be used if necessary
	SPDR = 0x3F&address; 
	while(!(SPSR &(1<<SPIF))); 	
  SPDR = dane;
	while(!(SPSR &(1<<SPIF)));
	//_delay_ms(1); //might be used if necessary
  SPI_PORT |= (1<<CS);

}


uint8_t ADXL345_DevID(){
	return SPI_Read(0x00);
}


void ADXL345_conf(){
	uint8_t config;
	// Register 0x2C—BW_RATE (Read/Write)
	config = BW_Rate_conf|Output_rate_50Hz; // 9 (dec)
	SPI_Write(BW_Rate_address,config);
	
	// Register 0x2D—POWER_CTL (Read/Write)
	config  = PWR_Control_conf; // 8(dec)
	SPI_Write(PWR_Control_address,config);
	
	// Register 0x2E—INT_ENABLE (Read/Write)
	config = 0b00000001;
	SPI_Write(INT_ENABLE_address,config);
	
	//  Register 0x2F—INT_MAP (R/W)
	config = 0b11111101
	SPI_Write(INT_MAP_address,config);
	
	//Register 0x31—DATA_FORMAT (Read/Write)
	config  = Data_Format_conf|Fix10bit_res|Range4g; 
	// Data_Format_conf  0b00000100
	//Fix10bit_res 0b00000000 
	//Range4g 0b00000001
	
	//		0b00000100
	//		0b00000000
	//		0b00000001
	//(OR): 0b00000101 (Data_Format) = 5 (dec)
	SPI_Write(Data_Format_address, config);
	
	//Register 0x38—FIFO_CTL (Read/Write)
	config = FIFO_mode_FIFO|FIFO_Trigger_bit_INT1|FIFO_Samples;
	SPI_Write(FIFO_CTL_address,config);
	
	
}


void External_Interrupt0_config(){
	EICRA = (1<<ISC01);
	EIMSK = (1<<INT0);
}
uint8_t ADXL345_read_conf(uint8_t address){
	return SPI_Read(address);
}

int16_t	ADXL345_ReadAxis_fix10bit_resolution(uint8_t address){
	int16_t return_value;
	return_value = SPI_Read(address); // Adres to jest adres nizszego rejestru
	return_value |= (SPI_Read(address+1)<<8); 
	if (return_value&0x8000) //czyli jesli MSB = 1
	{
		return_value = (return_value>>6);
		return_value |= 0b1111110000000000;
	}
	else // czyli jesli MSB = 0
		return_value = (return_value>>6);

	return 	return_value; // raw values
	}
	
