#define F_CPU						12000000UL			// Crystal oscillator clock frequency 12MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LCD_EN				PB2
#define LCD_R_W				PB1
#define LCD_RS				PB0
#define LCD_DB4				PC0
#define LCD_DB5				PC1
#define LCD_DB6				PC2
#define LCD_DB7				PC3
#define LCD_Data_Port		PORTC
#define LCD_Control_Port	PORTB

#define Vref				5.00

void Initialize_LCD (void);
void LCD_Send_Nibble (unsigned char);
void LCD_Command (unsigned char);
void LCD_Write_char (unsigned char);
void LCD_Write_String (unsigned char*);
void LCD_EN_Pulse (void);


void Initialize_ADC(void);
uint16_t ADC_Conversion (uint8_t);
float ADC_voltage_reading (uint16_t);
unsigned char* format_ADC_readings (float);
unsigned char Display_readings [] = " .  V";			//formating string for ADC results displayed on LCD

int main (void)
{
	uint16_t voltage_reading;
	float measured_value;
	
	DDRC |= 0X2F;
	DDRB |= 0X07;
	
	_delay_ms(30);
	
	LCD_Control_Port &= ~(1 << LCD_RS);
	_delay_ms(1);
	LCD_Control_Port |= (1 << LCD_RS);
	
	Initialize_LCD ();
	Initialize_ADC ();
	
	for(;;)
	{
		voltage_reading = ADC_Conversion(4);			// read ADC4 value
		
		measured_value = ADC_voltage_reading(voltage_reading);
		
		LCD_Write_String ("ADC 4 = ");
		//LCD_Command(0XC0);							//move cursor to first character position of second line (Newline)
		
		//LCD_Write_String ("=");
		LCD_Write_String (format_ADC_readings(measured_value));
		_delay_ms(500);
		LCD_Command(0X02);								//return home
	}
}

void Initialize_LCD ()
{
	_delay_ms(15);
	
	LCD_Control_Port &= ~ (1 << LCD_RS);				// RS = 0, select LCD instruction register
	LCD_Control_Port &= ~ (1 << LCD_R_W);				// RW = 0, Write to LCD
	
	LCD_Send_Nibble (0X03);								// LCD wake up calls
	LCD_EN_Pulse ();
	_delay_ms(5);
	
	LCD_Send_Nibble (0X03);
	LCD_EN_Pulse ();
	_delay_us(200);
	
	LCD_Send_Nibble (0X03);
	LCD_EN_Pulse ();
	_delay_us(200);
	
	LCD_Send_Nibble (0X02);								//	Configure LCD to run in 4-bit mode
	LCD_EN_Pulse ();
	_delay_us(200);
	
	LCD_Command(0X28);									// Function set: 4 bit mode, Two lines, 5x8 character font
	_delay_us(200);
	LCD_Command(0X01);									// Clear display
	_delay_us(200);
	LCD_Command(0X06);									// Entry mode set: Increment cursor, no display shift
	_delay_us(200);
	LCD_Command(0X0C);									// Display On/Off control: Display ON, Cursor OFF, Blink cursor OFF
	_delay_us(200);
}

void LCD_Send_Nibble (unsigned char i)
{
	// check state of LCD data bit 4, then set it accordingly
	if (0X01 & i)
	{
		LCD_Data_Port |= (1 << LCD_DB4);
	}
	else
	{
		LCD_Data_Port &= ~(1 << LCD_DB4);
	}
	
	// check state of LCD data bit 5, then set it accordingly
	if (0X02 & i)
	{
		LCD_Data_Port |= (1 << LCD_DB5);
	}
	else
	{
		LCD_Data_Port &= ~(1 << LCD_DB5);
	}
	
	// check state of LCD data bit 6, then set it accordingly
	if (0X04 & i)
	{
		LCD_Data_Port |= (1 << LCD_DB6);
	}
	else
	{
		LCD_Data_Port &= ~(1 << LCD_DB6);
	}
	
	// check state of LCD data bit 7, then set it accordingly
	if (0X08 & i)
	{
		LCD_Data_Port |= (1 << LCD_DB7);
	}
	else
	{
		LCD_Data_Port &= ~(1 << LCD_DB7);
	}
	
}

void LCD_Command (unsigned char i)
{
	LCD_Control_Port &= ~ (1 << LCD_RS);				// RS = 0, select LCD instruction register
	LCD_Control_Port &= ~ (1 << LCD_R_W);				// RW = 0, Write to LCD
	
	LCD_Send_Nibble(0X0F & (i >> 4));					// Put upper 4 bits of command on LCD data bus
	LCD_EN_Pulse ();									// latch data to LCD
	LCD_Send_Nibble(0X0F & i);							// Put lower 4 bits of command on LCD data bus
	LCD_EN_Pulse ();									// latch data to LCD
}

void LCD_Write_char (unsigned char i)
{
	LCD_Control_Port |= (1 << LCD_RS);					// RS = 1, select LCD data register
	LCD_Control_Port &= ~ (1 << LCD_R_W);				// RW = 0, Write to LCD
	
	LCD_Send_Nibble(0X0F & (i >> 4));					// Put upper 4 bits of data on LCD data bus
	LCD_EN_Pulse ();									// latch data to LCD
	LCD_Send_Nibble(0X0F & i);							// Put lower 4 bits of data on LCD data bus
	LCD_EN_Pulse ();									// latch data to LCD
}

void LCD_Write_String (unsigned char* string)
{
	while (*string)										// loop until string end is reached ('\0' or Null character)
	{
		LCD_Write_char(*string++);						// send each string character one after another
	}
}

void LCD_EN_Pulse ()
{
	_delay_us(200);										// 200 microseconds delay before pulse to allow data to stabilize on LCD bus
	LCD_Control_Port |= (1 << LCD_EN);					// drive EN pin high
	_delay_us(200);										// wait 200 microseconds
	LCD_Control_Port &= ~(1 << LCD_EN);					// drive EN pin low
	_delay_us(200);										// 200 microseconds delay before next set of data is applied on LCD bus
}

void Initialize_ADC()
{
	ADCSRA = 0b10000111;        						// Enable ADC, set clock prescaler to 128
	DIDR0  = 0b00010000;        						// Disable digital input on ADC Channel 4 to reduce power consumption
	ADMUX  = 0b01000100;								// Use AVcc as ADC reference voltage, disable left adjust, use ADC4 as default input
}

uint16_t ADC_Conversion (uint8_t channel)
{
	uint16_t result = 0;
	
	ADMUX = (ADMUX & 0XF0) | (0X0F & channel);			// Enable conversion on selected ADC channel
	_delay_us(400);
	ADCSRA |= (1<<ADSC);								// Start ADC Conversion

	while((ADCSRA & (1<<ADIF)) != 0x10);				// Wait till conversion is complete

	result = ADC;										// Read the ADC Result
	ADCSRA |= (1 << ADIF);								// Clear ADC Conversion Interrupt Flag
	
	return result;
}

float ADC_voltage_reading (uint16_t ADC_reading)
{
	float voltage_value;
	voltage_value = (ADC_reading * Vref / 1024.0);		// Convert ADC reading (in counts) to voltage reading
	
	return voltage_value;
}

unsigned char* format_ADC_readings (float ADC_reading)
{
	int tmp;
	tmp = (int) (ADC_reading * 100.0);								// multiply ADC_reading float variable by 100 to help separate its individual digits
	
	if((ADC_reading * 100 - tmp) >= 0.5)							// Round 3rd digit after the decimal point of ADC measurement (ie 2.365 volts = 2.37)
	tmp += 1;
	
	Display_readings [3] = ((unsigned char) (tmp % 10)) | 0X30;		// Get ASCII char equivalent for each digit
	tmp /= 10;
	
	Display_readings [2] = ((unsigned char) (tmp % 10)) | 0X30;
	tmp /= 10;
	
	Display_readings [0] = ((unsigned char) (tmp % 10)) | 0X30;
	tmp /= 10;
	
	return Display_readings;
}
