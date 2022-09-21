/*
 * FIVR.c
 *
 * Created: 3/28/2018 4:44:47 PM
 * Author : Alex
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#define F_CPU 8000000UL

#define BNO055_addr 0x50 //Address of BNO055 0x28
#define SLA_W1 0x5C //Address of VL6180X Thumb  0x2E
#define SLA_R1 0x5D //Address of VL6180X Thumb  0x2E  shifted by one for R bit = 1
#define SLA_W2 0x54 //Address of VL6180X Index  0x2A
#define SLA_R2 0x55 //Address of VL6180X Index  0x2A  shifted by one for R bit = 1
#define SLA_W3 0x56 //Address of VL6180X Middle 0x2B
#define SLA_R3 0x57 //Address of VL6180X Middle 0x2B  shifted by one for R bit = 1
#define SLA_W4 0x58 //Address of VL6180X Ring   0x2C
#define SLA_R4 0x59 //Address of VL6180X Ring   0x2C  shifted by one for R bit = 1
#define SLA_W5 0x5A //Address of VL6180X Pinky  0x2D Set
#define SLA_R5 0x5B //Address of VL6180X Pinky  0x2D  shifted by one for R bit = 1
#define VL_CHANGE_ADDR_REG 0x212
#define VL_SYSTEM__FRESH_OUT_OF_RESET 0x016
#define VL_TAKE_RANGE 0x018
#define VL_RESULT_RANGE_STATUS 0x04d
#define VL_RANGE_STATUS 0x04f
#define VL_RANGE_RESULT 0x062
#define VL_SYSTEM_INTERRUPT_CLEAR 0x015

#define SCL_CLOCK 200000L

//

/*************************************************************************
//TWI Status Codes pg 273
*************************************************************************/
#define START 0x08
#define REP_START 0x10

#define MT_SLA_ACK 0x18
#define MT_SLA_NOACK 0x20
#define MT_DATA_ACK 0x28
#define MT_DATA_NOACK 0x30

#define MR_SLA_ACK 0x40
#define MR_SLA_NOACK 0x48
#define MR_DATA_ACK 0x50
#define MR_DATA_NOACK 0x58
///////////////////////////////////////////////////////////////////////////

/*************************************************************************
//BNO055 Register Addresses
*************************************************************************/
#define BNO055_OPR_MODE 0x3D //Default value = 0x1C
#define BNO055_PWR_MODE_ADDR				(0X3E)
#define BNO055_PAGE_ID_ADDR				    (0X07)
#define BNO055_CALIB_STAT_ADDR 0x35 //check if components have been calibrated 
// <7:6> Current system calibration status: 3=Full 0=not calibrated. GYR=<5:4> ACC=<3:2> MAG=<1:0>
/* Quaternion data registers */
#define BNO055_SELFTEST_RESULT_ADDR 0x36
#define BNO055_POWER_MODE_NORMAL	(0X00)
/* Operation mode settings*/
#define BNO055_OPERATION_MODE_CONFIG			(0X00)
#define BNO055_OPERATION_MODE_ACCONLY			(0X01)
#define BNO055_OPERATION_MODE_MAGONLY			(0X02)
#define BNO055_OPERATION_MODE_GYRONLY			(0X03)
#define BNO055_OPERATION_MODE_ACCMAG			(0X04)
#define BNO055_OPERATION_MODE_ACCGYRO			(0X05)
#define BNO055_OPERATION_MODE_MAGGYRO			(0X06)
#define BNO055_OPERATION_MODE_AMG				(0X07)
#define BNO055_OPERATION_MODE_IMUPLUS			(0X08)
#define BNO055_OPERATION_MODE_COMPASS			(0X09)
#define BNO055_OPERATION_MODE_M4G				(0X0A)
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF		(0X0B)
#define BNO055_OPERATION_MODE_NDOF				(0X0C)
#define BNO055_CHIP_ID							(0xA0)
#define BNO055_CHIP_ID_ADDR					 (0x00)
// <7:4> reserved. Bit 3 = MCU, bit 2 = GYR, bit 1 = MAG, bit 0 = ACC. 1=pass, 0=fail
#define BNO055_SYS_STATUS_ADDR 0x39
//0=System idle, 1=system error, 2=initialized peripherals, 3=system initialization, 4=in self test
// 5=sensor fusion algorithm running, 6=system running w/o fusion algorithm 
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0x27

#define BNO055_EUL_DATA_X_LSB 0x1A

//#define BAUD 38400
#define BAUD 250000
#define MYUBRR ((F_CPU/16/BAUD)-1)

void PWM_Init(void);

void usart_init(unsigned int ubrr);
void usart_putchar(unsigned char data);
unsigned char usart_getchar(void);
void usart_string(char s[], uint8_t len);


void Init_Led(void);
void Set_Led(unsigned char output);


void i2c_init(void);
unsigned char i2c_start(unsigned char address);
void i2c_start_wait(unsigned char address);
unsigned char i2c_rep_start(unsigned char address);
void i2c_stop(void);
unsigned char i2c_write( unsigned char data );
unsigned char WriteByte(unsigned int reg, unsigned char data);
unsigned char i2c_readAck(void);
unsigned char i2c_readNak(void);
unsigned char i2c_find_addr(void);

unsigned char i2c_write8_to_dev(unsigned char i2c_addr, unsigned char reg_addr, unsigned char data);
unsigned char i2c_read8_from_dev(unsigned char i2c_addr, unsigned char reg_addr);
unsigned char i2c_readX_from_dev(unsigned char i2c_addr, unsigned char reg_addr, unsigned char* buffer, unsigned char len);

unsigned char VL6180X_Init_Set_Device_Addr(void);
void VL6180X_Initialize(unsigned char devaddrW);
void VL6180X_Initialize_OG(unsigned char devaddrW);
unsigned char WriteData(unsigned int reg, unsigned char *data, unsigned char len);


unsigned char VL6180X_Write_Addr(unsigned char currAddrW, unsigned char newAddrW);
unsigned char VL6180X_Single_Range(void);
unsigned char VL6180X_Single_Single_Range(unsigned char SLA);

unsigned char FIVR_Combined(void);
unsigned char FIVR_Combined2(void);
unsigned char FIVR_Combined3(void);


void waste_time(void);

unsigned char BNO055_Init(void);
void setExtCrystalUse(uint8_t usextal);
void setMode(uint8_t modeSelect);
unsigned char BNO055_GetQuat(void);
void BNO055_GetQuat_delayed(void);

void BNO055_GetEUL_delayed(void);
void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);


volatile unsigned char System_On = 0;
volatile unsigned char Calibrate_On = 0;
unsigned char Byte_Received = 0;
unsigned int Servo_State = 0;
unsigned char buffer[8];
double _xyz[3];
int main(void)
{	
	PWM_Init();
	DDRB |= (1<<DDB1)|(1<<DDB2);
	DDRD |= (1<<DDD3);
	PORTB |= (1<<PORTB2)|(1<<PORTB1);
	PORTD |= (1<<PORTD3);
	const double scale = (1.0/(1<<14));
			OSCCAL = 0xA1;
			usart_init(MYUBRR);
			
			MCUSR = 0;
			i2c_init();
			//usart_putchar(0xAE);
			sei();
			//Button Input
		// 	DDRD &= 0xEF;
		// 	PORTD &= 0xEF;
			memset(buffer, 0, 8);
			usart_putchar(0x66);
			usart_putchar(sizeof(float));
		
			//Set up Thumb = pin5(PD3) ,Index = pin16(PB2), Middle/Ring/Pinky = Pin15(PB1)
			//set all as outputs
			
		
			unsigned char ret=0;
			VL6180X_Init_Set_Device_Addr();
			int count = 0;
			
		//  	while(!BNO055_Init()){
		// 		while(count<10){
		// 			count++;
		// 			usart_putchar(0xE0);
		// 			usart_putchar((unsigned char)"\n");
		// 		}
		// 	}
			count = 0;
			//setExtCrystalUse(1);
			//usart_putchar(0xC2);
			
			
			while(1)
			{		
				while(System_On){
					//BNO055_GetEUL_delayed();
					FIVR_Combined3();
					//BNO055_GetQuat();
		// 			sprintf(out_str, "X = %f\r\n", (float)_xyz[0]);
		// 		 	usart_string(out_str ,16);
		// 			sprintf(out_str, "Y = %f\r\n", (float)_xyz[1]);
		// 			usart_string(out_str ,16);
		// 			sprintf(out_str, "Z = %f\r\n", (float)_xyz[2]);
		// 			usart_string(out_str ,16);
		// 
		// 			getCalibration(&sys, &gyro, &accel, &mag);
		// 			sprintf(out_str, "Calibration: %d\n", sys);
		// 			usart_string(out_str ,17);
		// 			sprintf(out_str, "gyro: %d\n", gyro);
		// 			usart_string(out_str ,10);
		// 			sprintf(out_str, "accel: %d\n", accel);
		// 			usart_string(out_str ,11);
		// 			sprintf(out_str, "mag: %d\n", mag);
		// 			usart_string(out_str ,9);
		// 			unsigned char *chptr;
		// 			chptr = (unsigned char *) &floatvalue;
		// 			usart_putchar(*chptr++);
		// 			usart_putchar(*chptr++);
		// 			usart_putchar(*chptr);
				}
		
			}
			/*Very important */
		// 	while(1){
		// 		//usart_putchar(0xDD);
		// 		//usart_putchar(System_On);
		// 		count++;
		// 		while(System_On){
		// 			count = 0;
		// 			cli();
		// 			//BNO055_GetQuat();
		// 			FIVR_Combined();
		// 			sei();
		// 			_delay_ms(150);
		// 		}
		// 	}
			/*END Very important */
		
				
}
// 	while(1){
// 		//usart_putchar(0xDD);
// 		//usart_putchar(System_On);
// 		count++;
// 		while(System_On){
// 			count = 0;
// 			while(count<10){
// 				//cli();
// 				VL6180X_Single_Range();
// 				//sei();
// 				count++;
// 			}
// 			//count =0;
// 			//cli();
// 			BNO055_GetQuat();
// 			//sei();
// 		}
// 	}
	
// 	unsigned char chr, answ;
// 	unsigned int count = 0;
	
	
	
	
// 	for(unsigned char i=0x48; i<0x58; i = i+2)
// 	{
// 		ret = i2c_start(i);
// 		if(ret)
// 		{
// 			i2c_stop();
// 			//Set_Led(0x44);
// 		}
// 		else
// 		{
// 			usart_putchar(i);
// 			i2c_stop();
// 		}
// 	}

// 	chr = i2c_find_addr();
//    	usart_putchar(chr);
//  	VL6180X_Initialize(chr<<1);

// 	int count = 0;
//     while (1) 
//     {
// 		if(PIND & (1<<PIND4)){
// 			while(count < 30){
// 				VL6180X_Single_Range();
// 				count++;
// 			}
// 			count = 0;
// 			_delay_ms(50);
// 		}	
// 	}

//if we receive a byte from Unity
ISR(USART_RX_vect){	
	Byte_Received = usart_getchar();
	usart_putchar(Byte_Received);
	//Bonnie wants us to start sending data
	if(Byte_Received == 0xA1){
		System_On = 1;
		Servo_State = 0;
		return;
	}
	//Bonnie wants us to stop sending data
	else if(Byte_Received == 0xDE){
		//Normalize servos 
		System_On = 0;
		Calibrate_On = 0;
		Servo_State = 0;
	}	
	else if(Byte_Received == 0xCA){
		Calibrate_On = 1;
	}

// 	if(Servo_State == 150000){
// 		Servo_State = 0;
// 	}
	if(System_On){
		//Index
		if(Servo_State%3 == 0){

			switch(Byte_Received){
				case 1:
				OCR1B = 45;
				break;
				case 2:
				OCR1B = 48;
				break;
				case 3:
				OCR1B = 51;
				break;
				
				case 4:
				OCR1B = 54;
				break;
				
				case 5:
				OCR1B=57;
				break;
				
				case 6:
				OCR1B=60;
				break;
				
				case 7:
				OCR1B = 63;
				break;
				
				case 8:
				OCR1B = 66;
				break;
				
				case 9:
				OCR1B = 69;
				break;
				
				case 10:
				OCR1B = 72;
				break;
				
				case 11:
				OCR1B = 75;
				break;
				
				case 12:
				OCR1B = 78;
				break;
				
				case 13:
				OCR1B=81;
				break;
				
				case 14:
				OCR1B = 84;
				break;
				
				case 15:
				OCR1B = 87;
				break;
				
// 				default:
// 				OCR1B = 45;
// 				break;
			}	
			Servo_State++;
		}
		//3 fingers
		else if(Servo_State%3 == 1){
			switch(Byte_Received){
				case 1:
				OCR1A = 130;
				break;
				
				case 2:
				OCR1A = 130;
				break;
				
				case 3:
				OCR1A = 131;
				break;
				
				case 4:
				OCR1A = 128;
				break;
				
				case 5:
				OCR1A= 125;
				break;
				
				case 6:
				OCR1A= 122;
				break;
				
				case 7:
				OCR1A = 119;
				break;
				
				case 8:
				OCR1A = 116;
				break;
				
				case 9:
				OCR1A = 113;
				break;
				
				case 10:
				OCR1A = 110;
				break;
				
				case 11:
				OCR1A = 107;
				break;
				
				case 12:
				OCR1A = 104;
				break;
				
				case 13:
				OCR1A=101;
				break;
				
				case 14:
				OCR1A = 99;
				break;
				
				case 15:
				OCR1A = 87;
				break;
				
				
// 				default:
// 				OCR1A = 98;
// 				break;
			}
			Servo_State++;
		}
		//Thumb
		else if(Servo_State%3 == 2){
			switch(Byte_Received){
				case 1:
				OCR2B = 30;
				break;
				
				case 2:
				OCR2B = 27;
				break;
				
				case 3:
				OCR2B = 24;
				break;
				
				case 4:
				OCR2B = 22;
				break;
				
				case 5:
				OCR2B = 20;
				break;
				
				case 6:
				OCR2B = 18;
				break;
				
				case 7:
				OCR2B = 16;
				break;
			}	
			Servo_State++;
		}
	}
}

	
	
	


void PWM_Init(void)
{
	
	//TIMSK1 |= (1<<OCIE1B)|(1<<TOIE1)|(1<<ICIE1)|(1<<OCIE1A);
	//TIMSK2 |= (1<<OCIE2B)|(1<<TOIE2);
	ICR1 = 0x04E2;
	OCR1A = 137;		//87 is closed 137 is open
	OCR1B = 87;		//87 is closed 45 is openOCR2B = 16;		//16 is closed 30 is open
	
	OCR2A = 98;
	
	TCCR1A |= (1<<COM1B1)|(1<<WGM11)|(1<<COM1A1);	//Starts upcounting and looking for compare B	//Phase Correct 8 bit, clear OCA0 on Compare Math
	// trigger hits on postive slope
	
	// enables capture flag(when timer reaches TOP)
	TCCR1B |= (1<<CS11)| (1<<CS10)| (1<<WGM13);
	
	TCCR2A |= (1<<COM2B1)|(1<<WGM20);	//compare match for B
	TCCR2B |= (1<<CS22)| (1<<CS21);		// sets prescalar to 1024 then sets top to
	
	//unsigned char sreg;
	//sreg = SREG;
	//cli();
			//set timer 1 top
	//set timer 2 top
	//sei();
	//need to define interrupts in ISR
}


void Init_Led(void)
{
	//Set LED Output
	DDRD |= DDRD | 0xFC;//(1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7);
	DDRB |= (1<<DDB7)|(1<<DDB0);

	PORTD = PORTD & 0x03 ;
	PORTB |= (0<<PORTB7)|(0<<PORTB0);
}

void Set_Led(unsigned char output)
{
	//clears LEDs to off
	PORTD = PORTD & 0x03;
	PORTB = PORTB & 0x7E;
	
	unsigned char outputMod = output;
	//set D2
	PORTD = PORTD | ((outputMod & 0x80)>>5);
	
	//set D3
	outputMod = output;
	PORTD = PORTD | ((outputMod & 0x40)>>3);
	
	//set D4
	outputMod = output;
	PORTD = PORTD | ((outputMod & 0x20)>>1);
	
	//set B7
	outputMod = output;
	PORTB = PORTB | ((outputMod & 0x10)<<3);
	
	//set D5
	outputMod = output;
	PORTD = PORTD | ((outputMod & 0x08)<<2);
	
	//set D6
	outputMod = output;
	PORTD = PORTD | ((outputMod & 0x04)<<4);
	
	//set D7
	outputMod = output;
	PORTD = PORTD | ((outputMod & 0x02)<<6);
	
	//set B0
	outputMod = output;
	PORTB = PORTB | (outputMod & 0x01);
}
/*************************************************************************
	VL6180X Protocol Code 
*************************************************************************/
unsigned char VL6180X_Init_Set_Device_Addr()

{
	unsigned char chr = 0x00; 

	////////////// Setting addresses
	unsigned char oldAddr = 0x00;
	//Draw VL6180x GPIO0 Low for one by one addr change
	DDRB |= (1<<DDB6) | (1<<DDB7);
	//DDRD |= (1<<DDD5) | (1<<DDD6) | (1<<DDD7);
	DDRD |= (1<<DDD5) | (1<<DDD6);
	PORTB &= 0x3F;
	//PORTD &= 0x1F;
	PORTD &= 0x9F;
	_delay_ms(50);
	//Find Thumb. change to address greater than all the rest 
	
	//PORTD |= (1<<PORTD7);
	chr = i2c_find_addr();
	//Convert oldAddress to call for device on i2c bus
	oldAddr = (chr<<1);
	//Write Thumb address as SLA_W1 
	
	chr = VL6180X_Write_Addr(oldAddr, SLA_W1);
	
	usart_putchar(chr);
	
	//Enable Pinky VL6180x
	PORTB |= (1<<PORTB6);
	_delay_ms(50);
	chr = i2c_find_addr();
	oldAddr = (chr<<1);
	//Write Pinky address as SLA_W5 
	chr = VL6180X_Write_Addr(oldAddr, SLA_W5);
		usart_putchar(chr);

	
	//Enable Ring finger VL6180x
	PORTB |= (1<<PORTB7);
	_delay_ms(50);
	chr = i2c_find_addr();
	oldAddr = (chr<<1);
	//Write Ring address as SLA_W4
	chr = VL6180X_Write_Addr(oldAddr, SLA_W4);
	usart_putchar(chr);
			
	//Enable Middle finger VL6180x
	PORTD |= (1<<PORTD5);
	_delay_ms(50);
	chr = i2c_find_addr();
	oldAddr = (chr<<1);
	//Write Middle address as SLA_W3
	chr = VL6180X_Write_Addr(oldAddr, SLA_W3);
	usart_putchar(chr);
	
	//Enable Index finger VL6180x
	PORTD |= (1<<PORTD6);
	_delay_ms(50);
	chr = i2c_find_addr();
	oldAddr = (chr<<1);
	//Write Index address as SLA_W2
	chr = VL6180X_Write_Addr(oldAddr, SLA_W2);
	usart_putchar(chr);
	
	//usart_putchar(0x11);
	////////////// End setting addresses
	
	// Initialize all VL6180x
	VL6180X_Initialize(SLA_W5);
	VL6180X_Initialize(SLA_W4);
	VL6180X_Initialize(SLA_W3);
	VL6180X_Initialize(SLA_W2);
	VL6180X_Initialize(SLA_W1);
	
	return 0x23;
}

/*************************************************************************
 Set new device address for VL6180X TOF sensor
 
 Input: currAddrW = current write address of the device that is put on the bus
		newAddrW = desired write address of the device 
 
 Return:  Address received from VL6180's device address register
 Note: currAddrw and newAddrW should already be shifted left by one.
		ie. VL6180x default address = 0x29 but must be put on the i2c
			bus as 0x52 because the 7 msb are used.
*************************************************************************/
unsigned char VL6180X_Write_Addr(unsigned char currAddrW, unsigned char newAddrW)
{
	////// Write new address
	unsigned char ret = 0x00;
	ret = i2c_start(currAddrW); /////////// Note that 11-27-17
	if(ret)
	{
		i2c_stop();
		return 1;
	}
	else
	{
		i2c_write((unsigned char)(VL_CHANGE_ADDR_REG>>8));
		i2c_write((unsigned char)(VL_CHANGE_ADDR_REG));
		i2c_write(newAddrW>>1);
		i2c_stop();
		
		i2c_start_wait(newAddrW);
		i2c_write((unsigned char)(VL_CHANGE_ADDR_REG>>8));
		i2c_write((unsigned char)(VL_CHANGE_ADDR_REG));
		i2c_rep_start(newAddrW + 1);
		ret = i2c_readNak();
		i2c_stop();

		return ret;
	}
}
unsigned char VL6180X_Single_Single_Range(unsigned char SLA){
	unsigned char chr;
	i2c_start(SLA);
	WriteByte(VL_TAKE_RANGE, 0x01);
	i2c_rep_start(SLA);
	i2c_write((unsigned char)(VL_RANGE_RESULT>>8));
	i2c_write((unsigned char)(VL_RANGE_RESULT));
	i2c_rep_start(SLA+1);
	chr = i2c_readNak();
	usart_putchar(chr);
	return 0;
}
unsigned char VL6180X_Single_Range()
{
	// 	unsigned char rangeData[4] = {0x00, 0x00, 0x00, 0x00};
	// 	unsigned char devAddr[4] = {SLA_W2, SLA_W3, SLA_W4, SLA_W5};
	
	unsigned char rangeData[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned char devAddr[5] = {SLA_W1, SLA_W2, SLA_W3, SLA_W4, SLA_W5};
		
	for(int i=0; i<5; i++)
	{
		i2c_start_wait(devAddr[i]);
		WriteByte(VL_TAKE_RANGE, 0x01);
	}
	_delay_ms(10);
	for(int j = 0; j < 5; j++)
	{
		i2c_start_wait(devAddr[j]);
		i2c_write((unsigned char)(VL_RANGE_RESULT>>8));
		i2c_write((unsigned char)(VL_RANGE_RESULT));
		i2c_rep_start(devAddr[j]+1);
		rangeData[j] = i2c_readNak();
		//i2c_stop();
		usart_putchar(rangeData[j]);
		
	}
	//	moveServo(rangeData[0]);
	return 0;
}////// end single range measurement

unsigned char FIVR_Combined()
{
	// 	unsigned char rangeData[4] = {0x00, 0x00, 0x00, 0x00};
	// 	unsigned char devAddr[4] = {SLA_W2, SLA_W3, SLA_W4, SLA_W5};
	
	unsigned char rangeData[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned char devAddr[5] = {SLA_W1, SLA_W2, SLA_W3, SLA_W4, SLA_W5};
	
	for(int i=0; i<5; i++)
	{
		i2c_start_wait(devAddr[i]);
		WriteByte(VL_TAKE_RANGE, 0x01);
	}
	//BNO055_GetQuat_delayed();
	BNO055_GetEUL_delayed();
	_delay_ms(70); // 9ms delay
	for(int j = 0; j < 5; j++)
	{
		i2c_start_wait(devAddr[j]);
		i2c_write((unsigned char)(VL_RANGE_RESULT>>8));
		i2c_write((unsigned char)(VL_RANGE_RESULT));
		i2c_rep_start(devAddr[j]+1);
		rangeData[j] = i2c_readNak();
		//i2c_stop();	
	}
	for(int k = 0; k<5; k++){
		usart_putchar(rangeData[k]);
	}
// 		usart_putchar(0x00);
// 		usart_putchar(0x00);
// 		usart_putchar(0x00);
		//X
// 		usart_putchar(buffer[3]);
// 		usart_putchar(buffer[2]);
// 		
// 		//Y
// 		usart_putchar(buffer[5]);
// 		usart_putchar(buffer[4]);
// 		
// 		//Z
// 		usart_putchar(buffer[7]);
// 		usart_putchar(buffer[6]);
// 		
// 		//W
// 		usart_putchar(buffer[1]);
// 		usart_putchar(buffer[0]);
		//X MSB
		usart_putchar(buffer[1]);
		usart_putchar(buffer[1]);
		//Y MSB
		usart_putchar(buffer[3]);
		usart_putchar(buffer[2]);
		//Z MSB
		usart_putchar(buffer[5]);
		usart_putchar(buffer[4]);
		usart_putchar(0x00);
		usart_putchar(0x00);

	//	moveServo(rangeData[0]);
	return 0;
}////// end single range measurement

unsigned char FIVR_Combined2()
{
	// 	unsigned char rangeData[4] = {0x00, 0x00, 0x00, 0x00};
	// 	unsigned char devAddr[4] = {SLA_W2, SLA_W3, SLA_W4, SLA_W5};
	char out_str[30] = {0};     // string to print to and transmit
	uint8_t sys, gyro, accel, mag = 0;
	unsigned char rangeData[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned char devAddr[5] = {SLA_W1, SLA_W2, SLA_W3, SLA_W4, SLA_W5};
	
	for(int i=0; i<5; i++)
	{
		i2c_start_wait(devAddr[i]);
		WriteByte(VL_TAKE_RANGE, 0x01);
	}
	//BNO055_GetQuat_delayed();
	BNO055_GetEUL_delayed();
	_delay_ms(70); // 9ms delay
	for(int j = 0; j < 5; j++)
	{
		i2c_start_wait(devAddr[j]);
		i2c_write((unsigned char)(VL_RANGE_RESULT>>8));
		i2c_write((unsigned char)(VL_RANGE_RESULT));
		i2c_rep_start(devAddr[j]+1);
		rangeData[j] = i2c_readNak();
		//i2c_stop();
	}
	//Send 1st 8 Byte Packet of TOF Sensor Output
	for(int k = 0; k<5; k++){
		usart_putchar(rangeData[k]);
	}
	usart_putchar(0x00);
	usart_putchar(0x00);
	usart_putchar(0x00);
	
	//Send second Packet of X Eul and Y Eul
	unsigned char *chptr;
	chptr = (unsigned char *) &_xyz;
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);

	usart_putchar(*chptr++);
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);
	// Done sending second

	//Third packet
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);
	usart_putchar(*chptr++);

	usart_putchar(0x00);
	usart_putchar(0x00);
	usart_putchar(0x00);
	usart_putchar(0x00);
	
	if(Calibrate_On){
	sprintf(out_str, "X = %f\r\n", (float)_xyz[0]);
	usart_string(out_str ,16);
	sprintf(out_str, "Y = %f\r\n", (float)_xyz[1]);
	usart_string(out_str ,16);
	sprintf(out_str, "Z = %f\r\n", (float)_xyz[2]);
	usart_string(out_str ,16);

		getCalibration(&sys, &gyro, &accel, &mag);
		sprintf(out_str, "Calibration: %d\n", sys);
		usart_string(out_str ,17);
		sprintf(out_str, "gyro: %d\n", gyro);
		usart_string(out_str ,10);
		sprintf(out_str, "accel: %d\n", accel);
		usart_string(out_str ,11);
		sprintf(out_str, "mag: %d\n", mag);
		usart_string(out_str ,9);
	}
		return 0;
}

unsigned char FIVR_Combined3()
{
	// 	unsigned char rangeData[4] = {0x00, 0x00, 0x00, 0x00};
	// 	unsigned char devAddr[4] = {SLA_W2, SLA_W3, SLA_W4, SLA_W5};
//	char out_str[30] = {0};     // string to print to and transmit
//	uint8_t sys, gyro, accel, mag = 0;
	unsigned char rangeData[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned char devAddr[5] = {SLA_W1, SLA_W2, SLA_W3, SLA_W4, SLA_W5};
	cli();
	for(int i=0; i<5; i++)
	{
		i2c_start_wait(devAddr[i]);
		WriteByte(VL_TAKE_RANGE, 0x01);
	}
	//BNO055_GetQuat_delayed();
	//BNO055_GetEUL_delayed();
	sei();
	_delay_ms(70); // 9ms delay
	_delay_ms(70);
	_delay_ms(70);
	_delay_ms(70);
	cli();
	for(int j = 0; j < 5; j++)
	{
		i2c_start_wait(devAddr[j]);
		i2c_write((unsigned char)(VL_RANGE_RESULT>>8));
		i2c_write((unsigned char)(VL_RANGE_RESULT));
		i2c_rep_start(devAddr[j]+1);
		rangeData[j] = i2c_readNak();
		//i2c_stop();
	}
	//Send 1st 8 Byte Packet of TOF Sensor Output
	for(int k = 0; k<5; k++){
		usart_putchar(rangeData[k]);
	}
	usart_putchar(0x00);
	usart_putchar(0x00);
	usart_putchar(0x00);
	sei();
// 	
// 	//Send second Packet of X Eul and Y Eul
// 	unsigned char *chptr;
// 	chptr = (unsigned char *) &_xyz;
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 	// Done sending second
// 
// 	//Third packet
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 	usart_putchar(*chptr++);
// 
// 	usart_putchar(0x00);
// 	usart_putchar(0x00);
// 	usart_putchar(0x00);
// 	usart_putchar(0x00);
// 	
// 	if(Calibrate_On){
// 		sprintf(out_str, "X = %f\r\n", (float)_xyz[0]);
// 		usart_string(out_str ,16);
// 		sprintf(out_str, "Y = %f\r\n", (float)_xyz[1]);
// 		usart_string(out_str ,16);
// 		sprintf(out_str, "Z = %f\r\n", (float)_xyz[2]);
// 		usart_string(out_str ,16);
// 
// 		getCalibration(&sys, &gyro, &accel, &mag);
// 		sprintf(out_str, "Calibration: %d\n", sys);
// 		usart_string(out_str ,17);
// 		sprintf(out_str, "gyro: %d\n", gyro);
// 		usart_string(out_str ,10);
// 		sprintf(out_str, "accel: %d\n", accel);
// 		usart_string(out_str ,11);
// 		sprintf(out_str, "mag: %d\n", mag);
// 		usart_string(out_str ,9);
// 	}
	return 0;
}

	//X
	// 		usart_putchar(buffer[3]);
	// 		usart_putchar(buffer[2]);
	//
	// 		//Y
	// 		usart_putchar(buffer[5]);
	// 		usart_putchar(buffer[4]);
	//
	// 		//Z
	// 		usart_putchar(buffer[7]);
	// 		usart_putchar(buffer[6]);
	//
	// 		//W
	// 		usart_putchar(buffer[1]);
	// 		usart_putchar(buffer[0]);
	//X MSB
// 	usart_putchar(buffer[1]);
// 	usart_putchar(buffer[1]);
// 	//Y MSB
// 	usart_putchar(buffer[3]);
// 	usart_putchar(buffer[2]);
// 	//Z MSB
// 	usart_putchar(buffer[5]);
// 	usart_putchar(buffer[4]);
// 	usart_putchar(0x00);
// 	usart_putchar(0x00);

	//	moveServo(rangeData[0]);


void VL6180X_Initialize(unsigned char devaddrW)
{
	unsigned char devaddrR = devaddrW + 1;
	//usart_putchar(devaddrW);
	unsigned char ret = 0;
	ret = i2c_start(devaddrW);
	if(ret)
	{
	 	i2c_stop();
			usart_putchar(0xEE);
			usart_putchar(devaddrW);
			usart_putchar(ret);
 	}
	else
	{
		i2c_write((unsigned char)(VL_SYSTEM__FRESH_OUT_OF_RESET>>8));
		i2c_write((unsigned char)(VL_SYSTEM__FRESH_OUT_OF_RESET));
		i2c_rep_start(devaddrR);
		ret = i2c_readNak();
		//usart_putchar(0x10);
		//usart_putchar(ret);
		i2c_stop();
	
		if(ret == 1)
		{
			i2c_start_wait(devaddrW);
		
			WriteByte(0x0207, 0x01);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x0208, 0x01);
		
			i2c_start_wait(devaddrW);		
			WriteByte(0x0096,0x00);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x0096, 0xFD);
		
			i2c_start_wait(devaddrW);		
			WriteByte(0x00E3, 0x00);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00E4, 0x04);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00E5, 0x02);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00E6, 0x01);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00E7, 0x03);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00F5, 0x02);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00D9, 0x05);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00DB, 0xCE);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00Dc, 0x03);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00DD, 0xF8);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x009F, 0x00);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00A3, 0x3C);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00B7, 0x00);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00BB, 0x3C);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00B2, 0x09);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00CA, 0x09);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x0198, 0x01);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x01B0, 0x17);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x01AD, 0x00);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x00FF, 0x05);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x0100, 0x05);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x0199, 0x05);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x01A6, 0x1B);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x01AC, 0x3E);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x01A7, 0x1F);
		
			i2c_start_wait(devaddrW);
			WriteByte(0x0030, 0x00);
		
			//Set VL_SYSTEM__FRESH_OUT_OF_RESET to false; initialization complete
			i2c_start_wait(devaddrW);
			WriteByte(VL_SYSTEM__FRESH_OUT_OF_RESET, 0x00);
			//i2c_write(0x00);
			//i2c_stop();
			//}
			//usart_putchar(devaddrW);
			//usart_putchar(0xFF);
		}
	}
}
/////////Initialization given by VL6180 datasheet
/*WriteByte(0x0207, 0x01);
WriteByte(0x0208, 0x01);
WriteByte(0x0096, 0x00);
WriteByte(0x0097, 0xfd);
WriteByte(0x00e3, 0x00);
WriteByte(0x00e4, 0x04);
WriteByte(0x00e5, 0x02);
WriteByte(0x00e6, 0x01);
WriteByte(0x00e7, 0x03);
WriteByte(0x00f5, 0x02);
WriteByte(0x00d9, 0x05);
WriteByte(0x00db, 0xce);
WriteByte(0x00dc, 0x03);
WriteByte(0x00dd, 0xf8);
WriteByte(0x009f, 0x00);
WriteByte(0x00a3, 0x3c);
WriteByte(0x00b7, 0x00);
WriteByte(0x00bb, 0x3c);
WriteByte(0x00b2, 0x09);
WriteByte(0x00ca, 0x09);
WriteByte(0x0198, 0x01);
WriteByte(0x01b0, 0x17);
WriteByte(0x01ad, 0x00);
WriteByte(0x00ff, 0x05);
WriteByte(0x0100, 0x05);
WriteByte(0x0199, 0x05);
WriteByte(0x01a6, 0x1b);
WriteByte(0x01ac, 0x3e);
WriteByte(0x01a7, 0x1f);
WriteByte(0x0030, 0x00);*/


/*************************************************************************
	I2C Protocol Code 
*************************************************************************/
void i2c_init(void)
{
	//set TWI bit in Power Reduction Register to "0"
	PRR &= ~(1 << PRTWI);
	//Initialize SCL to 200Khz
	TWSR = 0;
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
}

unsigned char i2c_start(unsigned char address)
{
	uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	//Check value of TWI Status Register, make sure status is START
	if (((TWSR & 0xF8) != START) && ((TWSR & 0xF8) != REP_START)) return 2;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TWSR & 0xF8;
	if ( (twst != MT_SLA_ACK) && (twst != MR_SLA_ACK) ) return TWSR;

	return 0;
}

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TWSR & 0xF8;
    	if ( (twst != START) && (twst != REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TWSR & 0xF8;
    	if ( (twst == MT_SLA_NOACK )||(twst == MR_DATA_NOACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */

/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
	return i2c_start( address );
}

/*************************************************************************
	release SDA
*************************************************************************/
void i2c_stop(void)
{
	/* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));
}

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data)
{	
    uint8_t twst;
	
	
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
		
	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));
		
	// check value of TWI Status Register. Mask prescaler bits
	twst = TWSR & 0xF8;
	if( twst != MT_DATA_ACK)
		return 1;
	
		
	return 0;
}

unsigned char WriteByte(unsigned int reg, unsigned char data)
{
	unsigned char data_write[3];
	
	
	data_write[0] = (unsigned char)((reg >> 8) & 0xFF);
	data_write[1] = (unsigned char)(reg & 0xFF);
	data_write[2] = data & 0xFF;
	
	for (uint8_t j=0; j<3; j++)
	{
		if(i2c_write(data_write[j]))
		{
			//Error_Handler(data_write[j], 1);
			return j;
		}
	}
	i2c_stop();
	
	
	
	return 0;
	
}

/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	  

    return TWDR;
}

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
	
    return TWDR;
}

//On error returns 0
//On success, return the address of the device 
unsigned char i2c_find_addr(void)
{
	
	unsigned char ret = 0x00;	
	
	for (unsigned char i=0x52; i<0x60; i = i+2)
	{
		ret = i2c_start(i);
		if(ret)
		{
			i2c_stop();
			//Set_Led(0x44);
		}
		else
		{
			i2c_write((unsigned char)(VL_CHANGE_ADDR_REG>>8));
			i2c_write((unsigned char)(VL_CHANGE_ADDR_REG));
			i2c_rep_start(i+1);
			ret = i2c_readNak();
			//i2c_stop();
			//Set_Led(ret);
			//_delay_ms(8000);

			return ret;
		}
		//break;
	}
	return 0; //NOTE no device found on bus
}
////////////////////////////////////////////////////////////////////////
//End I2C Protocol Code
////////////////////////////////////////////////////////////////////////


/*************************************************************************
	USART Code for SPI USB RS232 to serial com port  
*************************************************************************/
 void usart_init( unsigned int ubrr)
{
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	// Enable receiver and transmitter pg 247
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	// Set frame format: Asynchronous, 8data, 1stop bit pg 249
	UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);
}

void usart_putchar(unsigned char data)
{
	// Wait for empty transmit buffer
	while ( !(UCSR0A & (1<<UDRE0)) );
	// Start transmission
	UDR0 = data;
}

unsigned char usart_getchar(void)
{
	// Wait for incoming data
	while ( !(UCSR0A & (1<<RXC0)) );
	// Return the data
	return UDR0;
}

void usart_string(char s[], uint8_t len)
{
	int i =0;
	for(int i = 0; i<len; i++){
		usart_putchar(s[i]);
	}
}

void VL6180X_Initialize_OG(unsigned char devaddrW)
{
	unsigned char devaddrR = devaddrW + 1;
	
	unsigned char ret;
	ret = i2c_start(devaddrW);
	if(ret)
	{
		i2c_stop();
		usart_putchar(0xEE);
		//Set_Led(0xCC);
	}
	else
	{
		usart_putchar(0x99);
		i2c_write((unsigned char)(VL_SYSTEM__FRESH_OUT_OF_RESET>>8));
		i2c_write((unsigned char)(VL_SYSTEM__FRESH_OUT_OF_RESET));
		i2c_rep_start(devaddrR);
		ret = i2c_readNak();
		i2c_stop();
		
		if(ret == 1)
		{
			i2c_start(devaddrW);
			
			unsigned char passdata[2] = {0x01, 0x01};
			WriteData(0x0207, passdata, 2);
			
			i2c_start_wait(devaddrW);
			passdata[0] = 0x00;
			passdata[1] = 0xFD;
			WriteData(0x0096, passdata, 2);
			
			i2c_start_wait(devaddrW);
			unsigned char passdata2[5] = {0x00, 0x04, 0x02, 0x01, 0x03};
			WriteData(0x00E3, passdata2, 5);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00F5, 0x02);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00D9, 0x05);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00DB, 0xCE);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00Dc, 0x03);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00DD, 0xF8);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x009F, 0x00);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00A3, 0x3C);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00B7, 0x00);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00BB, 0x3C);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00B2, 0x09);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00CA, 0x09);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x0198, 0x01);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x01B0, 0x17);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x01AD, 0x00);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x00FF, 0x05);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x0100, 0x05);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x0199, 0x05);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x01A6, 0x1B);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x01AC, 0x3E);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x01A7, 0x1F);
			
			i2c_start_wait(devaddrW);
			WriteByte(0x0030, 0x00);
			
			//Set VL_SYSTEM__FRESH_OUT_OF_RESET to false; initialization complete
			i2c_start_wait(devaddrW);
			WriteByte(VL_SYSTEM__FRESH_OUT_OF_RESET, 0x00);
			//i2c_write(0x00);
			//i2c_stop();
		}
		
	}
}

unsigned char WriteData(unsigned int reg, unsigned char *data, unsigned char len)
{
	unsigned char data_write[len + 2];
	
	
	data_write[0] = (unsigned char)((reg >> 8) & 0xFF);
	data_write[1] = (unsigned char)(reg & 0xFF);
	for (uint8_t i=2; i<len; i++)
	{
		data_write[i] = *data & 0xFF;
		data++;
	}
	
	for (uint8_t j=0; j<len; j++)
	{
		if(i2c_write(data_write[j]))
		{
			return 1;
		}
	}
	
	i2c_stop();
	
	return 0;
	
}

unsigned char BNO055_Init(void){
	
	unsigned char ret = 0x00;
	ret =i2c_start(BNO055_addr);
	if(ret){
		usart_putchar(0xEE);
		usart_putchar(ret);
		i2c_stop();
	return 0;
	}
	i2c_stop();
	setMode(BNO055_OPERATION_MODE_CONFIG);
	
	uint8_t id = i2c_read8_from_dev(BNO055_addr, BNO055_CHIP_ID_ADDR);
	if(id != BNO055_CHIP_ID)
	{
		_delay_ms(256);
		_delay_ms(200);
		id = i2c_read8_from_dev(BNO055_addr, BNO055_CHIP_ID_ADDR);
		usart_putchar(id);
		if(id != BNO055_CHIP_ID)
		{
			_delay_ms(500);
			_delay_ms(500);
			_delay_ms(500);
			id = i2c_read8_from_dev(BNO055_addr, BNO055_CHIP_ID_ADDR);
			if(id != BNO055_CHIP_ID){
				while(ret < 20){
					usart_putchar(0xEE);
					//usart_putchar("");
					ret++;
				}
				ret = 0;
				//return 0;
			}
		}
	}
	//Set mode to CONFIGMODE
	setMode(BNO055_OPERATION_MODE_CONFIG);
	usart_putchar(0x07);
	//Reset
/*	i2c_write8_to_dev(BNO055_addr, BNO055_SYS_TRIGGER_ADDR, 0x20);*/
	usart_putchar(0x08);
// 	do{
// 		ret = i2c_read8_from_dev(BNO055_addr, BNO055_CHIP_ID_ADDR);
// 		usart_putchar(ret);
// 	}while(ret != BNO055_CHIP_ID);
	VL6180X_Single_Range();
	VL6180X_Single_Range();
	VL6180X_Single_Range();
	VL6180X_Single_Range();
	VL6180X_Single_Range();
	VL6180X_Single_Range();
	VL6180X_Single_Range();
	_delay_ms(50);
	usart_putchar(0x0B);
	if(i2c_write8_to_dev(BNO055_addr,  BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL)){
		usart_putchar(0x44);
	}

	if(i2c_write8_to_dev(BNO055_addr, BNO055_PAGE_ID_ADDR, 0X00)){
		usart_putchar(0x45);
	}

	if(i2c_write8_to_dev(BNO055_addr, BNO055_SYS_TRIGGER_ADDR, 0X00)){
		usart_putchar(0x46);
	}

	_delay_ms(50);

	setMode(BNO055_OPERATION_MODE_NDOF);

	usart_putchar(0x99);
	return 1;
	
}

void setExtCrystalUse(uint8_t usextal)
{
	uint8_t modeback = i2c_read8_from_dev(BNO055_addr, BNO055_OPR_MODE);

	/* Switch to config mode (just in case since this is the default) */
	setMode(BNO055_OPERATION_MODE_CONFIG);
	_delay_ms(25);
	if (usextal) {
		i2c_write8_to_dev(BNO055_addr, BNO055_SYS_TRIGGER_ADDR, 0x80);
	}
	else {
		i2c_write8_to_dev(BNO055_addr, BNO055_SYS_TRIGGER_ADDR, 0x00);
	}
	_delay_ms(20);
	/* Set the requested operating mode (see section 3.3) */
	setMode(modeback);
	_delay_ms(20);
}
void setMode(uint8_t modeSelect){
	i2c_write8_to_dev(BNO055_addr, BNO055_OPR_MODE, modeSelect);
}
unsigned char i2c_write8_to_dev(unsigned char i2c_addr, unsigned char reg_addr, unsigned char data){
	unsigned char ret = 0;
	
	ret = i2c_start(i2c_addr);
	if(ret){
		i2c_stop();
		return ret;
	}
	else{
		i2c_write(reg_addr);
		i2c_write(data);
		i2c_stop();
		return 0;
	}
}
unsigned char i2c_read8_from_dev(unsigned char i2c_addr, unsigned char reg_addr){
	unsigned char ret = 0;
	
	ret = i2c_start(i2c_addr);
	if(ret){
		i2c_stop();
		return ret;
	}
	else{
		i2c_write(reg_addr);
		i2c_rep_start(i2c_addr+1);
		ret = i2c_readNak();
		i2c_stop();
		
		return ret;
	}
}

unsigned char i2c_readX_from_dev(unsigned char i2c_addr, unsigned char reg_addr, unsigned char* buffer, unsigned char len)
{
	unsigned char ret = 0;
	unsigned char count = 0;
	ret = i2c_start(i2c_addr);
	if(ret){
		i2c_stop();
		return ret;
	}
	else{
		i2c_write(reg_addr);
		i2c_rep_start(i2c_addr+1);
		//i2c_write(reg_addr);
		while(count < (len-1)){
			buffer[count++]= i2c_readAck();
		}
		buffer[count] = i2c_readNak();
		i2c_stop();
		
		return 0;
	}	
}

unsigned char BNO055_GetQuat(void){
	unsigned char buffer[8];
	memset(buffer, 0, 8);
	i2c_readX_from_dev(BNO055_addr, BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
	
	//Add Scaler component
// 	short x,y,z,w;
// 	x = 0;
// 	y = 0;
// 	z = 0;
// 	w = 0;
// 	
// 	x = (((short)buffer[3]) << 8) | ((short)buffer[2]);
// 	y = (((short)buffer[5]) << 8) | ((short)buffer[4]);
// 	z = (((short)buffer[7]) << 8) | ((short)buffer[6]);
// 	w = (((short)buffer[1]) << 8) | ((short)buffer[0]);
// 	
// 	x = x*scale;
// 	y = y*scale;
// 	z = z*scale;
// 	w = w*scale;
	
	//Send MSB-LSB x,y,z,w
	
	//X
	usart_putchar(buffer[3]);
	usart_putchar(buffer[2]);
	
	//Y
	usart_putchar(buffer[5]);
	usart_putchar(buffer[4]);
	
	//Z
	usart_putchar(buffer[7]);
	usart_putchar(buffer[6]);
	
	//W
	usart_putchar(buffer[1]);
	usart_putchar(buffer[0]);	



	
}

void BNO055_GetQuat_delayed(){
	memset(buffer, 0, 8);
	i2c_readX_from_dev(BNO055_addr, BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
	
	//Add Scaler component
	// 	short x,y,z,w;
	// 	x = 0;
	// 	y = 0;
	// 	z = 0;
	// 	w = 0;
	//
	// 	x = (((short)buffer[3]) << 8) | ((short)buffer[2]);
	// 	y = (((short)buffer[5]) << 8) | ((short)buffer[4]);
	// 	z = (((short)buffer[7]) << 8) | ((short)buffer[6]);
	// 	w = (((short)buffer[1]) << 8) | ((short)buffer[0]);
	//
	// 	x = x*scale;
	// 	y = y*scale;
	// 	z = z*scale;
	// 	w = w*scale;
	
	//Send MSB-LSB x,y,z,w
	
}
void BNO055_GetEUL_delayed(void){
	memset(buffer, 0, 8);
	memset(_xyz, 0, 32*3);
	int16_t x, y, z;
	x = y = z = 0;

	/* Read vector data (6 bytes) */
	i2c_readX_from_dev(BNO055_addr, BNO055_EUL_DATA_X_LSB, buffer, 6);

	x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
	y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
	z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
	
	_xyz[0] = ((double)x)/16.0;
	_xyz[1] = ((double)y)/16.0;
	_xyz[2] = ((double)z)/16.0;
}

void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
	uint8_t calData = i2c_read8_from_dev(BNO055_addr, BNO055_CALIB_STAT_ADDR);
	if (sys != NULL) {
		*sys = (calData >> 6) & 0x03;
		//TODO
		//usart_putchar(0x0B);
	}
	if (gyro != NULL) {
		*gyro = (calData >> 4) & 0x03;
	}
	if (accel != NULL) {
		*accel = (calData >> 2) & 0x03;
	}
	if (mag != NULL) {
		*mag = calData & 0x03;
	}
}
