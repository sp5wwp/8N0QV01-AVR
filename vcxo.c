#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>

#define BAUD 		9600
#define MYUBRR 		F_CPU/16/BAUD-1

#define I2C_BITRATE F_CPU/2/100000UL-8	//100kHz SCK
#define VCXO_ADDR	0b11011100

#define	F_LOW		1950
#define	F_HIGH		2600
const double F_REF = 114.285f; //MHz

struct femtoclock
{
	double frequency;
	
	uint8_t  P;
	uint8_t  N;
	uint8_t  MINT;
	uint32_t MFRAC;
	
	uint8_t  ADC_GAIN;
	
	uint8_t  error;
};

//I2C uC hardware initialization
void I2C_init(void)                            
{
    TWBR=0;//I2C_BITRATE; //bitrate
    TWSR|=(0<<TWPS1)|(0<<TWPS0); //prescaler=1
	TWCR|=(1<<TWEN);
}

//I2C START sequence
void I2C_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while(!(TWCR & (1<<TWINT)));
}

//write byte
void I2C_write(uint8_t data)
{           
    TWDR=data;        
    TWCR=(1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

//I2C STOP sequence
void I2C_stop(void)
{
    TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    while(TWCR & (1<<TWSTO));
}

//read with ACK
/*uint8_t I2C_read_ACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}*/

//read with NACK
uint8_t I2C_read_NACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

//get I2C status from uC register
/*uint8_t I2C_get_status(void)
{
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}*/

void I2C_addr_write(uint8_t dev_addr, uint8_t addr, uint8_t val)
{
	I2C_start();
	I2C_write(dev_addr);
	I2C_write(addr);
	I2C_write(val);
	I2C_stop();
}

//UART
void UART_Init(uint8_t ubrr)
{
	UBRRH = (uint8_t)(ubrr>>8);
	UBRRL = (uint8_t)ubrr;

	UCSRB = (1<<RXEN)|(1<<TXEN);
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void UART_TX(uint8_t data)
{
	while( !( UCSRA & (1<<UDRE)) );
	UDR = data;
}

void UART_TX_str(uint8_t* str)
{
	for(uint8_t i=0; i<strlen(str); i++)
		UART_TX(str[i]);
}

//VCXO
void VCXO_Init(struct femtoclock *clk)
{
	clk->error=0;
	clk->P=1;
	//clk->ADC_GAIN=7;
}

uint8_t VCXO_p(uint8_t pe)
{
	if(pe==1)
		return 0;
	if(pe==2)
		return 1;
	if(pe==4)
		return 2;
	if(pe==5)
		return 3;
}

void VCXO_SetFreq(struct femtoclock *clk, double freq)
{
	if (clk->error) return;
	
	for(uint8_t i=0; i<=127; i++)
	{
		if(i*freq<=F_HIGH && i*freq>=F_LOW)
		{
			if(!(i%2))
			{
				clk->N=i;
				break;
			}
		}
		
		if(i==127)
		{
			clk->error=1;
			return;
		}
	}
	
	uint64_t freq_ext=freq*1000000000000;
	uint64_t fref_ext=F_REF*1000000000000;
	uint64_t MFRAC_ext=0;
	
	clk->MINT=clk->N*freq/(F_REF*clk->P);
	
	MFRAC_ext=(freq_ext*clk->N)/F_REF-(clk->MINT)*1000000000000;
	MFRAC_ext-=1907348;
	MFRAC_ext/=3814697;
	
	clk->MFRAC=(uint32_t)MFRAC_ext;
	
	clk->frequency=freq;
	
	//setting via I2C
	uint8_t CP=0b00;
	uint8_t val=0;
	
	val=(CP<<6)|((clk->MINT&(0b011111))<<1)|(clk->MFRAC>>17);
	I2C_addr_write(VCXO_ADDR, 0, val);//
	I2C_addr_write(VCXO_ADDR, 1, val);//
	I2C_addr_write(VCXO_ADDR, 2, val);//---fill with the same data
	I2C_addr_write(VCXO_ADDR, 3, val);//
	
	val=(clk->MFRAC&(0xFF<<9))>>9;
	I2C_addr_write(VCXO_ADDR, 4, val);//
	I2C_addr_write(VCXO_ADDR, 5, val);//
	I2C_addr_write(VCXO_ADDR, 6, val);//---fill with the same data
	I2C_addr_write(VCXO_ADDR, 7, val);//
	
	val=(clk->MFRAC&(0xFF<<1))>>1;
	I2C_addr_write(VCXO_ADDR, 8, val);//
	I2C_addr_write(VCXO_ADDR, 9, val);//
	I2C_addr_write(VCXO_ADDR, 10, val);//---fill with the same data
	I2C_addr_write(VCXO_ADDR, 11, val);//
	
	val=((clk->MFRAC&1)<<7)|(clk->N&(0b1111111));
	I2C_addr_write(VCXO_ADDR, 12, val);//
	I2C_addr_write(VCXO_ADDR, 13, val);//
	I2C_addr_write(VCXO_ADDR, 14, val);//---fill with the same data
	I2C_addr_write(VCXO_ADDR, 15, val);//

	val=(VCXO_p(clk->P)<<6)|(clk->MINT&0b100000)|(0b11111);
	I2C_addr_write(VCXO_ADDR, 0b10100, val);
}

void VCXO_SetADC(struct femtoclock *clk, uint8_t a)
{
	clk->ADC_GAIN=a;

	uint8_t val=(a<<2)|1;
	I2C_addr_write(VCXO_ADDR, 0b10011, val);
}

int main(void)
{
	UART_Init(MYUBRR);
	I2C_init();
	
	_delay_ms(1000);
	
	struct femtoclock vcxo;
	
	VCXO_Init(&vcxo);
	VCXO_SetFreq(&vcxo, 145.3125);
	VCXO_SetADC(&vcxo, 12);
	
	while(1);
	
	return 0;
}
