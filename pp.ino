/*
 * pp programmer, for SW 0.99 and higher
 *
 *
 */


#include <avr/io.h>
#include <util/delay.h>

#if defined(ARDUINO_AVR_UNO)
    // Arduino UNO
    #define ISP_PORT  PORTC
    #define ISP_DDR   DDRC
    #define ISP_PIN   PINC
    #define ISP_MCLR  3     // A3 (PC3)
    #define ISP_DAT   1     // A1 (PC1)
    #define ISP_CLK   0     // A0 (PC0)
#elif defined(ARDUINO_AVR_LEONARDO)
    // Arduino Leonardo
    #define ISP_PORT  PORTF
    #define ISP_DDR   DDRF
    #define ISP_PIN   PINF
    #define ISP_MCLR  4     // A3 (PF4)
    #define ISP_DAT   6     // A1 (PF6)
    #define ISP_CLK   7     // A0 (PF7)
#else
    #error Unsupported board selection.
#endif

#define  ISP_MCLR_1 ISP_PORT |= (1<<ISP_MCLR);
#define  ISP_MCLR_0 ISP_PORT &= ~(1<<ISP_MCLR);
#define  ISP_MCLR_D_I ISP_DDR &= ~(1<<ISP_MCLR);
#define  ISP_MCLR_D_0 ISP_DDR |= (1<<ISP_MCLR);

#define  ISP_DAT_1 ISP_PORT |= (1<<ISP_DAT);
#define  ISP_DAT_0 ISP_PORT &= ~(1<<ISP_DAT);
#define  ISP_DAT_V (ISP_PIN&(1<<ISP_DAT))
#define  ISP_DAT_D_I ISP_DDR &= ~(1<<ISP_DAT);
#define  ISP_DAT_D_0 ISP_DDR |= (1<<ISP_DAT);

#define  ISP_CLK_1 ISP_PORT |= (1<<ISP_CLK);
#define  ISP_CLK_0 ISP_PORT &= ~(1<<ISP_CLK);
#define  ISP_CLK_D_I ISP_DDR &= ~(1<<ISP_CLK);
#define  ISP_CLK_D_0 ISP_DDR |= (1<<ISP_CLK);

#define  ISP_CLK_DELAY  1
void isp_send (unsigned int data, unsigned char n);
unsigned int isp_read_16 (void);
//unsigned char enter_progmode (void);
unsigned char exit_progmode (void);
//void isp_read_pgm (unsigned int * data, unsigned char n);
//void isp_write_pgm (unsigned int * data, unsigned char n);
//void isp_mass_erase (void);
//void isp_reset_pointer (void);
void isp_send_8_msb (unsigned char data);
unsigned int isp_read_8_msb (void);
unsigned int isp_read_16_msb (void);
unsigned char p16c_enter_progmode (void);
void p16c_set_pc (unsigned long pc);
void p16c_bulk_erase (void);
void p16c_load_nvm (unsigned char inc, unsigned int data);
unsigned int p16c_read_data_nvm (unsigned char inc);
void p16c_begin_prog (unsigned char cfg_bit);
void p16c_isp_write_cfg (unsigned int data, unsigned int addr);
void p18q_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n);
void p18q_isp_write_cfg (unsigned char data, unsigned long addr);

//unsigned char p18_enter_progmode (void);
//unsigned int p18_get_ID (void);
void p18_send_cmd_payload (unsigned char cmd, unsigned int payload);
//unsigned int p18_get_cmd_payload (unsigned char cmd);
unsigned int isp_read_8 (void);
//void p18_set_tblptr (unsigned long val);
unsigned char p18_read_pgm_byte (void);
//void p_18_modfied_nop (void);
//void p18_isp_mass_erase (void);
//void p18fk_isp_mass_erase (unsigned char data1, unsigned char data2, unsigned char data3);


void usart_tx_b(uint8_t data);
uint8_t usart_rx_rdy(void);
uint8_t usart_rx_b(void);
void usart_tx_s(uint8_t * data);

#define _BAUD	57600	// Baud rate (9600 is default)
#define _UBRR	(F_CPU/16)/_BAUD - 1	// Used for UBRRL and UBRRH

unsigned int dat;
unsigned char rx,i,main_state,bytes_to_receive,rx_state;
unsigned char rx_message[280],rx_message_ptr;
unsigned int flash_buffer[260];
unsigned int test,cfg_val;
unsigned long addr;

void setup(void)
{
  // ************************** Added by J4F - 17/04/2021
  pinMode(13, OUTPUT);
  pinMode(A2, INPUT_PULLUP);
  while (digitalRead(A2))
  {
    digitalWrite(13, HIGH);
    _delay_ms(200);
    digitalWrite(13, LOW);
    _delay_ms(200);
  }
  // ************************** Enf of the added part

#if defined(ARDUINO_AVR_UNO)
  UBRR0H = ((_UBRR) & 0xF00);
  UBRR0L = (uint8_t) ((_UBRR) & 0xFF);
  UCSR0B |= _BV(TXEN0);
  UCSR0B |= _BV(RXEN0);
#endif

#if defined(ARDUINO_AVR_LEONARDO)
  Serial.begin(_BAUD);
  while (!Serial);
#endif

  ISP_CLK_D_0
  ISP_DAT_D_0
  ISP_DAT_0
  ISP_CLK_0
  ISP_MCLR_D_0
  ISP_MCLR_1
  rx_state = 0;

}

void loop()
{
    if (usart_rx_rdy())
      {
      rx = usart_rx_b();
      rx_state = rx_state_machine (rx_state,rx);
      if (rx_state==3)
        {
        if (rx_message[0]==0x02)
          {
          exit_progmode();
          usart_tx_b (0x82);
          rx_state = 0;
          }
        if (rx_message[0]==0x40)
          {
          p16c_enter_progmode();
          usart_tx_b (0xC0);
          rx_state = 0;
          }
        if (rx_message[0]==0x41)
          {
          usart_tx_b (0xC1);
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          p16c_isp_read_pgm (flash_buffer, addr, rx_message[2]);
          for (i=0;i<rx_message[2];i++)
            {
            usart_tx_b (flash_buffer[i]&0xFF);
            usart_tx_b (flash_buffer[i]>>8);
            }
          rx_state = 0;
          }
        if (rx_message[0]==0x43)
          {
          p16c_bulk_erase ();
          usart_tx_b (0xC3);
          rx_state = 0;
          }
        if (rx_message[0]==0x44)
          {
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          cfg_val = rx_message[6];
          cfg_val = (cfg_val<<8) + rx_message[7];
          p16c_isp_write_cfg (cfg_val, addr);
          usart_tx_b (0xC4);
          rx_state = 0;
          }
        if (rx_message[0]==0x45)
          {
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          p18q_isp_write_cfg (rx_message[6], addr);
          usart_tx_b (0xC5);
          rx_state = 0;
          }
        if (rx_message[0]==0x46)
          {
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          for (i=0;i<rx_message[2]/2;i++)
            {
            flash_buffer[i] = (((unsigned int)(rx_message[(2*i)+1+6]))<<8) + (((unsigned int)(rx_message[(2*i)+0+6]))<<0);
            }
          p18q_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
          usart_tx_b (0xC6);
          rx_state = 0;
          }
        if (rx_message[0]==0x47)
          {
          usart_tx_b (0xC7);
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          p18q_isp_read_cfg (flash_buffer, addr, rx_message[2]);
          for (i=0;i<rx_message[2];i++)
            {
            usart_tx_b (flash_buffer[i]&0xFF);
            }
          rx_state = 0;
          }
        }
      }
}



unsigned char rx_state_machine (unsigned char state, unsigned char rx_char)
{
if (state==0)
  {
    rx_message_ptr = 0;
    rx_message[rx_message_ptr++] = rx_char;
    return 1;
  }
if (state==1)
  {
    bytes_to_receive = rx_char;
    rx_message[rx_message_ptr++] = rx_char;
    if (bytes_to_receive==0) return 3;
    return 2;
  }
if (state==2)
  {
    rx_message[rx_message_ptr++] = rx_char;
    bytes_to_receive--;
    if (bytes_to_receive==0) return 3;
  }
return state;
}


unsigned int isp_read_16 (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<16;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out >> 1;
  if (ISP_DAT_V)
    out = out | 0x8000;
  }
  return out;
}

unsigned int isp_read_8 (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<8;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out >> 1;
  if (ISP_DAT_V)
    out = out | 0x80;
  }
  return out;
}

//unsigned int isp_read_14s (void)
//{
//unsigned char i;
//unsigned int out;
//out = isp_read_16();
//out = out &0x7FFE;
//out = out >> 1;
//return out;
//}



void isp_send (unsigned int data, unsigned char n)
{
unsigned char i;
ISP_DAT_D_0
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<n;i++)
  {
  if (data&0x01)
    {
    ISP_DAT_1
    }
  else
    {
    ISP_DAT_0
    }
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
//  _delay_us(ISP_CLK_DELAY);
  data = data >> 1;
  ISP_CLK_0
  ISP_DAT_0
//  _delay_us(ISP_CLK_DELAY);
  }
}


void isp_send_24_msb (unsigned long data)
{
unsigned char i;
ISP_DAT_D_0
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<23;i++)
  {
  if (data&0x400000)
    {
    ISP_DAT_1
    }
  else
    {
    ISP_DAT_0
    }
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  data = data << 1;
  ISP_CLK_0
//  _delay_us(ISP_CLK_DELAY);
  }
  ISP_DAT_0
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
}

void isp_send_8_msb (unsigned char data)
{
unsigned char i;
ISP_DAT_D_0
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<8;i++)
  {
  if (data&0x80)
    {
    ISP_DAT_1
    }
  else
    {
    ISP_DAT_0
    }
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  data = data << 1;
  ISP_CLK_0
  ISP_DAT_0
//  _delay_us(ISP_CLK_DELAY);
  }
}


unsigned int isp_read_8_msb (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<8;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out << 1;
  if (ISP_DAT_V)
    out = out | 0x1;
  }
  return out;
}

unsigned int isp_read_16_msb (void)
{
unsigned char i;
unsigned int out;
out = 0;
ISP_DAT_D_I
//_delay_us(3*ISP_CLK_DELAY);
for (i=0;i<16;i++)
  {
  ISP_CLK_1
  _delay_us(ISP_CLK_DELAY);
  ISP_CLK_0
  _delay_us(ISP_CLK_DELAY);
  out = out << 1;
  if (ISP_DAT_V)
    out = out | 0x1;
  }
  return out;
}

//unsigned char enter_progmode (void)
//{
//ISP_MCLR_0
//_delay_us(300);
//isp_send(0b01010000,8);
//isp_send(0b01001000,8);
//isp_send(0b01000011,8);
//isp_send(0b01001101,8);
//
//isp_send(0,1);
//
//}

/**************************************************************************************************************************/

unsigned char exit_progmode (void)
{
ISP_MCLR_1
_delay_ms(30);
ISP_MCLR_0
_delay_ms(30);
ISP_MCLR_1
}

//***********************************************************************************//

unsigned char p16c_enter_progmode (void)
{
ISP_MCLR_0
_delay_us(300);
isp_send_8_msb(0x4d);
isp_send_8_msb(0x43);
isp_send_8_msb(0x48);
isp_send_8_msb(0x50);
_delay_us(300);
}

void p16c_set_pc (unsigned long pc)
{
  isp_send_8_msb(0x80);
  _delay_us(2);
  isp_send_24_msb(pc);

}

void p16c_bulk_erase (void)
{
  isp_send_8_msb(0x18);
  _delay_us(2);
  isp_send_24_msb(0x00001e);    // Bit 3: Configuration memory Bit 1: Flash memory
  _delay_ms(11);

//  _delay_ms(100);
}

void p16c_load_nvm (unsigned int data, unsigned char inc)
{
  if (inc==0) isp_send_8_msb(0x00);
  else isp_send_8_msb(0x02);
  _delay_us(2);
  isp_send_24_msb(data);
  _delay_us(2);
  }

unsigned int p16c_read_data_nvm (unsigned char inc)   // 24bit (8 + 16) read
{
  unsigned int retval;
  unsigned char tmp;
  if (inc==0) isp_send_8_msb(0xFC);
  else isp_send_8_msb(0xFE);
  _delay_us(2);
  tmp = isp_read_8_msb();
  retval = isp_read_16_msb();
  retval = retval >> 1;
  if (tmp&0x01) retval = retval | 0x8000;
  return retval;
  }

unsigned int p16c_read_data_cfg (unsigned char inc)   // 24bit (16 + 8) read
{
  unsigned char retval;
  unsigned int tmp;
  if (inc==0) isp_send_8_msb(0xFC);
  else isp_send_8_msb(0xFE);
  _delay_us(2);
  tmp = isp_read_16_msb();
  retval = isp_read_8_msb();
  retval = retval >> 1;
  if (tmp&0x0001) retval = retval | 0x80;
  return retval;
  }

void p16c_begin_prog (unsigned char cfg_bit)
{
  isp_send_8_msb(0xE0);
  _delay_ms(3);
  if (cfg_bit!=0) _delay_ms(3);
  }

//unsigned int p16c_get_ID (void)
//{
//  p16c_set_pc(0x8006);
//  return p16c_read_data_nvm(1);
//  }

//void p16c_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n)
//{
//unsigned char i;
////_delay_us(3*ISP_CLK_DELAY);
//p16c_set_pc(addr);
//for (i=0;i<n;i++)
//  p16c_load_nvm(data[i],1);
//p16c_set_pc(addr);
//p16c_begin_prog(0);
//}

void p16c_isp_read_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
unsigned int tmp1,tmp2;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
for (i=0;i<n;i++)
  data[i] = p16c_read_data_nvm(1);
}

void p16c_isp_write_cfg (unsigned int data, unsigned long addr)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
p16c_load_nvm(data,0);
p16c_begin_prog(1);
}

void p18q_isp_write_pgm (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned char i;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
for (i=0;i<n;i++)
  {
  isp_send_8_msb(0xE0);
  _delay_us(2);
  isp_send_24_msb(data[i]);
  _delay_us(75);
//  _delay_us(65);
  }
}

void p18q_isp_write_cfg (unsigned char data, unsigned long addr)
{
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
isp_send_8_msb(0xE0);
_delay_us(2);
isp_send_24_msb(data);
_delay_ms(11);
//_delay_us(65);
}

void p18q_isp_read_cfg (unsigned int * data, unsigned long addr, unsigned char n)
{
unsigned int retval;
unsigned char tmp;
//_delay_us(3*ISP_CLK_DELAY);
p16c_set_pc(addr);
for (i=0;i<n;i++)
  {
  isp_send_8_msb(0xFE);
  _delay_us(2);
  tmp = isp_read_16_msb();
  retval = isp_read_8_msb();
  _delay_us(2);
  retval = retval >> 1;
  if (tmp&0x01) retval = retval | 0x80;
    data[i] = retval;
  }
}

void usart_tx_s(uint8_t * data)
{
while (*data!=0)
  usart_tx_b(*data++);
}


#if defined(ARDUINO_AVR_UNO)
void usart_tx_b(uint8_t data)
{
while (!(UCSR0A & _BV(UDRE0)));
UDR0 = data;
}
 
uint8_t usart_rx_rdy(void)
{
if (UCSR0A & _BV(RXC0))
  return 1;
else
  return 0;
}

uint8_t usart_rx_b(void)
{
return (uint8_t) UDR0;
}
#endif

#if defined(ARDUINO_AVR_LEONARDO)
void usart_tx_b(uint8_t data)
{
Serial.write(data);
}

uint8_t usart_rx_rdy(void)
{
return Serial.available();
}

uint8_t usart_rx_b(void)
{
return (uint8_t)Serial.read();
}
#endif


void usart_tx_hexa_8 (uint8_t value)
{
uint8_t temp;
temp = value;
usart_tx_b('0');
usart_tx_b('x');
usart_tx_hexa_8b(value);
usart_tx_b(' ');
}

void usart_tx_hexa_8b (uint8_t value)
{
uint8_t temp;
temp = value;
temp = ((temp>>4)&0x0F);
if (temp<10) temp = temp + '0';
else temp = temp + 'A'- 10;
usart_tx_b(temp);
temp = value;
temp = ((temp>>0)&0x0F);
if (temp<10) temp = temp + '0';
else temp = temp + 'A' - 10;
usart_tx_b(temp);
}


void usart_tx_hexa_16 (uint16_t value)
{
  usart_tx_b('0');
usart_tx_b('x');
  usart_tx_hexa_8b((value>>8)&0xFF);
  usart_tx_hexa_8b(value&0xFF);
  usart_tx_b(' ');
}
