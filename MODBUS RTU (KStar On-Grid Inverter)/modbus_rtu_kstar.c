#define SET                                     !PINB3_bit
#define INC                                     !PINB2_bit
#define DEC                                     !PINB1_bit
#define ESC                                     !PINB0_bit

#define REDE                                    PORTD2_bit

#define DIR_RX                                  0
#define DIR_TX                                  1

#define TX_buffer_length                        10
#define RX_buffer_length                        140
#define fixed_no_of_bytes_to_read               5

#define key_delay                               120
#define screen_delay                            2600


sbit LCD_RS at PORTD6_bit;
sbit LCD_EN at PORTD4_bit;
sbit LCD_D4 at PORTB4_bit;
sbit LCD_D5 at PORTB5_bit;
sbit LCD_D6 at PORTB6_bit;
sbit LCD_D7 at PORTB7_bit;

sbit LCD_RS_Direction at DDD6_bit;
sbit LCD_EN_Direction at DDD4_bit;
sbit LCD_D4_Direction at DDB4_bit;
sbit LCD_D5_Direction at DDB5_bit;
sbit LCD_D6_Direction at DDB6_bit;
sbit LCD_D7_Direction at DDB7_bit;


unsigned char cnt = 0;
volatile unsigned char TX_buffer[TX_buffer_length];
volatile unsigned char RX_buffer[RX_buffer_length];


void setup(void);
void flush_RX_buffer(void);
void flush_TX_buffer(void);
void send_read_command(void);
unsigned int make_word(unsigned char HB, unsigned char LB);
void get_HB_LB(unsigned int value, unsigned char *HB, unsigned char *LB);
unsigned int MODBUS_RTU_CRC16(unsigned char *data_input, unsigned char data_length);
signed int inc_dec(signed int value, signed int value_max, signed int value_min, unsigned char x_pos, unsigned char y_pos);
void display_value(unsigned int value, unsigned char x_pos, unsigned char y_pos);
void lcd_str(unsigned char x_pos, unsigned char y_pos, char *ch);
void print_C(unsigned char x_pos, unsigned char y_pos, signed int value);
void print_I(unsigned char x_pos, unsigned char y_pos, signed long value);
void print_D(unsigned char x_pos, unsigned char y_pos, signed int value, unsigned char points);
void print_F(unsigned char x_pos, unsigned char y_pos, float value, unsigned char points);
void page_pv_v(void);
void page_pv_i(void);
void page_pv_p(void);
void page_ac_v(void);
void page_ac_i(void);
void page_ac_f(void);
void page_ac_p(void);
void page_energy(void);


void UART1_Interrupt()
iv IVT_ADDR_USART_RXC
ics ICS_AUTO
{
    RX_buffer[cnt++] = UDR;
    RXC_bit = 0;
}



void main(void)
{
     bit sent;

     unsigned char temp_HB = 0;
     unsigned char temp_LB = 0;

     unsigned int temp = 0;
     unsigned int check1 = 0;
     unsigned int check2 = 0;

     setup();

     flush_TX_buffer();
     flush_RX_buffer();

     while(1)
     {
         send_read_command();
         page_pv_v();
         page_pv_i();
         page_pv_p();
         page_ac_v();
         page_ac_i();
         page_ac_f();
         page_ac_p();
         page_energy();
     };
}


void setup(void)
{
    PORTA = 0x00;
    DDRA = 0x00;
    PORTB = 0x0F;
    DDRB = 0xF0;
    PORTC = 0x00;
    DDRC = 0x00;
    PORTD = 0x00;
    DDRD = 0x76;
    TCCR0 = 0x00;
    TCNT0 = 0x00;
    OCR0 = 0x00;
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;
    ASSR = 0x00;
    TCCR2 = 0x00;
    TCNT2 = 0x00;
    OCR2 = 0x00;
    MCUCR = 0x00;
    MCUCSR = 0x00;
    TIMSK = 0x00;
    UCSRA = 0x00;
    UCSRB = 0x18;
    UCSRC = 0x86;
    UBRRH = 0x00;
    UBRRL = 0x0C;
    ACSR = 0x80;
    SFIOR = 0x00;
    ADCSRA = 0x00;
    SPCR = 0x00;
    TWCR = 0x00;

    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);
    UART1_Init_Advanced(9600, _UART_NOPARITY, _UART_ONE_STOPBIT);
    RXCIE_bit = 1;
    TXCIE_bit = 0;
    TXEN_bit = 1;
    SREG_I_bit = 1;
    delay_ms(100);
}


void flush_RX_buffer(void)
{
    signed char i = (RX_buffer_length - 1);

    while(i > -1)
    {
        RX_buffer[i] = 0x00;
        i--;
    };
}


void flush_TX_buffer(void)
{
    signed char i = (TX_buffer_length - 1);

    while(i > -1)
    {
        TX_buffer[i] = 0x00;
        i--;
    };
}


void send_read_command(void)
{
    unsigned char i = 0;
    
    flush_TX_buffer();

    TX_buffer[0] = 0x01;
    TX_buffer[1] = 0x04;
    TX_buffer[2] = 0x0B;
    TX_buffer[3] = 0xB8;
    TX_buffer[4] = 0x00;
    TX_buffer[5] = 0x3F;
    TX_buffer[6] = 0x32;
    TX_buffer[7] = 0x1B;
    
    REDE = DIR_TX;
    
    for(i = 0; i < TX_buffer_length; i++)
    {
        UART_Write(TX_buffer[i]);
    }

    cnt = 0;
    REDE = DIR_RX;
    
    delay_ms(1000);
}


unsigned int make_word(unsigned char HB, unsigned char LB)
{
    unsigned int tmp = 0;

    tmp = HB;
    tmp <<= 8;
    tmp |= LB;

    return tmp;
}


void get_HB_LB(unsigned int value, unsigned char *HB, unsigned char *LB)
{
    *LB = (value & 0x00FF);
    *HB = ((value & 0xFF00) >> 8);
}


unsigned int MODBUS_RTU_CRC16(unsigned char *data_input, unsigned char data_length)
{
    unsigned char n = 8;
    unsigned char s = 0;
    unsigned int CRC_word = 0xFFFF;

    for(s = 0; s < data_length; s++)
    {
      CRC_word ^= ((unsigned int)data_input[s]);

      n = 8;
      while(n > 0)
      {
        if((CRC_word & 0x0001) == 0)
        {
          CRC_word >>= 1;
        }

        else
        {
          CRC_word >>= 1;
          CRC_word ^= 0xA001;
        }

        n--;
      }
    }

    return CRC_word;
}


signed int inc_dec(signed int value, signed int value_max, signed int value_min, unsigned char x_pos, unsigned char y_pos)
{
    while(1)
    {
       if(INC)
       {
           value++;
       }
       if(value > value_max)
       {
           value = value_min;
       }

       if(DEC)
       {
           value--;
       }
       if(value < value_min)
       {
           value = value_max;
       }

       lcd_out(y_pos, x_pos, "   ");
       display_value(value, x_pos, y_pos);
       delay_ms(key_delay);

       if(SET)
       {
           delay_ms(key_delay);
           while(SET);
           return value;
       }
    }
}


void display_value(unsigned int value, unsigned char x_pos, unsigned char y_pos)
{
    unsigned char ch = 0x00;

    ch = (value / 10);
    lcd_chr(y_pos, x_pos, (ch + 0x30));
    ch = (value % 10);
    lcd_chr(y_pos, (x_pos + 1), (ch + 0x30));
}


void lcd_str(unsigned char x_pos, unsigned char y_pos, char *ch)
{
    do
    {
        Lcd_Chr(y_pos, x_pos++, *ch++);
    }while(*ch != '\0');
}


void print_C(unsigned char x_pos, unsigned char y_pos, signed int value)
{
     unsigned char ch[5] = {0x20, 0x20, 0x20, 0x20, '\0'};

     if(value < 0x00)
     {
        ch[0] = 0x2D;
        value = -value;
     }
     else
     {
        ch[0] = 0x20;
     }

     if((value > 99) && (value <= 999))
     {
         ch[1] = ((value / 100) + 0x30);
         ch[2] = (((value % 100) / 10) + 0x30);
         ch[3] = ((value % 10) + 0x30);
     }
     else if((value > 9) && (value <= 99))
     {
         ch[1] = (((value % 100) / 10) + 0x30);
         ch[2] = ((value % 10) + 0x30);
         ch[3] = 0x20;
     }
     else if((value >= 0) && (value <= 9))
     {
         ch[1] = ((value % 10) + 0x30);
         ch[2] = 0x20;
         ch[3] = 0x20;
     }

     lcd_str(x_pos, y_pos, ch);
}


void print_I(unsigned char x_pos, unsigned char y_pos, signed long value)
{
    unsigned char ch[7] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, '\0'};

    if(value < 0)
    {
        ch[0] = 0x2D;
        value = -value;
    }
    else
    {
        ch[0] = 0x20;
    }

    if(value > 9999)
    {
        ch[1] = ((value / 10000) + 0x30);
        ch[2] = (((value % 10000)/ 1000) + 0x30);
        ch[3] = (((value % 1000) / 100) + 0x30);
        ch[4] = (((value % 100) / 10) + 0x30);
        ch[5] = ((value % 10) + 0x30);
    }

    else if((value > 999) && (value <= 9999))
    {
        ch[1] = (((value % 10000)/ 1000) + 0x30);
        ch[2] = (((value % 1000) / 100) + 0x30);
        ch[3] = (((value % 100) / 10) + 0x30);
        ch[4] = ((value % 10) + 0x30);
        ch[5] = 0x20;
    }
    else if((value > 99) && (value <= 999))
    {
        ch[1] = (((value % 1000) / 100) + 0x30);
        ch[2] = (((value % 100) / 10) + 0x30);
        ch[3] = ((value % 10) + 0x30);
        ch[4] = 0x20;
        ch[5] = 0x20;
    }
    else if((value > 9) && (value <= 99))
    {
        ch[1] = (((value % 100) / 10) + 0x30);
        ch[2] = ((value % 10) + 0x30);
        ch[3] = 0x20;
        ch[4] = 0x20;
        ch[5] = 0x20;
    }
    else
    {
        ch[1] = ((value % 10) + 0x30);
        ch[2] = 0x20;
        ch[3] = 0x20;
        ch[4] = 0x20;
        ch[5] = 0x20;
    }

    lcd_str(x_pos, y_pos, ch);
}


void print_D(unsigned char x_pos, unsigned char y_pos, signed int value, unsigned char points)
{
    char ch[4] = {0x2E, 0x20, 0x20, '\0'};

    ch[1] = ((value / 10) + 0x30);

    if(points > 1)
    {
        ch[2] = ((value % 10) + 0x30);
    }

    lcd_str(x_pos, y_pos, ch);
}


void print_F(unsigned char x_pos, unsigned char y_pos, float value, unsigned char points)
{
    signed long tmp = 0x0000;

    tmp = value;
    print_I(x_pos, y_pos, tmp);
    tmp = ((value - tmp) * 100);

    if(tmp < 0)
    {
       tmp = -tmp;
    }

    if(value < 0)
    {
        value = -value;
        Lcd_Chr(y_pos, x_pos, 0x2D);
    }
    else
    {
        Lcd_Chr(y_pos, x_pos, 0x20);
    }

    if((value >= 10000) && (value < 100000))
    {
        print_D((x_pos + 6), y_pos, tmp, points);
    }
    else if((value >= 1000) && (value < 10000))
    {
        print_D((x_pos + 5), y_pos, tmp, points);
    }
    else if((value >= 100) && (value < 1000))
    {
        print_D((x_pos + 4), y_pos, tmp, points);
    }
    else if((value >= 10) && (value < 100))
    {
        print_D((x_pos + 3), y_pos, tmp, points);
    }
    else if(value < 10)
    {
        print_D((x_pos + 2), y_pos, tmp, points);
    }
}


void page_pv_v(void)
{
    float value = 0.0;
    
    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 5, "DC Voltage/V");
    lcd_out(2, 2, "PV 1");
    lcd_out(3, 2, "PV 2");
    lcd_out(4, 2, "PV 3");
    
    value = make_word(RX_buffer[3], RX_buffer[4]);
    value /= 10.0;
    print_F(14, 2, value, 1);

    value = make_word(RX_buffer[5], RX_buffer[6]);
    value /= 10.0;
    print_F(14, 3, value, 1);

    value = make_word(RX_buffer[7], RX_buffer[8]);
    value /= 10.0;
    print_F(14, 4, value, 1);
    
    delay_ms(screen_delay);
}


void page_pv_i(void)
{
    float value = 0.0;
    
    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 5, "DC Current/A");
    lcd_out(2, 2, "PV 1");
    lcd_out(3, 2, "PV 2");
    lcd_out(4, 2, "PV 3");

    value = make_word(RX_buffer[9], RX_buffer[10]);
    value /= 100.0;
    print_F(15, 2, value, 2);

    value = make_word(RX_buffer[11], RX_buffer[12]);
    value /= 100.0;
    print_F(15, 3, value, 2);

    value = make_word(RX_buffer[13], RX_buffer[14]);
    value /= 100.0;
    print_F(15, 4, value, 2);
    
    delay_ms(screen_delay);
}


void page_pv_p(void)
{
    float value = 0.0;
    
    unsigned long value1 = 0;
    unsigned long value2 = 0;
    
    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 6, "DC Power/W");
    lcd_out(2, 2, "PV 1");
    lcd_out(3, 2, "PV 2");
    lcd_out(4, 2, "PV 3");
    
    value1 = make_word(RX_buffer[15], RX_buffer[16]);
    value2 = make_word(RX_buffer[17], RX_buffer[18]);
    value = ((value1 << 16) | value2);
    print_F(14, 2, value, 1);

    value1 = make_word(RX_buffer[19], RX_buffer[20]);
    value2 = make_word(RX_buffer[21], RX_buffer[22]);
    value = ((value1 << 16) | value2);
    print_F(14, 3, value, 1);

    value1 = make_word(RX_buffer[23], RX_buffer[24]);
    value2 = make_word(RX_buffer[25], RX_buffer[26]);
    value = ((value1 << 16) | value2);
    print_F(14, 4, value, 1);
    
    delay_ms(screen_delay);
}


void page_ac_v(void)
{
    float value = 0.0;
    
    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 5, "AC Voltage/V");
    lcd_out(2, 2, "R");
    lcd_out(3, 2, "Y");
    lcd_out(4, 2, "B");
    
    value = make_word(RX_buffer[31], RX_buffer[32]);
    value /= 10.0;
    print_F(14, 2, value, 1);

    value = make_word(RX_buffer[33], RX_buffer[34]);
    value /= 10.0;
    print_F(14, 3, value, 1);

    value = make_word(RX_buffer[35], RX_buffer[36]);
    value /= 10.0;
    print_F(14, 4, value, 1);
    
    delay_ms(screen_delay);
}


void page_ac_i(void)
{
    float value = 0.0;
    
    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 5, "AC Current/A");
    lcd_out(2, 2, "R");
    lcd_out(3, 2, "Y");
    lcd_out(4, 2, "B");
    
    value = make_word(RX_buffer[43], RX_buffer[44]);
    value /= 100.0;
    print_F(15, 2, value, 2);

    value = make_word(RX_buffer[45], RX_buffer[46]);
    value /= 100.0;
    print_F(15, 3, value, 2);

    value = make_word(RX_buffer[47], RX_buffer[48]);
    value /= 100.0;
    print_F(15, 4, value, 2);
    
    delay_ms(screen_delay);
}


void page_ac_f(void)
{
    float value = 0.0;
    
    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 5, "AC Freq./Hz");
    lcd_out(2, 2, "R");
    lcd_out(3, 2, "Y");
    lcd_out(4, 2, "B");
    
    value = make_word(RX_buffer[37], RX_buffer[38]);
    value /= 100.0;
    print_F(15, 2, value, 1);

    value = make_word(RX_buffer[39], RX_buffer[40]);
    value /= 100.0;
    print_F(15, 3, value, 1);

    value = make_word(RX_buffer[41], RX_buffer[42]);
    value /= 100.0;
    print_F(15, 4, value, 1);
    
    delay_ms(screen_delay);
}


void page_ac_p(void)
{
    float value = 0.0;

    unsigned long value1 = 0;
    unsigned long value2 = 0;

    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 3, "Grid Parameters");
    lcd_out(2, 2, "Power/W");
    lcd_out(3, 2, "Pow.Fac");
    lcd_out(4, 2, "Temp/'C");
    
    value1 = make_word(RX_buffer[49], RX_buffer[50]);
    value2 = make_word(RX_buffer[51], RX_buffer[52]);
    value = ((value1 << 16) | value2);
    print_F(14, 2, value, 1);
    
    value = make_word(RX_buffer[115], RX_buffer[116]);
    value /= 1000.0;
    print_F(15, 3, value, 2);
    
    value = make_word(RX_buffer[53], RX_buffer[54]);
    value /= 10.0;
    print_F(15, 4, value, 1);
    
    delay_ms(screen_delay);
}


void page_energy(void)
{
    float value = 0.0;

    unsigned long value1 = 0;
    unsigned long value2 = 0;
    
    Lcd_Cmd(_LCD_CLEAR);
    lcd_out(1, 5, "Energy/kWHr");
    lcd_out(2, 2, "E. Total");
    lcd_out(3, 2, "E. Year");
    lcd_out(4, 2, "E. Today");
    
    value1 = make_word(RX_buffer[79], RX_buffer[80]);
    value2 = make_word(RX_buffer[81], RX_buffer[82]);
    value = ((value1 << 16) | value2);
    value /= 10.0;
    print_I(16, 2, value);
    
    value1 = make_word(RX_buffer[83], RX_buffer[84]);
    value2 = make_word(RX_buffer[85], RX_buffer[86]);
    value = ((value1 << 16) | value2);
    print_I(16, 3, value);
    
    value = make_word(RX_buffer[87], RX_buffer[88]);
    print_I(16, 4, value);
    
    delay_ms(screen_delay);
}