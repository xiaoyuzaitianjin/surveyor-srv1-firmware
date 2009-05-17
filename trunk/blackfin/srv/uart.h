void init_uart0();
void init_uart1();

void uart0SendChar(unsigned char s);
void uart0SendString(unsigned char *s);
void uart0SendChars(unsigned char *s, unsigned int count);
unsigned char uart0GetCh();
unsigned char uart0GetChar(unsigned char *s);
unsigned char uart0Signal();
void uart1SendChar(unsigned char s);
void uart1SendString(unsigned char *s);
void uart1SendChars(unsigned char *s, unsigned int count);
unsigned char uart1GetCh();
unsigned char uart1GetChar(unsigned char *s);
unsigned char uart1Signal();
void printNumber(unsigned char base, unsigned char noDigits, unsigned char sign, unsigned char pad, int number);


