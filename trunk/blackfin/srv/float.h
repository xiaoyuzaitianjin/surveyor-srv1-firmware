/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  A floating point emulation library. It emulates in software what an
 *  an FPU would do with the floats. Because the blackfin does not support
 *  64 bit calculations the mantissas in multiplication and division are
 *  shortened, losing some precision around the 4th-6th decimal digits
 *  STILL TESTING ... so far all the calculations have been correct but you never know
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Some defines to make it easier to do arithmetic with floats and ints
#define addFInt(f1,int1)    addFloat(f1,intToFloat(int1))
#define subFInt(f1,int1)    subFloat(f1,intToFloat(int1))
#define mulFInt(f1,int1)    mulFloat(f1,intToFloat(int1))
#define divFInt(f1,int1)    divFloat(f1,intToFloat(int1))

typedef struct
{
	union{
	    struct {
           unsigned long mantissa: 23;
           unsigned long exponent: 8;
           unsigned long sign: 1;
       } float_parts;   //the struct shares same memory space as the float
                        //allowing us to access its parts with the bitfields

		float all;

	};

}_float; //__attribute__((__packed__)); compiler says packed attribute is ignored by the bfin-gcc compiler



int isOne(unsigned char b,int bit);
void printBinary(unsigned char b,int startbit,int endbit);
void parseFloat(float ff,_float* f);
void bytesToFloat(unsigned char* bytes, _float* f);
_float intToFloat(int number);

_float addFloat(_float f1,_float f2);
_float subFloat(_float f1,_float f2);
_float mulFloat(_float f1,_float f2);//the essence of the whole thing
_float divFloat(_float f1,_float f2);
_float  minus(_float f1);
int     gtFloat(_float f1,_float f2);
int     ltFloat(_float f1,_float f2);

void printMantissa(_float f);
void printExponent(_float f);
void floatInBinary(float f);
void printSign(_float f);
void printLong(unsigned long a);
void printLLong(unsigned long long a);

//The power(x,y) function I use here is just a function I have to find the power of x raised to y(for ints)
//it's not include in this library, but you can make one quite easily.
void printFloat(_float f,int decimaldigits); 

