/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  A floating point emulation library. It emulates in software what an
 *  an FPU would do with the floats. It still needs to take in account the 
 *  special cases of NAN, +INF and -INF. Other than that, it seems to work fine
 *  Still in Testing mode so please, report any problems, with it to elefkar@it.teithe.gr 
 *  or http://www.surveyor.com/cgi-bin/yabb2/YaBB.pl forum
 *
 *  There are five distinct numerical ranges that single-precision floating-point 
 *  numbers are not able to represent:
 *
 *  1. Negative numbers less than -(2-2-23) × 2127 (negative overflow)
 *  2. Negative numbers greater than -2-149 (negative underflow)
 *  3. Zero
 *  4. Positive numbers less than 2-149 (positive underflow)
 *  5. Positive numbers greater than (2-2-23) × 2127 (positive overflow)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
//Some defines to make it easier to do arithmetic operations with floats and ints
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

}_float; //__attribute__((__packed__)); bfin-gcc ignores the attribute packed



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
int     gteFloat(_float f1,_float f2);
int     lteFloat(_float f1,_float f2);

void printMantissa(_float f);
void printExponent(_float f);
void floatInBinary(float f);


//WARNING: The floats printed by this function are an approximation of the real number, do not be alarmed
//if it is not exactly the number you were expecting For example 0.004231 could be printed as: 0.0042309
void printFloat(_float f,int decimaldigits); 

//A function to emulate a big binary multiplication since blackfin will throw undefined references
void bigMulti(unsigned long long m1,unsigned long long m2,unsigned long long* res);

//I put this function here so that it won't have any external dependencies, it is just a power(x raised to y)
// for integers function, which I usually have in my math.h . IF you have anything better just link to it
int power( int x,int p);

