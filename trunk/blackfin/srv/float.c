/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  A floating point emulation library. It emulates in software what an
 *  an FPU would do with the floats. Because the blackfin does not support
 *  64 bit calculations the mantissas in multiplication and division are
 *  shortened, losing some precision around the 4th-6th decimal digits
 *  STILL TESTING ... so far all the calculations have been correct but you never know
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "float.h"

int power( int x,int p)
{
    int res = 1;
    
    if (p !=0) {
        while(p != 0) {
            res *= x;
            p--;
        }
        return res;
    } else
        return 1;
}
 
void bytesToFloat(unsigned char* buffer, _float* f)
{
    //f->all = *(float*)&bytes[0];
    unsigned char* fbytes = (unsigned char*)&f->all;
    fbytes[0] = buffer[0];
    fbytes[1] = buffer[1];
    fbytes[2] = buffer[2];
    fbytes[3] = buffer[3];
}


void parseFloat(float ff, _float* flow)
{
   unsigned char* bytes= (unsigned char*)&ff;

   flow->all = *(float*)&bytes[0];

    return;
}

_float  minus(_float f1)
{
    _float result = f1;
    result.float_parts.sign ^= 0x01;
    return result;
}


_float divFloat(_float f1,_float f2)
{
   // unsigned long long mm1,mm; |NOT SUPPORTED BY BLACKFIN|
   int i;
    unsigned long m1,m;

    char texp1,texp2;

    _float result;



    texp1 = f1.float_parts.exponent-127;
    texp2 = f2.float_parts.exponent-127;


    result.float_parts.exponent = texp1-texp2+127;

    //getting the hidden bit out for both of them
    f1.float_parts.mantissa = ((f1.float_parts.mantissa)>>1) | (0x01<<22);
    f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);


   // mm1 = f1.float_parts.mantissa; |NOT SUPPORTED BY BLACKFIN|
   // mm1 <<=25; //extend the mantissa to 48 bits for the division |NOT SUPPORTED BY BLACKFIN|
    m1 = f1.float_parts.mantissa;//>>12;

    m1<<=9; //extend the mantissa to 32 bits
    m = m1/((f2.float_parts.mantissa)>>7); //lower the divisor's mantissa to half the bits of the dividend's
                                    //was >>7

    //mm = mm1/f2.float_parts.mantissa; |NOT SUPPORTED BY BLACKFIN|


   // m<<=6;//shift it so that the msb matches the mantissa's size
   // mm>>=3;//shift it so that the msb fits into mantissa's size |NOT SUPPORTED BY BLACKFIN|


    result.float_parts.mantissa = m;

    i = 0;
    while(!(result.float_parts.mantissa & (0x01<<(22))))//if(manBitSet(result,0))
    {
        i++;
        result.float_parts.mantissa <<=1;
        if(i>6)
           result.float_parts.exponent -=1; //In some cases, it needs to reduce the exponent by 1
                                            //specifically the way I do the division now, if the 7th bit
    }                                       //is not 1 (the hidden bit)
    result.float_parts.mantissa <<=1;



    if(f1.float_parts.sign & 0x01 || f2.float_parts.sign &0x01)
        result.float_parts.sign = 1;






    return result;

}

_float mulFloat(_float f1,_float f2)
{
    unsigned long m,m1,m2;
   // unsigned long long gig,mm1,mm2; |NOT SUPPORTED BY BLACKFIN|
    char texp1,texp2;

    _float result;

    //getting the hidden bit out for both of them
    f1.float_parts.mantissa = ((f1.float_parts.mantissa)>>1) | (0x01<<22);
    f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);

    m1 = f1.float_parts.mantissa>>12;//shitinf them to the right by 12 since blackfin won't take
    m2 = f2.float_parts.mantissa>>12;//calculations of 64bit numbers
  //  mm1 = f1.float_parts.mantissa;
  //  mm2 = f2.float_parts.mantissa; |NOT SUPPORTED BY BLACKFIN|

    m = (m1 * m2)<<1;
   // gig = (mm1*mm2);|NOT SUPPORTED BY BLACKFIN|


    result.float_parts.mantissa = m;



    texp1 = f1.float_parts.exponent-127;
    texp2 = f2.float_parts.exponent-127;


    result.float_parts.exponent = texp1+texp2+127;



    if(f1.float_parts.sign & 0x01 || f2.float_parts.sign &0x01)
        result.float_parts.sign = 1;




    //if first bit is set
    if(result.float_parts.mantissa & (0x01<<22)) //if(manBitSet(result,0))
    {
    result.float_parts.mantissa <<=1;
    result.float_parts.exponent +=1;
    }
    else
    {
        //hide the hidden bit
        while(1)
        {
            result.float_parts.mantissa <<=1;
            if(result.float_parts.mantissa & (0x01<<22))//if(manBitSet(result,0))
            {
                result.float_parts.mantissa <<=1;
                break;
            }
        }
    }

    return result;

}

_float subFloat(_float f1,_float f2)
{
    unsigned char diff;
    unsigned long m;
    _float result;

    if(f1.float_parts.sign == 1 &&   //negative f1 subtracting positive f2 ...is adding two same-sign numbers
       f2.float_parts.sign == 0)
       {
           f2.float_parts.sign = 1; //negate f2
           //add them
           return addFloat(f1,f2);
       }

    if(f1.float_parts.sign == 0 && //positive f1, subtracting negative f2, is same sign addition
       f2.float_parts.sign == 1)
       {
            f2.float_parts.sign = 0;
            return addFloat(f1,f2);
       }
    //first see whose exponent is greater
    if(f1.float_parts.exponent > f2.float_parts.exponent)
    {
        diff = f1.float_parts.exponent - f2.float_parts.exponent;

          if(diff > 22)
            return f1;

        //now shift f2's mantissa by the difference of their exponent to the right
        //adding the hidden bit
        f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);
        f2.float_parts.mantissa >>= (int)(diff);//was (diff-1)

        //also increase its exponent by the difference shifted
        f2.float_parts.exponent = f2.float_parts.exponent + diff;

    }
    else if(f1.float_parts.exponent < f2.float_parts.exponent)
    {
        diff = f2.float_parts.exponent - f1.float_parts.exponent;

         if(diff > 22)
            return f2;
        result = f1;
        f1 = f2;        //swap them
        f2 = result;

        //now shift f2's mantissa by the difference of their exponent to the right
        //adding the hidden bit
        f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);
        f2.float_parts.mantissa >>= (int)(diff);

        //also increase its exponent by the difference shifted
        f2.float_parts.exponent = f2.float_parts.exponent + diff;


         result.float_parts.sign = f1.float_parts.sign^0x01;//flip the sign
    }
    else//if the exponents were equal
      f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22); //bring out the hidden bit


     //this brings out the hidden bit of the f1 mantissa too  //ELEOS MALAKA AYTO EIXA KSEXASEI TOSI WRA
     f1.float_parts.mantissa = ((f1.float_parts.mantissa)>>1) | (0x01<<22);



    result.float_parts.exponent = f1.float_parts.exponent;



    m = f1.float_parts.mantissa - f2.float_parts.mantissa ;//subtracting mantissas
    result.float_parts.mantissa = m;


    int index = 0;
    int i;
    for(i = 0; i < 7; i++)
    {
        if(result.float_parts.mantissa & (0x01<<(22-i)))//if(!manBitSet(result,i))
            index++;
        else
        {
            index++;
            break;
        }
    }

    if(index >0)
    {
        if(index >1)
            result.float_parts.exponent -=1;
      result.float_parts.mantissa <<=index;
    }


    return result;
}


/*Bug spotted: Mantissa, hence the result itself gets wrong for VERY small numbers, like 0.000001*/
//Well being an idiot helps with these problems, for such a small number you shift the mantissa
//by more than 30 bits, think about it a little bit, shifting a 24 bit word, for 32 bit gives you ...
//hmm.... hmm ..... yeah there's your problem!
_float addFloat(_float f1,_float f2)
{
    unsigned char diff;
    _float result;

    //addition of different signed numbers is subtraction
    if( f1.float_parts.sign == 1 && f2.float_parts.sign == 0)
    {
        f1.float_parts.sign = 0;
        result = subFloat(f1,f2);
        result.float_parts.sign ^= 1;
        return result;
    }
    if(f1.float_parts.sign == 0 && f2.float_parts.sign ==1)
    {
        f2.float_parts.sign = 0;
        return subFloat(f1,f2);
    }


    //first see whose exponent is greater
    if(f1.float_parts.exponent > f2.float_parts.exponent)
    {
        diff = f1.float_parts.exponent - f2.float_parts.exponent;

        if(diff > 22)
            return f1; //return the first number unaltered
        //now shift f2's mantissa by the difference of their exponent to the right
        //adding the hidden bit
        f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);
        f2.float_parts.mantissa >>= (int)(diff);//was (diff-1)

        //also increase its exponent by the difference shifted
        f2.float_parts.exponent = f2.float_parts.exponent + diff;

    }
    else if(f1.float_parts.exponent < f2.float_parts.exponent)
    {
        diff = f2.float_parts.exponent - f1.float_parts.exponent;

        if(diff > 22)
            return f2; //return the second number unaltered

        result = f1;
        f1 = f2;        //swap them
        f2 = result;

       // floatInBinary(f2);

        //now shift f2's mantissa by the difference of their exponent to the right
        //adding the hidden bit
        f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);
        f2.float_parts.mantissa >>= (int)(diff); //was (diff-1)

        //also increase its exponent by the difference shifted
        f2.float_parts.exponent = f2.float_parts.exponent + diff;

    }
    else//if the exponents were equal
      f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22); //bring out the hidden bit




        //this brings out the hidden bit of the f1 mantissa too
        f1.float_parts.mantissa = ((f1.float_parts.mantissa)>>1) | (0x01<<22);

       // printf("\nAfter shifting...\n");
        //floatInBinary(f1.all); //is as it should be ... hm what the heck is wrong then?





        result.float_parts.sign = f1.float_parts.sign;  //they are the same anyway :)
        result.float_parts.exponent = f1.float_parts.exponent;
        result.float_parts.mantissa = f1.float_parts.mantissa +f2.float_parts.mantissa;

        if(result.float_parts.mantissa & (0x01<<22))
        {
             result.float_parts.mantissa <<= 1;  //hide the hidden bit

        }
        else
           result.float_parts.exponent +=1;




        return result;

}

int isOne(unsigned char b,int bit)
{
    if(b & (0x01<<(7-bit)))
        return 1;
    else
        return 0;
}

void printBinary(unsigned char b,int startbit,int endbit)
{
    int i;
    for(i = startbit; i <= endbit; i++)
    {
        if(isOne(b,i))
            uart0SendChar('1');
        else
            uart0SendChar('0');
    }


}



int     ltFloat(_float f1,_float f2)
{
    return gtFloat(f2,f1);
}

int     gtFloat(_float f1,_float f2)
{
    if(f1.float_parts.sign == 1 && f2.float_parts.sign == 0)
        return 0;
    else if(f1.float_parts.sign == 0 && f2.float_parts.sign ==1)
        return 1;

    //if same signed and negative
    if(f1.float_parts.sign == 1)
    {
        if(f1.float_parts.exponent > f2.float_parts.exponent)
            return 0;
        else if(f1.float_parts.exponent < f2.float_parts.exponent)
            return 1;

        //else if they have same exponents compare mantissas
        if(f1.float_parts.mantissa > f2.float_parts.mantissa )
            return 0;
        else
            return 1;
    }
    else//same signed and positive, exactly the opposite
    {
        if(f1.float_parts.exponent > f2.float_parts.exponent)
            return 1;
        else if(f1.float_parts.exponent < f2.float_parts.exponent)
            return 0;

        //else if they have same exponents compare mantissas
        if(f1.float_parts.mantissa > f2.float_parts.mantissa )
            return 1;
        else
            return 0;
    }

    //don't think it's possible to get here ... but if we do
    return 0;

}


void printMantissa(_float f)
{
    unsigned long m = f.float_parts.mantissa;
    unsigned char* bytes = (unsigned char*)&m;

    uart0SendChar('\n');
    printBinary(bytes[2],1,7);
    printBinary(bytes[1],0,7);
    printBinary(bytes[0],0,7);
   uart0SendChar('\n');
}


void printExponent(_float f)
{
    unsigned char e =(unsigned char) f.float_parts.exponent;
    float a = f.all;
    uart0SendString("The exponent is: \n");
    printBinary(e,0,7);
}

void floatInBinary(float a)
{
    unsigned char* bytes = (unsigned char*)&a;
    

    uart0SendString("Float in Binary is: \n");
    printBinary(bytes[3],0,0);//sign
    uart0SendChar(' ');
    printBinary(bytes[3],1,7);//exponent
    printBinary(bytes[2],0,0);
    uart0SendChar(' ');
    printBinary(bytes[2],1,7);
    printBinary(bytes[1],0,7);//mantissa
    printBinary(bytes[0],0,7);
}


void printLong(unsigned long a)
{
     unsigned char* bytes = (unsigned char*)&a;

    uart0SendString("\nLong is: \n");
    printBinary(bytes[3],0,7);
    printBinary(bytes[2],0,7);
    printBinary(bytes[1],0,7);
    printBinary(bytes[0],0,7);
}

void printLLong(unsigned long long a)
{
    unsigned char* bytes = (unsigned char*)&a;

    uart0SendString("\nBig Long is: \n");
    printBinary(bytes[7],0,7);
    printBinary(bytes[6],0,7);
    printBinary(bytes[5],0,7);
    printBinary(bytes[4],0,7);
    printBinary(bytes[3],0,7);
    printBinary(bytes[2],0,7);
    printBinary(bytes[1],0,7);
    printBinary(bytes[0],0,7);
}

void printFloat(_float f,int ddigits)
{
    int i,j;
    unsigned long bradix,aradix;
    char e;

    e = f.float_parts.exponent;
    e -=127; //get true exponent

    _float sum;
    parseFloat(0.0,&sum);


    if(e > 0)
    {
        bradix = (f.float_parts.mantissa >>(23-(int)e)) | (0x01<<(int)e);


        _float one = intToFloat(1);
        _float eleos,division;

        //now going for the real thing,let's get what the bits after the radix sum up to
        for(i = e,j=1; i < 23; i ++,j++)
        {
            if(f.float_parts.mantissa & (0x01<<(22-i)) )//if(manBitSet(f,i))
            {
                //sum += bigptwo/power(2,j);
                eleos = intToFloat(power(2,j));
                division = divFloat(one,eleos);
                sum = addFloat(sum,division);
            }
        }
    }
    else if( e < 0)
    {
        bradix = 0;
        f.float_parts.mantissa = (f.float_parts.mantissa>>1) | (0x01 << 22);
        f.float_parts.mantissa >>= (int)(e*(-1));


        _float one = intToFloat(1);
        _float eleos,division;
        //now going for the real thing,let's get what the bits after the radix sum up to
        for(i = 0,j=0; i < 23; i ++,j++)
        {
            if(f.float_parts.mantissa & (0x01<<(22-i)) )//if(manBitSet(f,i))
            {
                 //sum += bigptwo/power(2,j);
                eleos = intToFloat(power(2,j));
                division = divFloat(one,eleos);
                sum = addFloat(sum,division);
            }
        }
    }

    int decimaldigits = power(10,ddigits);
    sum = mulFloat(sum,intToFloat(decimaldigits));
    e = sum.float_parts.exponent;
    e -=127; //get true exponent
    aradix = (sum.float_parts.mantissa >>(23-(int)e)) | (0x01<<(int)e);


   /*//printf printing 
   if(f.float_parts.sign == 0)
        printf("\n%d.%d",bradix,aradix);
    else
        printf("\n-%d.%d\n",bradix,aradix);
*/

    //uart printing ...
    if(f.float_parts.sign == 1)
        uart0SendChar('-');

    printNumber(10, 12, 0 , ' ', (int)bradix);
    uart0SendChar('.'); printNumber(10, 12, 0 , ' ', (int)aradix);

}


_float intToFloat(int number)
{
    int i;
    _float result;
    unsigned char* bytes= (unsigned char*)&number;

    if(number > 0)
        result.float_parts.sign = 0;
    else if(number < 0)
        result.float_parts.sign = 1;
    else
    {
        parseFloat(0.0,&result);
        return result;
    }



    //we have to get the most significant bit of the int
    for(i = 31; i >=0; i --)
    {
        if(bytes[i/8] & (0x01 << (i-((i/8)*8))))
            break;
        /*if(i >= 24)
        {
            if(bytes[3] & (0x01<<(i-24)))
                break;
        }
        else if( i <24 && i >= 16)
        {
            if(bytes[2] & (0x01<<(i-16)))
                break;
        }
        else if(i<16 && i >=8)
        {
            if(bytes[1] & (0x01<<(i-8)))
                break;
        }
        else
        {
            if(bytes[0] & (0x01<<i))
                break;
        }*/
    }

    result.float_parts.exponent = i+127;



    result.float_parts.mantissa = (bytes[2] <<  16 | bytes[1] << 8 | bytes[0]);
    while(!(result.float_parts.mantissa & (0x01<<22)))
    {
        result.float_parts.mantissa <<=1;
    }
    result.float_parts.mantissa <<=1;



    return result;
}

