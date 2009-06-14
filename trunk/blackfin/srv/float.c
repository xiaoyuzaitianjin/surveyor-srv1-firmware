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

#include "float.h"
#include "print.h"

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


    //the new sign is the xor of the sign of the two numbers
    result.float_parts.sign = f1.float_parts.sign ^ f2.float_parts.sign;





    return result;

}

_float mulFloat(_float f1,_float f2)
{
    unsigned long m;
    unsigned long long gig,mm1,mm2;
    char texp1,texp2;
    _float result;

    //multiplication by zero needs to be handled like this
    if( (f1.float_parts.mantissa == 0 && f1.float_parts.exponent == 0) ||
        (f2.float_parts.mantissa == 0 && f2.float_parts.exponent == 0))
        {
            parseFloat(0.0,&result);
            return result;
        }


    texp1 = f1.float_parts.exponent-127;
    texp2 = f2.float_parts.exponent-127;


    result.float_parts.exponent = texp1+texp2+127;



    mm1 =  f1.float_parts.mantissa|(0x01<<(23));
    mm2 =  f2.float_parts.mantissa|(0x01<<(23));

    //gig = mm1*mm2; //blackfin will not accept this so ...
    bigMulti(mm1,mm2,&gig);




int i =0;
//while(!( gig &  (0x8000000000000000)))
while(!( (gig>>32) &  (0x80000000)))
{
    i++;
    gig <<=1;
}
gig <<=1;//hide the hidden bit

if( i == 16)
    result.float_parts.exponent +=1;

m = gig >>41;

result.float_parts.mantissa = m;

//result.float_parts.mantissa +=1; //testing

//lastly adding the new sign
result.float_parts.sign = f1.float_parts.sign ^ f2.float_parts.sign;



    return result;

}

_float subFloat(_float f1, _float f2)
{
	
    unsigned char diff;
    unsigned long m,m1,m2;
    _float result;
	result.float_parts.sign = 0; //default sign is positive, and should be set here

  //if one of the numbers is zero
    if( (f1.float_parts.mantissa == 0 && f1.float_parts.exponent == 0 && f1.float_parts.sign == 0) ||
	  (f2.float_parts.mantissa == 0 && f2.float_parts.exponent == 0 && f2.float_parts.sign == 0))
	  {
		if(f1.float_parts.mantissa == 0 && f1.float_parts.exponent == 0 && f1.float_parts.sign == 0)
		{
		    //flip the other number's sign
		    //if the other number is not zero
		    if(!(f2.float_parts.mantissa == 0 && f2.float_parts.exponent == 0 && f2.float_parts.sign == 0))
		    {
			  f2.float_parts.sign ^= 0x01;
			  return f2; //and return the other number
		    }
		    else
			  return f2; //zero minus zero is zero
		}
		else if(f2.float_parts.mantissa == 0 && f2.float_parts.exponent == 0 && f2.float_parts.sign == 0)
		    return f1;
	  }


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
	  //f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);
	 // f2.float_parts.mantissa >>= (int)(diff);//was (diff-1)

	  m1 = f1.float_parts.mantissa | (0x01<<(23));
	  m2 = f2.float_parts.mantissa | (0x01<<(23));

	  if(diff > 8)
	  {
		m1<<=8;
		m2>>=(int)(diff-8);

	  }
	  else
		m1 <<=(int)diff;

	  m = m1-m2;

	 if(diff <=8)
		m>>= (int)diff;
	  else
		m>>= 8;

	  if(f1.float_parts.sign == 1)
		result.float_parts.sign = 1;
	  else
		result.float_parts.sign = 0;

	  //also increase its exponent by the difference shifted
	  f2.float_parts.exponent = f2.float_parts.exponent + diff;

    }
    else if(f1.float_parts.exponent < f2.float_parts.exponent)
    {
	  diff = f2.float_parts.exponent - f1.float_parts.exponent;

	    if(diff > 22)//if the difference is huge return the number unaltered
	   {
		//and since it is subtraction flip its sign
		f2.float_parts.sign ^= 0x01;
		return f2;
	   }

	  result = f1;
	  f1 = f2;	  //swap them
	  f2 = result;


	  //now shift f2's mantissa by the difference of their exponent to the right
	  //adding the hidden bit
	 // f2.float_parts.mantissa = ((f2.float_parts.mantissa)>>1) | (0x01<<22);
	 // f2.float_parts.mantissa >>= (int)(diff);
	  m1 = f1.float_parts.mantissa | (0x01<<(23));
	  m2 = f2.float_parts.mantissa | (0x01<<(23));

	  if(diff > 8)
	  {
		m1<<=8;
		m2>>=(int)(diff-8);

	  }
	  else
		m1 <<=(int)diff;

	  m = m1-m2;

	 if(diff <=8)
		m>>= (int)diff;
	  else
		m>>= 8;

	  //also increase its exponent by the difference shifted
	  f2.float_parts.exponent = f2.float_parts.exponent + diff;


	   result.float_parts.sign = f1.float_parts.sign^0x01;//flip the sign
    }
    else//if the exponents were equal
    {
	if(f2.float_parts.mantissa > f1.float_parts.mantissa)
	{
	   result = f1;
	  f1 = f2;	  //swap them
	  f2 = result;

	   result.float_parts.sign = f1.float_parts.sign^0x01;//flip the sign
	}

	  m1 = f1.float_parts.mantissa | (0x01<<(23));
	  m2 = f2.float_parts.mantissa | (0x01<<23);
	  m = m1-m2;


    }

    result.float_parts.exponent = f1.float_parts.exponent;





  //if we got an all zero mantissa
  if(m == 0)
  {//SPECIAL CASE .... ZERO
	  result.float_parts.mantissa = 0;
	  result.float_parts.sign =0;
	  result.float_parts.exponent = 0;
	  return result;
  }



int i = 0;
   while(!(m & (0x01<<31)) && (i < 32)) //i > 31 means mantissa is completely zero
    {
	  i++;

	  m <<=1;
    }
    m>>=8;//fit back into the mantissa
    if(i > 8)
	  result.float_parts.exponent -=(i-8);
   
   result.float_parts.mantissa = m;


    return result;
} 


_float addFloat(_float f1,_float f2)
{
    //Reason why I added big longs in here is because there was a case
    //where addition was so long that the most significant 1 bit went out of long scope
    //hence literally killing the result
    unsigned long long f1_mantissa ;
    unsigned long long f2_mantissa;
    unsigned long long bigm;
    unsigned char diff;
    _float result;

    unsigned long m;


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

        f1_mantissa = f1.float_parts.mantissa;
        f2_mantissa = f2.float_parts.mantissa;

        f1_mantissa = f1_mantissa | (0x01<<(23));
        f2_mantissa = f2_mantissa | (0x01<<(23));



         if(diff > 8)
        {
            f1_mantissa<<=8;
            f2_mantissa>>=(int)(diff-8);

        }
        else
            f1_mantissa <<=(int)diff;



       if(diff > 8)
            bigm = f1_mantissa + f2_mantissa+(1<<(diff-8)); //maybe this aint's right?
        else
            bigm = f1_mantissa + f2_mantissa+(1<<(diff-1));


       if(diff <=8)
            bigm>>= (int)diff;
        else
            bigm>>= 8;

    }
    else if(f1.float_parts.exponent < f2.float_parts.exponent)
    {
        diff = f2.float_parts.exponent - f1.float_parts.exponent;

         if(diff > 22)
            return f2; //return the second number unaltered

        result = f1;
        f1 = f2;        //swap them
        f2 = result;


        f1_mantissa = f1.float_parts.mantissa;
        f2_mantissa = f2.float_parts.mantissa;


        f2_mantissa = f2_mantissa | (0x01<<(23));
        f1_mantissa = f1_mantissa | (0x01<<(23));


         if(diff > 8)
        {
            f1_mantissa<<=8;
            f2_mantissa>>=(int)(diff-8);

        }
        else
            f1_mantissa <<=(int)diff;


       if(diff > 8)
            bigm = f1_mantissa + f2_mantissa+(1<<(diff-8));//maybe this aint's right?
        else
            bigm = f1_mantissa + f2_mantissa+(1<<(diff-1));



       if(diff <=8)
            bigm>>= (int)diff;
        else
            bigm>>= 8;
    }
    else //if the exponents were equal
    {

        f1_mantissa = f1.float_parts.mantissa;
        f2_mantissa = f2.float_parts.mantissa;

        f2_mantissa = f2_mantissa | (0x01<<(23));
        f1_mantissa = f1_mantissa | (0x01<<(23));
        bigm = f1_mantissa + f2_mantissa;

    }


    result.float_parts.sign = f1.float_parts.sign;  //they are the same anyway :)
    result.float_parts.exponent = f1.float_parts.exponent;

    m = bigm;
    int i =0;
    while(!(m & 0x01<<31) && (i < 32))
   // while(!( (m>>24) & (0x8000000000) ) && (i < 64))
    {
        m<<=1;
        i++;
    }
    if(i <8)
        result.float_parts.exponent +=1;

    m>>=8; //shift to fit into the mantissa
    result.float_parts.mantissa = m;



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
            putchar('1');
        else
            putchar('0');
    }


}



int     lteFloat(_float f1,_float f2)
{
    return gteFloat(f2,f1);
}

int     gteFloat(_float f1,_float f2)
{
    if(f1.float_parts.sign == 1 && f2.float_parts.sign == 0)
        return 0;
    else if(f1.float_parts.sign == 0 && f2.float_parts.sign ==1)
        return 1;
        
     //check for equality
    if( (f1.float_parts.sign == f2.float_parts.sign) &&
        (f1.float_parts.mantissa == f2.float_parts.mantissa) &&
        (f1.float_parts.exponent == f2.float_parts.exponent) )
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

    putchar('\n');
    printBinary(bytes[2],1,7);
    printBinary(bytes[1],0,7);
    printBinary(bytes[0],0,7);
   putchar('\n');
}


void printExponent(_float f)
{
    unsigned char e =(unsigned char) f.float_parts.exponent;
    printf("The exponent is: \n");
    printBinary(e,0,7);
}

void floatInBinary(float a)
{
    unsigned char* bytes = (unsigned char*)&a;
    

    printf("Float in Binary is: \n");
    printBinary(bytes[3],0,0);//sign
    putchar(' ');
    printBinary(bytes[3],1,7);//exponent
    printBinary(bytes[2],0,0);
    putchar(' ');
    printBinary(bytes[2],1,7);
    printBinary(bytes[1],0,7);//mantissa
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


    if(e >= 0)
    {
	  bradix = (f.float_parts.mantissa >>(23-(int)e)) | (0x01<<(int)e);

	  _float one = intToFloat(1);
	  _float eleos,division;

	  //now going for the real thing,let's get what the bits after the radix sum up to
	  for(i = e,j=1; i < 23; i ++,j++)
	  {

		if(f.float_parts.mantissa & (0x01<<(22-i)) )//was 22-i
		{
		    //sum += bigptwo/power(2,j);
		    if(i >=22)
			  e = 1;
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


    if(f.float_parts.sign == 0)
	  printf("\n%d.",bradix);
    else
	  printf("\n-%d.",bradix);



    _float temp;
    for(i = 0; i < ddigits; i ++)
    {
	  //then it means we go
	  //beyond the precision
	  //this library can achieve
	  if( i >= 6)
		break;


	   temp = mulFInt(sum,power(10,i+1));
	   //e = sum.float_parts.exponent;
	   e = temp.float_parts.exponent;
	   e -=127; //get true exponent
	   if(e >= 0)
	   {
	     aradix = (temp.float_parts.mantissa >>(23-(int)e)) | (0x01<<(int)e);

	   }
	   else
	   {//that is to print the leading zeros after the radix if there are any ofcourse
		aradix = 0;
		printf("%d",aradix);
	   }

    }

	  //and finally print the rest of the number after the radix
	  printf("%d",aradix);

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
    }

    result.float_parts.exponent = i+127;



    result.float_parts.mantissa = (bytes[2] <<  16 | bytes[1] << 8 | bytes[0]);

    i= 0;
    while(!(result.float_parts.mantissa & (0x01<<22)) && i<23) //the i is to make sure that
    {                                                          //for all zero mantissas we don't
        result.float_parts.mantissa <<=1;                      //get infinite loop
        i++;
    }
    result.float_parts.mantissa <<=1;



    return result;
}


void bigMulti(unsigned long long mm1,unsigned long long mm2,unsigned long long* res)
{
    int i;
    *res = 0;

    for(i = 0; i < 24; i++) //24 is since we have 24 bit mantissa (+the hidden bit)
    {

        if(mm2 & 0x01)
            *res += (mm1<<(i));//*res += (mm1<<i);

        mm2 >>=1;
    }
}

//I put this function here so that it won't have any external dependencies, it is just a power(x raised to y)
// for integers function, which I usually have in my math.h . IF you have anything better just link to it
int power( int x,int p)
{
     int res = 1;
    
    if(p !=0)
    {
        while(p != 0)
        {
            res *= x;
            p--;
        }

        return res;
    }
    else
        return 1;
}
