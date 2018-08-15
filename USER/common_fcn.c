#include "common_fcn.h"
#include "includes.h"
#include "string.h"
#include "assert.h"
//
//
//

// global varaibles declaration

RB_State State={0,0,0,0,0,0,0,0};
RB_State Exp_State={0,0,0,0,0,0,0,0};

long steps1=0,steps2=0,steps3=0,steps4=0,steps_X=0,steps_Y=0;

//common functions definition

float my_abs(float num){
	if(num<0)
			num=-num;
	return num;
}

float deg2rad(float deg){
	return deg*3.14159f/180;
}

u8 strcmp_real(char str1[3],char str2[3]){
	u16 sum1=0,sum2=0;
	
	sum1 = str1[0]+str1[1]+str1[2];
	sum2 = str2[0]+str2[1]+str2[2];
	if(sum1==sum2)
		return 0;
	else 
		return 1;
}

float my_atof(char str[4]){
	float temp;
	temp = ((float)str[3]-48)/10+((float)str[2]-48)+(((float)str[1]-48)*10);
	if(str[0]=='-')
		temp=-temp;
	return temp;
}

int is_digit(char ch)  
{  
    if(ch>='0'&&ch<='9')  
        return 1;  
    else  
        return 0;  
}  
int is_space(char ch)  
{  
    if(ch==' ')  
        return 1;  
    else  
        return 0;  
}

double gps_atof(char *s)  
{  
    double power,value;  
    int i,sign;  
    //assert(s!=NULL);
    for(i=0;is_space(s[i]);i++);//?????????  
    sign=(s[i]=='-')?-1:1;  
    if(s[i]=='-'||s[i]=='+')//???????????  
        i++;  
    for(value=0.0;is_digit(s[i]);i++)//?????????  
        value=value*10.0+(s[i]-'0');  
    if(s[i]=='.')  
        i++;  
    for(power=1.0;is_digit(s[i]);i++)//?????????  
    {  
        value=value*10.0+(s[i]-'0');  
        power*=10.0;  
    }  
    return sign*value/power;  
}

