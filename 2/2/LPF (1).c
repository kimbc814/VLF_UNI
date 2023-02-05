#include<stdio.h>

#define ALPHA 0.7

float pre_avg = 0, new_avg =0;

void Lpf(int lpf_data){
	if(pre_avg == 0){pre_avg = lpf_data;}
	new_avg=(pre_avg*ALPHA)+((1-ALPHA)*lpf_data);
	pre_avg = new_avg;
}

int main(){
	float a=0;
	
	while(1){
		printf("값을 넣으세요 : ");
		scanf("%f",&a);
		Lpf(a);
		printf("%f \n",new_avg);
	}
}