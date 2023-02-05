#include<stdio.h>
#include<stdlib.h>
#include <string.h>

#define CHANGE_TEMP_DATA  10

const int MAX_DISTANCE = 500;
const int MIN_DISTANCE = 0;

//이걸 구조체로 만들어보면 좋을듯.
float Temp_data[CHANGE_TEMP_DATA]={0};
int Count_input_num = 0;
float resault = 0;
float pre_sum=0;

void Recursive_Expression_Filter(int input_num){
	
	Count_input_num++;
	
	//범위보다 클때 마지막값 삭제후 최신값 공간생성.
	if(Count_input_num>CHANGE_TEMP_DATA){
		pre_sum-=Temp_data[(Count_input_num-1)%(CHANGE_TEMP_DATA)];
		for(int j=1;j<CHANGE_TEMP_DATA;j++){
			Temp_data[j-1]=Temp_data[j];
		}
		Temp_data[CHANGE_TEMP_DATA-1]=input_num;
	}
	//범위 배열에 데이터 입력.
	else {Temp_data[(Count_input_num-1)%(CHANGE_TEMP_DATA)]=input_num;}
	//범위 배열의 합.이거말고 1번째 값 기억시키고 다음 계산때 그거빼고 다시 두번째거 
	/*for(int i=0;i<CHANGE_TEMP_DATA;i++){
		temp_sum+=Temp_data[i];
	}*/
	

	int k=0;
	if(Count_input_num>CHANGE_TEMP_DATA){k=CHANGE_TEMP_DATA;}else {k=Count_input_num;}
	resault = (float)((pre_sum+input_num)/k);
	pre_sum+=input_num;
	printf("pre_sum : %0.2f\n",pre_sum);
}
void line() {
	puts("---------------------------------------------------");
}

int main() {
	int temp=0;
	int count_num = 0;
	
	line();
	puts("-------------------재귀평균 필터-------------------");
	printf("\n\n-  종료를 원할시 out or OUT 입력.\n\n");
	while (1) {
		char check_num[20];
		printf("-  입력 : ");
		scanf("%s",check_num);
		temp = atoi(check_num);
		if (strcmp(check_num, "out") == 0 || strcmp(check_num, "OUT") == 0) { break; }
		else if (temp > MAX_DISTANCE || temp < MIN_DISTANCE) { printf("-  data error \n"); }
		else {
			Recursive_Expression_Filter(temp);
			printf("-  평균 값 : %0.2f\n-  데이터 갯수 : %d\n\n", resault,Count_input_num);
			for(int i=0;i<CHANGE_TEMP_DATA;i++){
				printf("-  %2d %0.2f\n",i+1,Temp_data[i]);
			}
			printf("\n");
		}
	}
	line();
	printf("-  최종 평균 값 : %0.2f\n", resault);

	return 0;
}