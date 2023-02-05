#include<stdio.h>
#include<stdlib.h>
#include <string.h>


const int MAX_DISTANCE = 500;
const int MIN_DISTANCE = 0;


int number_input = 0;
int num3 = 0;
float resault = 0, presum_num = 0;


void Recursive_Expression_Filter(int input_num){
	number_input++;
	resault = (float)((presum_num + input_num) / number_input);
	presum_num += input_num;
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
			printf("-  평균 값 : %0.2f\n-  데이터 갯수 : %d\n\n", resault,number_input);
			
		}
	}
	line();
	printf("-  최종 평균 값 : %0.2f\n", resault);

	return 0;
}