// gcc -o sort_algorithm -g sort_algorithm.cpp
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define TRUE  1
#define FALSE 0
#define num_count 10

int a[num_count];
int b[6] = {34, 9, 3, 45 ,23, 27};

void display_data(int a[], int n);
void bubble_sort(int a[], int n); // 冒泡排序 时间复杂度 T(n) = O(n^2)
void generate_random(int *a, int count, int range, int min);

int main(void)
{
	int i;
	
	printf("This program is some examples about data sort algorithm.\r\n");
	
	// Generate random number for sort algorithm
	generate_random(a, num_count, 100, 1);
	display_data(a, num_count);
		
	bubble_sort(a, num_count);
	printf("Sorted numnber is: \r\n");
	display_data(a, num_count);


	return 0;
}

void display_data(int a[], int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		printf("%d\t", a[i]);
	}
	printf("\r\n");
}

void generate_random(int *a, int count, int range, int min)
{
	srand(time(NULL));
	
	for (int i = 0; i < count; ++i)
	{
		a[i] = rand() % range + min;
	}
	
}


//将a中整数序列重新排列成从小到大的有序整数序列
void bubble_sort(int *a, int n)
{
	int i, j, tmp, change;
	
	for ((i = n - 1, change = TRUE); ( i >= 1 && change); --i)
	{
		change = FALSE;
		for (j = 0; j < i; ++j)
		{
			if (a[j] > a[j + 1])
			{
				tmp = a[j];
				a[j] = a[j + 1];
				a[j + 1] = tmp;
				change = TRUE;
			}
		}
	}


}
