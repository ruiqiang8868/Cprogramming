// gcc -o abstract_data_triplet -g abstract_data_triplet.c

#include <stdio.h>
#include <stdlib.h>

#define TRUE        1
#define FALSE       0
#define OK          1
#define ERROR       0
#define INFEASIBLE -1
#define OVERFLOW   -2

typedef int Status;
typedef int *Triplet;


int InitTriplet(Triplet &T, int v1, int v2, int v3)
{
	//
	T = (int *)malloc(3 * sizeof(int));
	if (!T) exit(OVERFLOW);
	T[0] = v1;
	T[1] = v2;
	T[2] = v3;

	return OK;
}//InitTriplet


int DestoryTriplet(Triplet & T)
{
	//
	free(T);
	T = NULL;
	return OK;

}// DestoryTriple

int Get(Triplet T, int i, int & e)
{
	//
	if (i < 1 || i > 3) return ERROR;
	e = T[i -1];
	return OK;
}//Get

int Put(Triplet &T, int i, int e)
{
	//
	if (i < 1 || i > 3) return ERROR;
	T[i - 1] = e;
	return OK;
}//Put

int IsAscending(Triplet T)
{
	//
	return (T[0] <= T[1]) && (T[1] <= T[2]);
}// IsAscending

int IsDescending(Triplet T)
{
	//
	return (T[0] >= T[1]) && (T[1] >= T[2]);
}// IsDescending

int Max(Triplet T, int & e)
{
	//
	e = (T[0] >= T[1])? ((T[0] >= T[2])? T[0]:T[2]):((T[1] >= T[2])? T[1]:T[2]);
	return OK;
}//Max

int Min(Triplet T, int & e)
{
	//
	e = (T[0] <= T[1])? ((T[0] <= T[2])? T[0]:T[2]):((T[1] <= T[2])? T[1]:T[2]);
	return OK;
}// Min

int main(void)
{
	Triplet T;
	int element1, element2, element3;
	int max, min;
	char change_command;
	int num, loc;
	
	printf("this is a test program...\r\n");
	printf("Pleae input three int number to run this program:\r\n");
	
	scanf("%d %d %d", &element1, &element2, &element3);
	
	if (InitTriplet(T, element1, element2, element3))
	{
		printf("Initilize Triplet successful!\r\n");
		printf("Triplet elements are %d, %d, %d\r\n", T[0], T[1], T[2]);
	}
	else
	{
		printf("Initialize Triplet failed!\r\n");
	}
	

	Max(T, max);
	Min(T, min);
	printf("Maximum number is %d.\r\n", max);
	printf("Minimum number is %d.\r\n", min);
	
	while(TRUE)
	{
		printf("If you want to change the number or not? [Y/N]\r\n");
		
		scanf("%s", &change_command);
		if (change_command == 'Y' || change_command == 'y')
		{
			printf("Please input the number and location to replace\r\n");
			scanf("%d %d",&num, &loc);
			if (Put(T,loc, num))
			{
				printf("New Triplet elements are %d, %d, %d\r\n", T[0], T[1], T[2]);
			}
			else
			{
				printf("location out of range!\r\n");
			}
		}
		else
		{
			break;
		}
		
		printf("If you want to get the number or not? [Y/N]\r\n");
		
		scanf("%s", &change_command);
		if (change_command == 'Y' || change_command == 'y')
		{
			printf("Please input the location of number to get\r\n");
			scanf("%d",  &loc);
			if (Get(T, loc, num))
			{
				printf("Get elements is %d, location is %d\r\n", num, loc);
			}
			else
			{
				printf("Location out of range!\r\n");
			}
		}
		else
		{
			break;
		}
	}
	
	if (DestoryTriplet(T))
	{
		printf("Destory triplet successful!\r\n");
	}
	
	return OK;
}
