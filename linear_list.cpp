// linux环境下，当前程序的编译命令
/* 
	gcc -o linear_list -g linear_list.cpp
*/

#include <stdio.h>
#include <stdlib.h>

#define TRUE        1
#define FALSE       0
#define OK          1
#define ERROR       0
#define INFEASIBLE -1
#define OVERFLOW   -2

#define LIST_INIT_SIZE 100 // 线性列表存储空间的初始分配量
#define LISTINCREMENT  10  // 线性表存储空间的分配增量

#define ElemType char
#define Status   int

typedef struct
{
	ElemType * elem; // 存储空间基地址
	int length;      // 当前长度
	int listsize;    // 当前分配的存储荣来嗯
}SqList;

int main(void)
{
	return 0;
}

Status InitList_Sq(SqList &L)
{
	L.elem = (ElemType *)malloc(LIST_INIT_SIZE * sizeof(ElemType));
	if (! L.elem) exit(OVERFLOW);
	L.length = 0;
	L.listsize = LIST_INIT_SIZE;
	
	return OK;
	
}

Status ListInsert_Sq(SqList &L, int i, ElemType e)
{
	ElemType * newbase;
	ElemType * p;
	ElemType * q;
	// 在顺序线性列表L中第i个位置之前插入新的元素e，
	// i的合法值为 l <= i <= ListLength_Sq(L) + 1
	if (i < 1 || i > L.length + 1) // i 值不合法
		return ERROR;
	
	if (L.length >= L.listsize) // 当前空间已满，需要增加配额
	{
		newbase = (ElemType *)realloc(L.elem, (L.listsize + LISTINCREMENT) * sizeof(ElemType));
		
		if(!newbase) exit(OVERFLOW); // 存储分配失败
		
		L.elem = newbase; // 新基地址
		L.listsize += LISTINCREMENT; // 增加存储容量
	}
	
	q = &(L.elem[i - 1]); // q为插入的位置
	
	for (p = &(L.elem[L.length -1]); p >= q; --p) // 插入位置及之后的元素后移
		*(p + 1) = *p;
	
	*q = e; // 插入元素
	++L.length; // 表长度增加
	
	return OK;
} // listInsert_Sq


