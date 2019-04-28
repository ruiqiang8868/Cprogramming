// linux环境下，当前程序的编译命令
/* 
	gcc -o link_list -g link_list.cpp
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

#define ElemType int
#define Status   int

//单链表结构
typedef struct LNode
{
	ElemType data;
	struct LNode *next;
}LNode, *LinkList;


void CreateList_L(LinkList &L, int n);
void Display_List_L(LinkList &L, int n);
Status GetElem_L(LinkList L, int i, ElemType &e);
Status ListInsert_L(LinkList &L, int i, ElemType e);
Status ListDelete_L(LinkList &L, int i, ElemType &e);

int main(void)
{
	LinkList L;
	int n = 5;
	
	CreateList_L(L, n);
	Display_List_L(L, n);
	
	return 0;
}


void CreateList_L(LinkList &L, int n)
{
	// 逆位序输入n个元素的值，建立带表头节点的单链线性表L
	int i;
	LinkList p;
	
	L = (LinkList)malloc(sizeof(LNode)); // 先建立一个带头节点的单链表
	
	L->next = NULL;
	for(i = n; i > 0; --i)
	{
		p = (LinkList)malloc(sizeof(LNode)); // 生成新节点
		scanf("%d",&p->data); // 输入元素值
		p->next = L->next; // 将新节点插入到链表尾部
		L->next = p; // 原先的节点头指针指向新节点
	}
} // CreateList_L


void Display_List_L(LinkList &L, int n)
{
	LinkList p;
	int i;
	
	p = L;
	p = p->next;
	
	printf("Link list's elements are:\n");
	
	for(i = 0; i < n; ++i)
	{
		printf("%d\t", p->data);
		p = p->next;
	}
	printf("\n");
	
}//Display_List_L

	
Status GetElem_L(LinkList L, int i, ElemType &e)
{
	// L为带头节点的单链表的头指针
	// 当第i个元素存在时，其赋值给e并返回OK，否则返回ERROR
	LinkList p;
	int j;
	
	p = L->next;
	j = 1;
	while(p && (j < i))
	{
		p = p->next;
		++j;
	}
	if((!p->next) || (j > i)) return ERROR;
	e = p->data;
	return OK;
}// GetElem_L


Status ListInsert_L(LinkList &L, int i, ElemType e)
{
	LinkList p, s;
	int j = 0;
	
	p = L;
	
	while(p && j < i -1) // 查找插入的点
	{
		p = p->next;
		++j;
	}
	if(!p || j > i - 1) return ERROR;
	
	s = (LinkList)malloc(sizeof(LNode)); // 分配内存
	
	s->data = e; 
	s->next = p->next; //代插入数据的指针指向插入点后的数据
	p->next = s; // 插入点的数据指针指向插入数据
	
	return OK;
}//ListInsert_L


Status ListDelete_L(LinkList &L, int i, ElemType &e)
{
	LinkList p, q;
	int j = 0;
	
	while(p->next && j < i - 1) // 查找待删除的点
	{
		p = p->next;
		++j;
	}
	
	if(!(p->next) || j > i - 1) return ERROR;
	
	q = p->next; // 将待删除的节点的指针保存到q
	p->next = q->next; // 将q指向的下一个数据指针重新赋给p的指针
	e = q->data; // 将待删除的节点的数据赋给e
	
	free(q); // 释放内存
	
	return OK;
} //ListDelete_L












