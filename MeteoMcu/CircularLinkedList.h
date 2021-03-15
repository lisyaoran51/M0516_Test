#ifndef CIRCULAR_LINKED_LIST_H
#define CIRCULAR_LINKED_LIST_H



#ifdef __cplusplus
extern "C"
{
#endif


#include "M051Series.h"
	
	
#define CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE 16
#define CIRCULAR_LINKED_LIST_ELEMENT_MAX_SIZE 4


struct CircularLinkedListElement{
	char* content;
	volatile struct CircularLinkedListElement* next;
	volatile uint8_t size;
};

struct CircularLinkedListElement* CircularLinkedList_CreateElement();

short CircularLinkedListElement_SetContent(struct CircularLinkedListElement* element, char* content, uint8_t size);


struct CircularLinkedList{
	volatile struct CircularLinkedListElement* head;
	volatile struct CircularLinkedListElement* tail;
	volatile uint8_t size;
	volatile uint8_t maxSize;
	volatile uint8_t isLocked;	/* 0:locked, 1:unlocked */
};


struct CircularLinkedList* CircularLinkedList_Create();

/*
 * >0:success, -1:failed
 */
short CircularLinkedList_PushBack(struct CircularLinkedList* list, char* buffer, uint8_t size);
short CircularLinkedList_PushBackBlocking(struct CircularLinkedList* list, char* buffer, uint8_t size);
short CircularLinkedList_PushBackNonBlocking(struct CircularLinkedList* list, char* buffer, uint8_t size);

/*
 * >0:success, -1:failed
 */
short CircularLinkedList_PopHead(struct CircularLinkedList* list, char* buffer);
short CircularLinkedList_PopHeadBlocking(struct CircularLinkedList* list, char* buffer);
short CircularLinkedList_PopHeadNonBlocking(struct CircularLinkedList* list, char* buffer);

/*
 * 0:success, -1:failed
 */
short CircularLinkedList_Lock(struct CircularLinkedList* list);
short CircularLinkedList_Unlock(struct CircularLinkedList* list);
short CircularLinkedList_IsLocked(struct CircularLinkedList* list);
short CircularLinkedList_TryLock(struct CircularLinkedList* list);




#ifdef __cplusplus
}
#endif

#endif