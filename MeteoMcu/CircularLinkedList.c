#include "CircularLinkedList.h"

#include <stdlib.h> 
#include <string.h>
#include <stdio.h>

// ------------------ CircularLinkedListElement


struct CircularLinkedListElement* CircularLinkedList_CreateElement(){
	char *buffer;
	struct CircularLinkedListElement* ret;
	
	ret = (struct CircularLinkedListElement*)calloc(1, sizeof(struct CircularLinkedListElement));
	
	buffer = (char*)calloc(CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE, sizeof(char));
	
	ret->content = buffer;
	ret->size = 0;
	
	return ret;
}

short CircularLinkedListElement_SetContent(struct CircularLinkedListElement* element, char* content, uint8_t size){
	
	if(size > CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE)
		size = CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE;
	
	memcpy(element->content, content, size);
	
	element->size = size;
	
	return size;
}

// ------------------ CircularLinkedList

struct CircularLinkedList* CircularLinkedList_Create(){
	uint8_t i;
	struct CircularLinkedListElement *first, *temp, *last;
	
	struct CircularLinkedList* ret;
	ret	= (struct CircularLinkedList*)calloc(1, sizeof(struct CircularLinkedList));
	
	for(i = 0; i < CIRCULAR_LINKED_LIST_ELEMENT_MAX_SIZE; i++){
		temp = CircularLinkedList_CreateElement();
		
		if(i == 0){
			first = temp;
			last = temp;
		}
		else{
			last->next = temp;
			last = temp;
		}
	}
	
	temp->next = first;
	
	ret->head = first;
	ret->tail = first;
	ret->size = 0;
	ret->maxSize = CIRCULAR_LINKED_LIST_ELEMENT_MAX_SIZE;
	ret->isLocked = 1;
	
	return ret;
}




short CircularLinkedList_PushBack(struct CircularLinkedList* list, char* buffer, uint8_t size){
	if(list->size == list->maxSize)
		return -1;
	
	CircularLinkedListElement_SetContent(list->tail, buffer, size);
	
	list->tail = list->tail->next;
	list->size++;
	
	return list->size;
}

short CircularLinkedList_PushBackBlocking(struct CircularLinkedList* list, char* buffer, uint8_t size){
	short ret;	
	
	while(CircularLinkedList_TryLock(list) != 0);
	
	ret = CircularLinkedList_PushBack(list, buffer, size);
	CircularLinkedList_Unlock(list);
	return ret;
}

short CircularLinkedList_PushBackNonBlocking(struct CircularLinkedList* list, char* buffer, uint8_t size){
	short ret;
	
	if(CircularLinkedList_TryLock(list) != 0)
		return -1;
	
	ret = CircularLinkedList_PushBack(list, buffer, size);
	CircularLinkedList_Unlock(list);
	return ret;
}
	
short CircularLinkedList_PopHead(struct CircularLinkedList* list, char* buffer){
	
	uint8_t i;
	
	uint8_t size;
	if(list->size == 0)
		return -1;
	//printf("list size %d, head size %d\n", list->size, list->head->size);
	
	// copy
	//memcpy(buffer, list->head->content, sizeof(char) * CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE);
	for(i = 0; i < CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE; i++)	// memcpy cause memory wrong
			buffer[i] = list->head->content[i];
	size = list->head->size;
	
	// clear element
	//memset(list->head->content, 0, sizeof(char) * CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE);
	for(i = 0; i < CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE; i++)	// memset cause memory wrong
			list->head->content[i] = 0x0;
	list->head->size = 0;
	
	list->head = list->head->next;
	list->size--;
	
	return size;
}

short CircularLinkedList_PopHeadBlocking(struct CircularLinkedList* list, char* buffer){
	short ret;	
	
	while(CircularLinkedList_TryLock(list) != 0);
	
	ret = CircularLinkedList_PopHead(list, buffer);
	CircularLinkedList_Unlock(list);
	return ret;
}

short CircularLinkedList_PopHeadNonBlocking(struct CircularLinkedList* list, char* buffer){
	short ret;
	
	if(CircularLinkedList_TryLock(list) != 0)
		return -1;
	
	ret = CircularLinkedList_PopHead(list, buffer);
	CircularLinkedList_Unlock(list);
	return ret;
}

short CircularLinkedList_Lock(struct CircularLinkedList* list){
	list->isLocked = 0;
	return 0;
}

short CircularLinkedList_Unlock(struct CircularLinkedList* list){
	list->isLocked = 1;
	return 0;
}
	
short CircularLinkedList_IsLocked(struct CircularLinkedList* list){
	if(list->isLocked == 0)
		return 0;
	else
		return -1;
}
	
short CircularLinkedList_TryLock(struct CircularLinkedList* list){
	if(list->isLocked == 0)
		return -1;
	list->isLocked = 0;
	return 0;
}


