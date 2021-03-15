#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include "M051Series.h"

#include "CircularLinkedList.h"



int32_t I2cInterface_Init(struct CircularLinkedList* iBuffer, struct CircularLinkedList* oBuffer);
	
short I2cInterface_ErrorCode();

#ifdef __cplusplus
}
#endif

#endif