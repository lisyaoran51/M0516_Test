/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 14/11/14 10:48a $
 * @brief    M051 Series I2C Driver Sample Code (Slave)
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

#include "I2cInterface.h"
#include <string.h>

uint8_t g_au8SlvSendData[CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE];
uint8_t g_au8SlvReceiveData[CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE];

volatile struct CircularLinkedList* inputBuffer;
volatile struct CircularLinkedList* outputBuffer;
volatile short isOutputEmpty;	/* 0:empty, -1:has output */


/*
 *	-1: last receive has no end of line.(0x60)
 *	-2: receive buffer overflow.(0x80)
 *	-3: send buffer overflow.(0xB8)
 *	-4: new status not processed(?)
 */
short errorCode; /* 0:nothing, other: error code */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8SlvSendDataLen;
volatile uint8_t g_u8SlvReceiveDataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRxTest(uint32_t u32Status){
		I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
}
	
void I2C_SlaveTRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
				if(g_u8SlvReceiveDataLen != 0)
				{
						//printf("Error: last receive has no end of line.\n");
						//CircularLinkedList_PushBack(inputBuffer, (char*)g_au8SlvReceiveData, g_u8SlvReceiveDataLen); // TODO: should use non-blocking function
				}
				
        g_u8SlvReceiveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvReceiveData[g_u8SlvReceiveDataLen] = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8SlvReceiveDataLen++;

        if((unsigned char) I2C_GET_DATA(I2C0) == '\0')	/* end of receive */
        {
						//printf("I2cInterface: read message end of line.\n");
						//CircularLinkedList_Lock(inputBuffer);
						//CircularLinkedList_PushBack(inputBuffer, (char*)g_au8SlvReceiveData, g_u8SlvReceiveDataLen); // TODO: should use non-blocking function
						//CircularLinkedList_Unlock(inputBuffer);
						g_u8SlvReceiveDataLen = 0;
						
						memset(g_au8SlvReceiveData, 0, sizeof(uint8_t) * CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE);
        }
				if(g_u8SlvReceiveDataLen == CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE){
					//printf("Error: receive buffer overflow.\n");
					g_u8SlvReceiveDataLen = 0;
				}

        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
				
				//if(CircularLinkedList_PopHeadNonBlocking(outputBuffer, (char*)g_au8SlvSendData) > 0){	/* has new message to read */
				//	isOutputEmpty = -1;
				//}
				//else{	/* no new message to read */
				//	isOutputEmpty = 0;
				//}
				
				
				I2C_SET_DATA(I2C0, 0x0);	/* if has message pi will read 128, if not pi will read 0 */
				g_u8SlvSendDataLen = 0;
				
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xB8)                  /* from datasheet to know 0xb8 is how to use TRM_ML51_Series_EN_Rev1.02.pdf */
    {

        I2C0->I2CDAT = g_au8SlvSendData[g_u8SlvSendDataLen];
				//I2C_SET_DATA(I2C0, g_au8SlvSendData[g_u8SlvSendDataLen]);
        g_u8SlvSendDataLen++;
				
				if(g_u8SlvSendDataLen == CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE){
					//printf("Error: send buffer overflow.\n");
					g_u8SlvSendDataLen = 0;
				}
			
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
				g_u8SlvSendDataLen = 0;
				if(isOutputEmpty == -1){
					memset(g_au8SlvSendData, 0, sizeof(uint8_t) * CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE);
					
				}
				
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
				/*
				printf("I2cInterface: pi read i2c over. left output buffer size:%d. next buffer %x size: %d,\n next 2 buffer %x size: %d,\n next 3 buffer %x size: %d\n", 
					outputBuffer->size, outputBuffer->head, outputBuffer->head->size,
					outputBuffer->head->next, outputBuffer->head->next->size,
					outputBuffer->head->next->next, outputBuffer->head->next->next->size
				);
				*/
				g_u8SlvSendDataLen = 0;
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        //g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
				// when pi reading i2c message, this will call when one over and another start
        //g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else
    {
        /* TO DO */
        //printf("Status 0x%x is NOT processed\n", u32Status);
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
}


void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    //printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0)); // don't know why this line ruins other memory

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Set I2C 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t I2cInterface_Init(struct CircularLinkedList* iBuffer, struct CircularLinkedList* oBuffer)
{
    uint8_t i;

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
		
		/*
    printf("+-------------------------------------------------------+\n");
    printf("|  M05xx I2C Driver Sample Code(Slave) for access Slave |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(P3.4), I2C0_SCL(P3.5)\n");
		*/

    /* Init I2C0 */
    I2C0_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);

		inputBuffer = iBuffer;
		outputBuffer = oBuffer;

    for(i = 0; i < CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE; i++)
    {
        g_au8SlvSendData[i] = 0;
        g_au8SlvReceiveData[i] = 0;
    }
		g_u8SlvSendDataLen = 0;
		g_u8SlvReceiveDataLen = 0;
		
		isOutputEmpty = 0;
		

    //printf("\n");
    //printf("I2C Slave Mode is Running.\n");
    /* I2C function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;
		//s_I2C0HandlerFn = I2C_SlaveTRxTest;
		
		return 0;
}

int32_t I2cInterface_Init_Test()
{
    uint8_t i;

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
		
		/*
    printf("+-------------------------------------------------------+\n");
    printf("|  M05xx I2C Driver Sample Code(Slave) for access Slave |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(P3.4), I2C0_SCL(P3.5)\n");
		*/

    /* Init I2C0 */
    I2C0_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);

		

    for(i = 0; i < CIRCULAR_LINKED_LIST_CONTENT_MAX_SIZE; i++)
    {
        g_au8SlvSendData[i] = 0;
        g_au8SlvReceiveData[i] = 0;
    }
		g_u8SlvSendDataLen = 0;
		g_u8SlvReceiveDataLen = 0;
		
		isOutputEmpty = 0;
		

    //printf("\n");
    //printf("I2C Slave Mode is Running.\n");
    /* I2C function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;
		//s_I2C0HandlerFn = I2C_SlaveTRxTest;
		
		return 0;
}

short I2cInterface_ErrorCode(){
		return errorCode;
}


