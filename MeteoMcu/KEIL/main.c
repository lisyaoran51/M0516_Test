#include <stdio.h>
#include "M051Series.h"


#include "I2cInterface.h"

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000


char buffer[16];
char inBuffer[16];

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART & I2C0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Configure the SDA0 & SCL0 of I2C0 pins */
    SYS->P3_MFP &= ~(SYS_MFP_P34_Msk | SYS_MFP_P35_Msk);
    SYS->P3_MFP |= (SYS_MFP_P34_SDA0 | SYS_MFP_P35_SCL0);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}




/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
		int i;
		uint8_t count;
		char first[16];
		
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("Start program.\n");
		
		struct CircularLinkedList* inputBuffer = CircularLinkedList_Create();
		struct CircularLinkedList* outputBuffer = CircularLinkedList_Create();
		
		 
		
		buffer[0] = 'a';
		buffer[1] = 'b';
		buffer[2] = 'c';
		CircularLinkedList_PushBack(outputBuffer, buffer, 3);
		CircularLinkedList_PushBack(outputBuffer, buffer, 3);
		CircularLinkedList_PushBack(outputBuffer, buffer, 3);
		
		
		printf("size1:%d, size2:%d, size3:%d, size4:%d, buffer size:%d\n", 
			outputBuffer->head->size, 
			outputBuffer->head->next->size,
			outputBuffer->head->next->next->size,
			outputBuffer->head->next->next->next->size,
			outputBuffer->size);
			
			printf("pos1:%x, pos2:%x, pos3:%x, pos4:%x pos4_size: %x\n", 
			outputBuffer->head, 
			outputBuffer->head->next,
			outputBuffer->head->next->next,
			outputBuffer->head->next->next->next,
			&(outputBuffer->head->next->next->next->size));
		
		
		
		I2cInterface_Init(inputBuffer, outputBuffer);
		
		
		printf("size4:%d\n", outputBuffer->head->next->next->next->size);
		//for(i = 0; i < 1000; i++)
		//		printf(" ");
				
		count = 0;
    while(1){
			//if(inputBuffer->size > 0)
			//	printf("I2cInterface: pi read i2c end.\n");
			
			short ret = inputBuffer->size;
				
			if(ret > 0){
					CircularLinkedList_PopHeadNonBlocking(inputBuffer, inBuffer);
					//count++;
					//first[count] = inBuffer[count];
					printf("Receive %s, size %d, input list size: %d\n", inBuffer, ret, inputBuffer->size);
			}
			//printf("");
			//ret = CircularLinkedList_PushBackBlocking(outputBuffer, buffer, 3);
			//printf("%d ", ret);
			
			
		}
}



