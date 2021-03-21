#line 1 "..\\main.c"
 








 
#line 1 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 12 "..\\main.c"
#line 1 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
 









 


















 









 



 
typedef enum IRQn
{
     
    NonMaskableInt_IRQn       = -14,       
    HardFault_IRQn            = -13,       
    SVCall_IRQn               = -5,        
    PendSV_IRQn               = -2,        
    SysTick_IRQn              = -1,        

     
    BOD_IRQn                  = 0,         
    WDT_IRQn                  = 1,         
    EINT0_IRQn                = 2,         
    EINT1_IRQn                = 3,         
    GPIO_P0P1_IRQn            = 4,         
    GPIO_P2P3P4_IRQn          = 5,         
    PWMA_IRQn                 = 6,         
    PWMB_IRQn                 = 7,         
    TMR0_IRQn                 = 8,         
    TMR1_IRQn                 = 9,         
    TMR2_IRQn                 = 10,        
    TMR3_IRQn                 = 11,        
    UART0_IRQn                = 12,        
    UART1_IRQn                = 13,        
    SPI0_IRQn                 = 14,        
    SPI1_IRQn                 = 15,        
    I2C0_IRQn                 = 18,        
    I2C1_IRQn                 = 19,        
    ACMP01_IRQn               = 25,        
    ACMP23_IRQn               = 26,        
    PWRWU_IRQn                = 28,        
    ADC_IRQn                  = 29         

} IRQn_Type;






 

 





#line 1 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
 




















 













 












 




 


 

 













#line 89 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"


 







#line 114 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

#line 1 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 116 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




















 






 


 



 


 









 







 







 






 








 







 







 









 









 
__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 



#line 268 "..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"



#line 619 "..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

   

#line 117 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




















 






 

 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 260 "..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 296 "..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 615 "..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

   

#line 118 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"








 
#line 143 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 






 
#line 159 "..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








   

#line 94 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\system_M051Series.h"
 









 







 
 
 











 




 





extern uint32_t SystemCoreClock;     
extern uint32_t CyclesPerUs;         
extern uint32_t PllClock;            

#line 65 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\system_M051Series.h"












 
extern void SystemInit(void);











 
extern void SystemCoreClockUpdate(void);







 
#line 95 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"



#pragma anon_unions











 
extern void SystemInit(void);


 
 
 





 

 




 

typedef struct
{







































 

    volatile uint32_t CR[2];          
    volatile uint32_t SR;             

} ACMP_T;






 

 















 












 











   
   


 



 

typedef struct
{











































































































































































































 

    volatile const  uint32_t ADDR[8];        
    volatile uint32_t ADCR;           
    volatile uint32_t ADCHER;         
    volatile uint32_t ADCMPR[2];      
    volatile uint32_t ADSR;           
    volatile const  uint32_t RESERVED[4];
    volatile uint32_t ADTDCR;         
} ADC_T;







 
 









 



























 






 


















 





















 


   
   


 




 


typedef struct
{































































































































































































































































































































 

    volatile uint32_t PWRCON;         
    volatile uint32_t AHBCLK;         
    volatile uint32_t APBCLK;         
    volatile uint32_t CLKSTATUS;      
    volatile uint32_t CLKSEL0;        
    volatile uint32_t CLKSEL1;        
    volatile uint32_t CLKDIV;         
    volatile uint32_t CLKSEL2;        
    volatile uint32_t PLLCON;         
    volatile uint32_t FRQDIV;         

} CLK_T;






 

 




























 









 



























































 



















 






 

































 












 









 





















 








   
   


 



 

typedef struct
{


















































 

    volatile uint32_t EBICON;         
    volatile uint32_t EXTIME;         
    volatile uint32_t EBICON2;        

} EBI_T;







 
 












 












 








   
   


 





 

typedef struct
{



















































































































 

    volatile uint32_t ISPCON;         
    volatile uint32_t ISPADR;         
    volatile uint32_t ISPDAT;         
    volatile uint32_t ISPCMD;         
    volatile uint32_t ISPTRG;         
    volatile const  uint32_t DFBADR;         
    volatile uint32_t FATCON;         
    volatile const  uint32_t RESERVED[9];   
    volatile uint32_t ISPSTA;         
} FMC_T;






 
 



















 









 



 



 











   
   


 



 

typedef struct
{



























































































































 

    volatile uint32_t PMD;            
    volatile uint32_t OFFD;           
    volatile uint32_t DOUT;           
    volatile uint32_t DMASK;          
    volatile uint32_t PIN;            
    volatile uint32_t DBEN;           
    volatile uint32_t IMD;            
    volatile uint32_t IEN;            
    volatile uint32_t ISRC;           

} GPIO_T;



typedef struct
{
































 

    volatile uint32_t DBNCECON;       

} GPIO_DBNCECON_T;







 

 
























 



 



 



 



 



 



 






 



 








   
   



 



 

typedef struct
{













































 

    volatile int32_t  DIVIDEND;       
    volatile int32_t  DIVISOR;        
    volatile int32_t  DIVQUO;         
    volatile int32_t  DIVREM;         
    volatile uint32_t DIVSTS;         

} HDIV_T;







 







   
   



 




 

typedef struct
{
























































































































































































 

    volatile uint32_t I2CON;          
    volatile uint32_t I2CADDR0;       
    volatile uint32_t I2CDAT;         
    volatile const  uint32_t I2CSTATUS;      
    volatile uint32_t I2CLK;          
    volatile uint32_t I2CTOC;         
    volatile uint32_t I2CADDR1;       
    volatile uint32_t I2CADDR2;       
    volatile uint32_t I2CADDR3;       
    volatile uint32_t I2CADM0;        
    volatile uint32_t I2CADM1;        
    volatile uint32_t I2CADM2;        
    volatile uint32_t I2CADM3;        
    volatile const  uint32_t RESERVED0[2]; 
    volatile uint32_t I2CWKUPCON;     
    volatile uint32_t I2CWKUPSTS;     

} I2C_T;






 

 


















 






 



 



 



 









 



 



 



   
   



 




 

typedef struct
{























































































































































































































































































































































































































































































































































































































































 

    volatile uint32_t PPR;            
    volatile uint32_t CSR;            
    volatile uint32_t PCR;            
    volatile uint32_t CNR0;           
    volatile uint32_t CMR0;           
    volatile const  uint32_t PDR0;           
    volatile uint32_t CNR1;           
    volatile uint32_t CMR1;           
    volatile const  uint32_t PDR1;           
    volatile uint32_t CNR2;           
    volatile uint32_t CMR2;           
    volatile const  uint32_t PDR2;           
    volatile uint32_t CNR3;           
    volatile uint32_t CMR3;           
    volatile const  uint32_t PDR3;           
    volatile const  uint32_t RESERVE0;     
    volatile uint32_t PIER;           
    volatile uint32_t PIIR;           
    volatile const  uint32_t RESERVE1[2];  
    volatile uint32_t CCR0;           
    volatile uint32_t CCR2;           
    volatile uint32_t CRLR0;          
    volatile uint32_t CFLR0;          
    volatile uint32_t CRLR1;          
    volatile uint32_t CFLR1;          
    volatile uint32_t CRLR2;          
    volatile uint32_t CFLR2;          
    volatile uint32_t CRLR3;          
    volatile uint32_t CFLR3;          
    volatile uint32_t CAPENR;         
    volatile uint32_t POE;            
    volatile uint32_t TCON;           
    volatile uint32_t TSTATUS;        
    volatile const  uint32_t RESERVE2[4];  
    volatile uint32_t PSCR;           

} PWM_T;






 

 












 












 




























































 



 



 




 




































 
























 










































 










































 



 



 












 












 
























 












 











   
   



 



 

typedef struct
{


































































































































































































































































































































































































 

    volatile uint32_t CNTRL;          
    volatile uint32_t DIVIDER;        
    volatile uint32_t SSR;            
    volatile const  uint32_t RESERVE0;     
    volatile const  uint32_t RX0;            
    volatile const  uint32_t RX1;            
    volatile const  uint32_t RESERVE1[2];  
    volatile  uint32_t TX0;            
    volatile  uint32_t TX1;            
    volatile const  uint32_t RESERVE2[3];  
    volatile uint32_t VARCLK;         
    volatile const  uint32_t RESERVE3;     
    volatile uint32_t CNTRL2;         
    volatile uint32_t FIFO_CTL;       
    volatile uint32_t STATUS;         

} SPI_T;






 

 






















































 






 















 





















 
























 



































   
   



 




 

typedef struct
{



























































































































































































































































































































































































































































































































































 

    volatile const  uint32_t PDID;           
    volatile uint32_t RSTSRC;         
    volatile uint32_t IPRSTC1;        
    volatile uint32_t IPRSTC2;        
    volatile const  uint32_t RESERVED0[2]; 
    volatile uint32_t BODCR;          
    volatile uint32_t TEMPCR;         
    volatile const  uint32_t RESERVED1;    
    volatile uint32_t PORCR;          
    volatile const  uint32_t RESERVED2[2]; 
    volatile uint32_t P0_MFP;         
    volatile uint32_t P1_MFP;         
    volatile uint32_t P2_MFP;         
    volatile uint32_t P3_MFP;         
    volatile uint32_t P4_MFP;         
    volatile const  uint32_t RESERVED3[47];
    volatile uint32_t REGWRPROT;      

} GCR_T;







 

 





















 












 


















































 





















 



 



 












 









 









 









 









 





   



typedef struct
{

























































































































 

    volatile const  uint32_t IRQSRC[32];     
    volatile uint32_t NMISEL;         

} GCR_INT_T;






 

 



 





   
   



 



 

typedef struct
{








































































































































 

    volatile uint32_t TCSR;           
    volatile uint32_t TCMPR;          
    volatile uint32_t TISR;           
    volatile uint32_t TDR;            
    volatile uint32_t TCAP;           
    volatile uint32_t TEXCON;         
    volatile uint32_t TEXISR;         

} TIMER_T;






 

 










































 



 






 



 



 





















 


   
   



 




 

typedef struct
{




















































































































































































































































































































































































































 

    union {
        volatile uint32_t DATA;           
        volatile uint32_t THR;            
        volatile uint32_t RBR;            
    };
    volatile uint32_t IER;            
    volatile uint32_t FCR;            
    volatile uint32_t LCR;            
    volatile uint32_t MCR;            
    volatile uint32_t MSR;            
    volatile uint32_t FSR;            
    volatile uint32_t ISR;            
    volatile uint32_t TOR;            
    volatile uint32_t BAUD;           
    volatile uint32_t IRCR;           
    volatile uint32_t ALT_CSR;        
    volatile uint32_t FUN_SEL;        
} UART_T;






 

 



 



 

































 















 


















 









 









 







































 










































 






 












 









 
























 


   
   



 



 

typedef struct
{




















































 

    volatile uint32_t WTCR;           
    volatile uint32_t WTCRALT;        

} WDT_T;






 

 






























 


   
   


 



 

typedef struct
{

























































 

    volatile uint32_t WWDTRLD;        
    volatile uint32_t WWDTCR;         
    volatile uint32_t WWDTSR;         
    volatile uint32_t WWDTCVR;        

} WWDT_T;






 

 



 















 






 


   
   
   


 
 
 



 
 






 
#line 5779 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"









































   

 
 
 




 
#line 5836 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"







































   



typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;




#line 5892 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"

#line 5899 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"













 
#line 5945 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"


 











 
 
 
#line 1 "..\\..\\Library\\StdDriver\\inc\\sys.h"
 









 











 



 



 

 
 
 
#line 54 "..\\..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 65 "..\\..\\Library\\StdDriver\\inc\\sys.h"


 
 
 







 





#line 89 "..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 96 "..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 103 "..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 110 "..\\..\\Library\\StdDriver\\inc\\sys.h"











































#line 159 "..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 166 "..\\..\\Library\\StdDriver\\inc\\sys.h"













































#line 217 "..\\..\\Library\\StdDriver\\inc\\sys.h"






#line 229 "..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 236 "..\\..\\Library\\StdDriver\\inc\\sys.h"







#line 249 "..\\..\\Library\\StdDriver\\inc\\sys.h"












#line 267 "..\\..\\Library\\StdDriver\\inc\\sys.h"











































   



 







 








 








 








 








 









 








 








 








 












 








 








 








 









 








 








 








 








 








 








 








 














 








 
static __inline void SYS_LockReg(void)
{
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0;
}







 
static __inline void SYS_UnlockReg(void)
{
    while(((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT != (1ul << 0))
    {
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x59;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x16;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x88;
    }
}




void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);


   

   

   






#line 5963 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\clk.h"
 









 













 



 



 







 
 
 
#line 53 "..\\..\\Library\\StdDriver\\inc\\clk.h"


 
 
 
#line 99 "..\\..\\Library\\StdDriver\\inc\\clk.h"


 
 
 
#line 118 "..\\..\\Library\\StdDriver\\inc\\clk.h"


 
 
 





 
 
 














 
 
 
#line 157 "..\\..\\Library\\StdDriver\\inc\\clk.h"
 
 
 





#line 185 "..\\..\\Library\\StdDriver\\inc\\clk.h"


   



 







 
static __inline uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq = 0, u32PllReg;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4};

    u32PllReg = ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PLLCON;

    if(u32PllReg & ((1ul << 16) | (1ul << 18)))
        return 0;            

    if(u32PllReg & 0x00080000UL)
        u32FIN = (22118400UL);     
    else
        u32FIN = (12000000UL);      

    if(u32PllReg & (1ul << 17))
        return u32FIN;       

     
    u32NO = au8NoTbl[((u32PllReg & (3ul << 14)) >> 14)];
    u32NF = ((u32PllReg & (0x1FFul << 0)) >> 0) + 2;
    u32NR = ((u32PllReg & (0x1Ful << 9)) >> 9) + 2;

     
    u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);

    return u32PllFreq;
}









 
static __inline void CLK_SysTickDelay(uint32_t us)
{
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x00);
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

     
    while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0);
    
     
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0;    
}








 
static __inline void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t delay;
        
     
    delay = 335544UL;

    do
    {
        if(us > delay)
        {
            us -= delay;
        }
        else
        {
            delay = us;
            us = 0UL;
        }        
        
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = delay * CyclesPerUs;
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x0UL);
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

         
        while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0UL);

         
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;
    
    }while(us > 0UL);
    
}


void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);




   

   

   











 
#line 5964 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\adc.h"
 








 











 



 



 
 
 
 

























 
 
 




 
 
 
#line 75 "..\\..\\Library\\StdDriver\\inc\\adc.h"

 
 
 




 
 
 





 
 
 





 
 
 



 
 
 





   



 











 








 












 












 








 









 









 







 







 















 
#line 235 "..\\..\\Library\\StdDriver\\inc\\adc.h"






 















 
#line 267 "..\\..\\Library\\StdDriver\\inc\\adc.h"






 









 











 







 














 


void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);



   

   

   







 
#line 5965 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\ebi.h"
 








 











 



 



 
 
 
 



 
 
 



 
 
 
#line 52 "..\\..\\Library\\StdDriver\\inc\\ebi.h"

#line 60 "..\\..\\Library\\StdDriver\\inc\\ebi.h"

   




 









 











 










 











 










 











 


void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint32_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);

   

   

   







 
#line 5966 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\fmc.h"
 









 



#line 1 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
 









 


















 

#line 5978 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"

 



#line 16 "..\\..\\Library\\StdDriver\\inc\\fmc.h"









 



 



 


 
 
 







 
 
 



 
 
 
#line 62 "..\\..\\Library\\StdDriver\\inc\\fmc.h"


   



 

 
 
 









 












 













 













 














 











 













 












 













 















 














 



 
 
 











 
static __inline void FMC_Write(uint32_t u32addr, uint32_t u32data)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x21;    
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT = u32data;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                   
    __isb(0xF);                             
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                  
}










 
static __inline uint32_t FMC_Read(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x00;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;              
    __isb(0xF);                        
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);             

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}













 
static __inline int32_t FMC_Erase(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x22;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;                
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                    
    __isb(0xF);                              
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                   

     
    if(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON & (1ul << 6))
    {
        ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON |= (1ul << 6);
        return -1;
    }
    return 0;
}










 
static __inline uint32_t FMC_ReadUID(uint8_t u8index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (u8index << 2);       
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                  
    __isb(0xF);                            
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                 

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}











 
static __inline uint32_t FMC_ReadCID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0B;            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x0;                            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);           
    __isb(0xF);                                      
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0)) ;   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadPID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0C;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x04;                          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (0x04 * u32Index) + 0x10;      
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}















 
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x2e;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32PageAddr;        
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                
    __isb(0xF);                          
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);               
}














 
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPSTA & (0xffful << 9));
}

extern void FMC_Open(void);
extern void FMC_Close(void);
extern void FMC_EnableAPUpdate(void);
extern void FMC_DisableAPUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern void FMC_SetBootSource(int32_t i32BootSrc);
extern int32_t FMC_GetBootSource(void);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);

   

   

   








#line 5967 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\gpio.h"
 









 











 



 



 


 
 
 





 
 
 






 
 
 



 
 
 






#line 82 "..\\..\\Library\\StdDriver\\inc\\gpio.h"














 
#line 138 "..\\..\\Library\\StdDriver\\inc\\gpio.h"
   




 











 












 











 












 












 












 












 













 































 










 











 










 













 












 














 












 



void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);


   

   

   







 
#line 5968 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\i2c.h"
 









 



#line 16 "..\\..\\Library\\StdDriver\\inc\\i2c.h"









 



 



 

 
 
 
#line 47 "..\\..\\Library\\StdDriver\\inc\\i2c.h"




   



 









 










 










 










 











 










 











 











 










 


 
 
 









 
static __inline void I2C_STOP(I2C_T *i2c)
{
    (i2c)->I2CON |= ((1ul << 3) | (1ul << 4));
    while((i2c)->I2CON & (1ul << 4));
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);

   

   

   

#line 5969 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\pwm.h"
 








 











 



 



 
#line 47 "..\\..\\Library\\StdDriver\\inc\\pwm.h"
 
 
 






   




 










 
#line 81 "..\\..\\Library\\StdDriver\\inc\\pwm.h"










 
#line 100 "..\\..\\Library\\StdDriver\\inc\\pwm.h"










 
#line 120 "..\\..\\Library\\StdDriver\\inc\\pwm.h"









 










 













 

















 













 














 














 
#line 225 "..\\..\\Library\\StdDriver\\inc\\pwm.h"


uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequncy,
                                 uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



   

   

   







 
#line 5970 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\spi.h"
 








 











 



 



 













#line 50 "..\\..\\Library\\StdDriver\\inc\\spi.h"









   




 






 







 







 







 







 







 








 








 








 







 







 








 








 







 







 







 







 











 







 







 








 








 







 



 
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void SPI_DisableFIFO(SPI_T *spi);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);

   

   

   







 
#line 5971 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\timer.h"
 








 











 



 



 

#line 48 "..\\..\\Library\\StdDriver\\inc\\timer.h"

   




 













 












 











 













 










 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 30);
}









 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 30);
}











 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 23);
}









 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 23);
}









 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 6);
}









 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 6);
}









 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 7);
}









 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 7);
}









 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 29);
}









 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 29);
}









 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 5);
}









 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 5);
}










 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 0) ? 1 : 0);
}









 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->TEXISR;
}









 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 1) ? 1 : 0);
}









 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 1);
}









 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}









 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->TDR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);

   

   

   







 
#line 5972 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\wdt.h"
 








 











 



 



 
 
 
 
#line 42 "..\\..\\Library\\StdDriver\\inc\\wdt.h"

 
 
 





   




 









 










 










 











 











 











 













 










 
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR = 0;
    return;
}









 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR |= (1ul << 6);
    return;
}









 
static __inline void WDT_DisableInt(void)
{
    
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR &= ~((1ul << 6) | (1ul << 2) | (1ul << 3));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

   

   

   







 
#line 5973 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\wwdt.h"
 








 











 



 



 
 
 
 
#line 50 "..\\..\\Library\\StdDriver\\inc\\wwdt.h"



   




 









 










 











 











 










 













 


void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);

   

   

   







 
#line 5974 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\uart.h"
 








 












 



 



 

 
 
 



    
 
 
 











 
 
 
















 
 
 



 
 
 



 
 
 





 
 
 





   




 











 











 












 











 












 












 











 











 











 












 











 












 












 





















 




















 



























 











 
static __inline void UART_CLEAR_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9);
    (uart)->MCR &= ~(1ul << 1);
}








 
static __inline void UART_SET_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9) | (1ul << 1);
}










 












 



void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart);
void UART_DisableFlowCtrl(UART_T* uart);
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T* uart);
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 

#line 5975 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\hdiv.h"
 









 














 
static __inline int32_t HDIV_Div(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)((( uint32_t)0x50000000) + 0x14000);
    *p32++ = x;
    *p32++ = y;
    return *p32;
}











 
static __inline int16_t HDIV_Mod(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)((( uint32_t)0x50000000) + 0x14000);
    *p32++ = x;
    *p32++ = y;
    return p32[1];
}





#line 5976 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"
#line 1 "..\\..\\Library\\StdDriver\\inc\\acmp.h"
 








 











 



 



 

 
 
 
#line 44 "..\\..\\Library\\StdDriver\\inc\\acmp.h"

   




 







 








 











 








 








 








 








 









 








 








 








 








 



 
void ACMP_Open(ACMP_T *, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *, uint32_t u32ChNum);

   

   

   







 
#line 5977 "..\\..\\Library\\Device\\Nuvoton\\M051Series\\Include\\M051Series.h"


 



#line 13 "..\\main.c"
#line 1 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 14 "..\\main.c"
#line 1 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 15 "..\\main.c"
#line 1 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"




 





 












 






   









 






#line 61 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 75 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   




 















 
#line 112 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











 





extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double  , double  );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double  );
     
     

__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
__inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 230 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   
  typedef float float_t;
  typedef double double_t;
#line 251 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"



extern const int math_errhandling;
#line 261 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    __inline double _sqrt(double __x) { return sqrt(__x); }




    __inline float _sqrtf(float __x) { return (float)sqrt(__x); }

    



 

extern __declspec(__nothrow) __attribute__((const)) double ceil(double  );
    
    
extern __declspec(__nothrow) __attribute__((const)) double fabs(double  );
    
    

extern __declspec(__nothrow) __attribute__((const)) double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
__inline __declspec(__nothrow) __attribute__((const)) double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
__inline __declspec(__nothrow) __attribute__((const)) float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 479 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __attribute__((const)) double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __attribute__((const)) float _fabsf(float);  
__inline __declspec(__nothrow) __attribute__((const)) float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __attribute__((const)) float ceilf(float  );
extern __declspec(__nothrow) __attribute__((const)) float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );



#line 875 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"





#line 896 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 1087 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











#line 1317 "c:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"





 
#line 16 "..\\main.c"





 
 
 

		

 
		uint32_t pressedKeyTime[48] = {0};
		
		




 
		uint8_t pressedKey[48] = {0};		
		
		uint8_t powerButton = 0;
		uint8_t sensitiveButton = 0;
		uint8_t sustainButton = 0;
		uint8_t lowerOctaceButton = 0;
		uint8_t raiseOctaceButton = 0;
		uint8_t pauseButton = 0;
		uint8_t speedButton = 0;
		
		uint8_t pedalpluggedIn = 0;
		uint8_t pedalDown = 0;
		
		



 
		uint8_t sectionKnobState = 0;
		
		


 
 
 

void AdcContScanModeTest(void);


 
 
 
double sliderDataAccumulation[2] = {0};
int sliderDataAverage[2] = {0};
uint8_t adcScanCount = 0;


 
 
 

uint8_t indicatorLights[16] = {0};





 
 
 

uint8_t i2cWriteData[16][16];		
uint8_t i2cReadData[16][16];		

volatile uint8_t i2cWriteDataStartPos = 0;
volatile uint8_t i2cWriteDataEndPos = 0;

volatile uint8_t i2cReadDataStartPos = 0;
volatile uint8_t i2cReadDataEndPos = 0;

uint8_t i2cTempWriteBuffer[16] = {0};
uint8_t i2cTempReadBuffer[16] = {0};

volatile uint8_t writeDataLen = 0;
volatile uint8_t readDataLen = 0;

 
 
 

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = 0;

 
 
 


 
 
 

volatile uint32_t timerCount = 0;









 
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(((TIMER_T *) ((( uint32_t)0x40000000) + 0x10000))) == 1)
    {
         
        TIMER_ClearIntFlag(((TIMER_T *) ((( uint32_t)0x40000000) + 0x10000)));

        timerCount++;
    }
}


void SYS_Init(void)
{
     
     
     

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PWRCON |= (1ul << 2);

     
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 4)));

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 = (((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 & (~(7ul << 0))) | 0x07UL;
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKDIV = (((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKDIV & (~(0xFul << 0))) | ((1)-1);
		

		((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKDIV &= ~(0xFul << 0);

	
     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PLLCON |= (1ul << 16);    
    
     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PWRCON |= (1ul << 0);
		

		((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PWRCON |= (1ul << 0) | (1ul << 2);



		 
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PWRCON |= (1ul << 0) | (1ul << 3);

		
     
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 0)));
		
		

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PLLCON = (0x00000000UL | (((3)-2)<<9) | ((25)-2) | 0x4000UL);
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 2)));
		

		while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & ((1ul << 2) | (1ul << 0) | (1ul << 4))));

		
		

     
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 2)));
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 0)));
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 3)));

		
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 = (((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 & (~(7ul << 0))) | 0x02UL;


		 
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 = 0x18UL | 0x02UL;

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK = (1ul << 16) | (1ul << 20) | (1ul << 21) | (1ul << 22) | (1ul << 23);
     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 = 0x01000000UL;
     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 = 0x00000000UL | 0x00000000UL;
		((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL2 = 0x00000000UL | 0x00000000UL;

     
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 = (1ul << 20);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 = 0;
		
		((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 = (1ul << 21);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 = 0;



		 
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK |= (1ul << 2);
		
		 
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 |= 0x01000000UL | 0x00000500UL;
		


     
     
    
    PllClock        = 50000000;            
    SystemCoreClock = 50000000 / 1;        
    CyclesPerUs     = 50000000 / 1000000;  

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK |= (1ul << 16);


		 
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK |= (1ul << 28) ;



     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 = (((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 & (~(3ul << 24))) | 0x01000000UL;


     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 &= ~(3ul << 2) ;
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 |= 0x0000000CUL ;
		
		 
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKDIV  = (((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKDIV & ~(0xFFul << 16)) | (((7) - 1) << 16);


     
     
     

     
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P3_MFP &= ~(0x00000101UL | 0x00000202UL);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P3_MFP |= (0x00000001UL | 0x00000002UL);
		

		 
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P2_MFP &= ~(0x00000101UL | 0x00000202UL | 0x00000404UL | 0x00000808UL | 0x00001010UL | 0x00002020UL | 0x00004040UL | 0x00008080UL);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P2_MFP |= 0x00000100UL | 0x00000200UL | 0x00000400UL | 0x00000800UL | 0x00001000UL | 0x00002000UL | 0x00004000UL | 0x00008000UL;

		

		 
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P1_MFP &= ~(0x00000101UL | 0x00000202UL | 0x00000404UL); 
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P1_MFP |= 0x00000001UL | 0x00000002UL | 0x00000004UL; 

		
}

void I2C0_SYS_Init(void)
{
     
     
     

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PWRCON |= (1ul << 2);

     
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 4)));

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 &= ~(7ul << 0);
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 |= 0x07UL;

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PWRCON |= (1ul << 0);

     
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 0)));

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PLLCON = (0x00000000UL | (((3)-2)<<9) | ((25)-2) | 0x4000UL);
    while(!(((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSTATUS & (1ul << 2)));
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 &= (~(7ul << 0));
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 |= 0x02UL;

     
     
    
    PllClock        = 50000000;            
    SystemCoreClock = 50000000 / 1;        
    CyclesPerUs     = 50000000 / 1000000;  

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK |= (1ul << 16) | (1ul << 8);

     
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 &= ~(3ul << 24);
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL1 |= 0x00000000UL;

     
     
     
     
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P3_MFP &= ~(0x00000101UL | 0x00000202UL);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P3_MFP |= (0x00000001UL | 0x00000002UL);

     
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P3_MFP &= ~(0x00001010UL | 0x00002020UL);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->P3_MFP |= (0x00001000UL | 0x00002000UL);
}

void UART0_Init()
{
     
     
     
     
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 |=  (1ul << 16);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 &= ~(1ul << 16);

     
    ((UART_T *) ((( uint32_t)0x40000000) + 0x50000))->BAUD = ((1ul << 29) | (1ul << 28)) | ((((50000000) + ((115200)/2)) / (115200))-2);
    ((UART_T *) ((( uint32_t)0x40000000) + 0x50000))->LCR = (3) | (0x0 << 3) | (0x0 << 2);
}

void setPwm(){
	
		printf("setPWM\n");
	
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 0))) | (0x1UL << 0);
    ((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 2))) | (0x1UL << 2);
    ((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 4))) | (0x1UL << 4);
    ((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 6))) | (0x1UL << 6);
    ((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 8))) | (0x1UL << 8);
    ((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 10))) | (0x1UL << 10);
    ((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 12))) | (0x1UL << 12);
    ((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0080))->PMD & (~(0x3ul << 14))) | (0x1UL << 14);
	
		 
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 3);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 11);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 19);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 27);
	
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 3);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 11);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 19);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 27);

		 
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x0) >> 1) * 8))) | ((0x01) << (((0x0) >> 1) * 8))); 
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x1) >> 1) * 8))) | ((0x01) << (((0x1) >> 1) * 8))); 
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x2) >> 1) * 8))) | ((0x01) << (((0x2) >> 1) * 8))); 
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x3) >> 1) * 8))) | ((0x01) << (((0x3) >> 1) * 8))); 
	
		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x0) >> 1) * 8))) | ((0x02) << (((0x0) >> 1) * 8))); 
		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x1) >> 1) * 8))) | ((0x02) << (((0x1) >> 1) * 8))); 
		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x2) >> 1) * 8))) | ((0x02) << (((0x2) >> 1) * 8))); 
		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->PPR & ~((0xFFul << 0) << (((0x3) >> 1) * 8))) | ((0x02) << (((0x3) >> 1) * 8))); 
	
	
		 
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x0) * 4))) | (((3UL)) << ((0x0) * 4)));
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x1) * 4))) | (((3UL)) << ((0x1) * 4)));
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x2) * 4))) | (((3UL)) << ((0x2) * 4)));
		((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40000000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x3) * 4))) | (((3UL)) << ((0x3) * 4)));

		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x0) * 4))) | (((3UL)) << ((0x0) * 4)));
		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x1) * 4))) | (((3UL)) << ((0x1) * 4)));
		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x2) * 4))) | (((3UL)) << ((0x2) * 4)));
		((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR = ((((PWM_T *) ((( uint32_t)0x40100000) + 0x40000)))->CSR & ~((7ul << 0) << ((0x3) * 4))) | (((3UL)) << ((0x3) * 4)));
		
		 
		
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR0 = 0x0;
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR1 = 0x0;
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR2 = 0x0;
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR3 = 0x0;
		
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR0 = 0x0;
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR1 = 0x0;
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR2 = 0x0;
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR3 = 0x0;
	
		 
		
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CNR0 = 0x1000;
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CNR1 = 0x1000;
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CNR2 = 0x1000;
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CNR3 = 0x1000;
											
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CNR0 = 0x1000;
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CNR1 = 0x1000;
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CNR2 = 0x1000;
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CNR3 = 0x1000;

		 
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->POE |= (1ul << 0);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->POE |= (1ul << 1);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->POE |= (1ul << 2);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->POE |= (1ul << 3);
		
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->POE |= (1ul << 0);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->POE |= (1ul << 1);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->POE |= (1ul << 2);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->POE |= (1ul << 3);

		 
		
		
		
		

		 
		

		 
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 0);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 8);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 16);
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->PCR |= (1ul << 24);
		
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 0);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 8);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 16);
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->PCR |= (1ul << 24);

		

		 
		 
		 
		 
		 

		 
		

		 
		

		 
		

		 
		
	
}

void setGpio(){
	
		 
    
    
		
		
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 0))) | (0x0UL << 0);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 2))) | (0x0UL << 2);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 4))) | (0x0UL << 4);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 6))) | (0x0UL << 6);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 8))) | (0x0UL << 8);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 10))) | (0x0UL << 10);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 12))) | (0x0UL << 12);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) ))->PMD & (~(0x3ul << 14))) | (0x0UL << 14);
		
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD & (~(0x3ul << 4))) | (0x1UL << 4);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD & (~(0x3ul << 6))) | (0x1UL << 6);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD & (~(0x3ul << 12))) | (0x1UL << 12);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x00C0))->PMD & (~(0x3ul << 14))) | (0x1UL << 14);
		
		
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD & (~(0x3ul << 8))) | (0x1UL << 8);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD & (~(0x3ul << 10))) | (0x1UL << 10);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD & (~(0x3ul << 14))) | (0x1UL << 14);
		
		
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0100))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0100))->PMD & (~(0x3ul << 0))) | (0x0UL << 0);
		((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD = (((GPIO_T *) (((( uint32_t)0x50000000) + 0x4000) + 0x0040))->PMD & (~(0x3ul << 6))) | (0x0UL << 6);
	
}


 
 
 
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CSTATUS;

    if(((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CTOC & (1ul << 0))
    {
         
        ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CTOC |= (1ul << 0);
    }
    else
    {
        if(s_I2C0HandlerFn != 0)
            s_I2C0HandlerFn(u32Status);
    }
}

 
 
 
void I2C_SlaveTRx(uint32_t u32Status)
{  	
		uint32_t i;
	
    if(u32Status == 0x60)                        
    {
        
        ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
    else if(u32Status == 0x80)                 
 
    {
				i2cTempReadBuffer[readDataLen] = (unsigned char)(((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CDAT);
			
				readDataLen++;
			
				if((unsigned char) ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CDAT) == '\0')	 
        {
					i2cReadDataEndPos++;
					if(i2cReadDataEndPos == 16)
						i2cReadDataEndPos = 0;
					
					
					if(i2cReadDataEndPos == i2cReadDataStartPos){
						i2cReadDataStartPos++;
						if(i2cReadDataStartPos == 16)
							i2cReadDataStartPos = 0;
					}
					
					memcpy(i2cReadData[i2cReadDataEndPos], i2cTempReadBuffer, 16);
					memset(i2cTempReadBuffer, 0, 16);
					
					readDataLen = 0;
				}
				
				
        ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
    else if(u32Status == 0xA8)                   
    {
				if(i2cWriteDataStartPos == i2cWriteDataEndPos){
					((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CDAT = 0x0;
					
					
					memset(i2cTempWriteBuffer, 0, 16);
					
				}
				else{
					
					memcpy(i2cTempWriteBuffer, i2cWriteData[i2cWriteDataStartPos], 16);
					
					((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CDAT = 0x80;
					
					i2cWriteDataStartPos++;
					if(i2cWriteDataStartPos == 16)
						i2cWriteDataStartPos = 0;
					
				}
				
				((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
		else if(u32Status == 0xB8)                   
    {
				if(writeDataLen == 16){
					((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CDAT = 0x0;
					
				}
				else{
					((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CDAT = i2cTempWriteBuffer[writeDataLen++];
				}
			
				((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
    else if(u32Status == 0xC0)                 
 
    {
				if(writeDataLen == 16){
					((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CDAT = 0x0;
				}
				else{
					((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CDAT = i2cTempWriteBuffer[writeDataLen++];
				}
				writeDataLen = 0;
        ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
    else if(u32Status == 0x88)                 
 
    {
        
        ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
    else if(u32Status == 0xA0)                 
 
    {
        
        ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
		
    else
    {
         
        printf("Status 0x%x is NOT processed\n", u32Status);
				((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
    }
}

void I2C0_Init(void)
{
     
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 |=  (1ul << 8);
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRSTC2 &= ~(1ul << 8);

     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CON |= (1ul << 6);

     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CLK = 125 - 1;

     
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CLK) + 1) << 2)));

     
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR0 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR0 & ~(0x7Ful << 1)) | (0x15 << 1);
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR1 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR1 & ~(0x7Ful << 1)) | (0x35 << 1);
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR2 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR2 & ~(0x7Ful << 1)) | (0x55 << 1);
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR3 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADDR3 & ~(0x7Ful << 1)) | (0x75 << 1);

     
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM0 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM0 & ~(0x7Ful << 1)) | (0x01 << 1);
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM1 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM1 & ~(0x7Ful << 1)) | (0x04 << 1);
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM2 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM2 & ~(0x7Ful << 1)) | (0x01 << 1);
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM3 = (((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CADM3 & ~(0x7Ful << 1)) | (0x04 << 1);

     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CON |= (1ul << 7);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CON &= ~(1ul << 7);
    NVIC_DisableIRQ(I2C0_IRQn);

     
    ((I2C_T *) ((( uint32_t)0x40000000) + 0x20000))->I2CON &= ~(1ul << 6);
    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK &= ~(1ul << 8);
}

 
 
 
 
 
 
 
 
 
 
 
 
void AdcContScanModeTest()
{
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;
		double conversedData;

    
    
    
    
    

    

		 
		((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADCR = ((3UL<<2) | (0UL<<10) | (1UL<<0));
		 
		((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADCHER |= ((((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADCHER & ~(0xFFul << 0)) | (0x7));

		 
		((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADSR = (1ul << 0);

		 
		((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADCR |= (1ul << 11);
		
		
		
		
		
		
		
		
		
		
		
		
		
		

		 
		while(!((((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADSR & (1ul << 0)) >> 0));

		 
		((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADCR &= ~(1ul << 11);
		
		for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
		{
				i32ConversionData = (((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADDR[(u32ChannelCount)] & (0xFFFFul << 0)) >> 0;
				
				conversedData = log((double)(3710 - i32ConversionData)) / log(1.04375021467);
				sliderDataAccumulation[u32ChannelCount] += conversedData;
				
		}
		
		
		 
		((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADSR = (1ul << 0);

    adcScanCount++;
		if(adcScanCount >= 10.0){
				if((int)(sliderDataAccumulation[0] / 10.0) > sliderDataAverage[0] + 1 || 
					 (int)(sliderDataAccumulation[0] / 10.0) < sliderDataAverage[0] - 1){
						sliderDataAverage[0] = (int)(sliderDataAccumulation[0] / 10.0);
						printf("slider 1:%d\n", sliderDataAverage[0]);
				}
				if((int)(sliderDataAccumulation[1] / 10.0) > sliderDataAverage[1] + 1 ||
					 (int)(sliderDataAccumulation[1] / 10.0) < sliderDataAverage[1] - 1){
						sliderDataAverage[1] = (int)(sliderDataAccumulation[1] / 10.0);
						printf("slider 2:%d\n", sliderDataAverage[1]);
				}
				adcScanCount = 0;
				sliderDataAccumulation[0] = 0;
				sliderDataAccumulation[1] = 0;
		}
    
}

void setI2c(){
	
		uint8_t i, j;
		 
    I2C0_Init();

     
    ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON = ((((I2C_T *) ((( uint32_t)0x40000000) + 0x20000)))->I2CON & ~0x3c) | (0x0CUL));
		
		printf("+-------------------------------------------------------+\n");
    printf("|  M05xx I2C Driver Sample Code(Slave) for access Slave |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(P3.4), I2C0_SCL(P3.5)\n");
		
		 
    s_I2C0HandlerFn = I2C_SlaveTRx;
		
		for(i = 0; i < 16; i++){
			for(j = 0; j < 16; j++){
				i2cWriteData[i][j] = 0;
				i2cReadData[i][j] = 0;
			}
		}
}

void setTimer(){
		printf("+-------------------------------------------------+\n");
    printf("|    Timer0 Power-down and Wake-up Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer Settings:\n");
    printf("  Timer0: Clock source is LIRC(10 kHz); Toggle-output mode and frequency is 2 Hz; Enable interrupt and wake-up.\n");
    printf("# System will enter to Power-down mode while Timer0 interrupt count is reaching 3;\n");
    printf("  And be waken-up while Timer0 interrupt count is reaching 4.\n\n");
	
		 
		((TIMER_T *) ((( uint32_t)0x40000000) + 0x10000))->TCMPR = (10000UL) / 1000UL;	
		((TIMER_T *) ((( uint32_t)0x40000000) + 0x10000))->TCSR = (2UL << 27) | (1ul << 29) | (1ul << 23);
		((TIMER_T *) ((( uint32_t)0x40000000) + 0x10000))->TCSR = (1ul << 29) | (1UL << 27);
	
		 
    NVIC_EnableIRQ(TMR0_IRQn);

     
    TIMER_Start(((TIMER_T *) ((( uint32_t)0x40000000) + 0x10000)));
}

void SetPin(uint8_t port, uint8_t pin, uint8_t value){
	(*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(port))) + ((pin)<<2)))) = value;
}

uint32_t GetPin(uint8_t port, uint8_t pin){
	return (*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(port))) + ((pin)<<2))));
}

void ReadPanel(){
	
		
		





 
	
		uint8_t i, j, k;
	
		uint8_t pressedNum = 0;
		
		char tempCommand[16];
		char value[4];
			
		for(i = 0; i < 16; i++){
			if(i & 0x8)
				SetPin(3, 7, 1);
			else
				SetPin(3, 7, 0);
			
			if(i & 0x4)
				SetPin(3, 6, 1);
			else
				SetPin(3, 6, 0);
			
			if(i & 0x2)
				SetPin(3, 3, 1);
			else
				SetPin(3, 3, 0);
			
			if(i & 0x1)
				SetPin(3, 2, 1);
			else
				SetPin(3, 2, 0);
			
			for(j = 0; j < 8; j++){
				if(i == 8 || i ==10 || i == 12){	
					
						pressedNum = (i-8)/2*8+j;
						if(GetPin(0, j) == 0){
								if(pressedKey[pressedNum] == 0){
										pressedKeyTime[pressedNum] = timerCount;
										pressedKey[pressedNum] = 1;
										
										memset(tempCommand, 0x0, 16);
										sprintf(value, "%03d", 127 + 23 - pressedNum);
										strncpy(tempCommand, value, 3);
										
										strncat(tempCommand, ",", 1);
										
										sprintf(value, "%03d", rand()%128);
										strncat(tempCommand, value, 3);
										
										
										
										
										
										memcpy(i2cWriteData[i2cWriteDataEndPos], tempCommand, 16);
										i2cWriteDataEndPos++;
										if(i2cWriteDataEndPos == 16)
											i2cWriteDataEndPos = 0;
										
										
										
									
								}
						}
						else if(pressedKey[pressedNum] == 2 || 
										pressedKey[pressedNum] == 3){  
								printf("release [%d] - %d %d at %ds\n", pressedNum, i, j, timerCount);
								pressedKey[pressedNum] = 0;
						}
						else if(pressedKey[pressedNum] == 1){	
								pressedKeyTime[pressedNum] = 0;
								pressedKey[pressedNum] = 0;
						}
						
				}
				else if(i == 9 || i ==11 || i == 13){	
						if(GetPin(0, j) == 0){
								pressedNum = (i-8)/2*8+j;
								if(pressedKey[pressedNum] == 1){
										
										printf("press [%d] %d %d with speed %ds\n", pressedNum, i, j, timerCount - pressedKeyTime[pressedNum]);
										pressedKey[pressedNum] = 2;
									
								}
								
						}
				}
				else{
					
					if(GetPin(0, j) == 0){
							if(i ==14 && (j == 4 || j == 5)){
									switch(sectionKnobState){
									case 0:
											if(GetPin(0, 4) == 0 && GetPin(0, 5) != 0){
													sectionKnobState = 1;
													printf("section knob forward.\n");
											}
											else if(GetPin(0, 5) == 0 && GetPin(0, 4) != 0){
													sectionKnobState = 2;
													printf("section knob backward.\n");
											}
											break;
									case 1:
									case 2:
											if(GetPin(0, 4) == 0 && GetPin(0, 5) == 0){
													sectionKnobState = 0;
													printf("section knob floating.\n");
											}
											break;
									}
							}
							else{
									if(i == 14){
											switch(j){
											case 1:
													if(sensitiveButton != 1){
															sensitiveButton = 1;
															printf("Press Sensitive button.\n");
													}
													break;
											case 2:
													if(sustainButton != 1){
															sustainButton = 1;
															printf("Press Sustain button.\n");
													}
													break;
											case 3:
													if(speedButton != 1){
															speedButton = 1;
															printf("Press Speed button.\n");
													}
													break;
													
												
											}
										
									}
									else if(i ==15){
											switch(j){
											case 0:
													if(pauseButton != 1){
															pauseButton = 1;
															printf("Press Pause button.\n");
													}
													break;
											case 1:
													if(raiseOctaceButton != 1){
															raiseOctaceButton = 1;
															printf("Press Raise octave button.\n");
													}
													break;
											case 2:
													if(lowerOctaceButton != 1){
															lowerOctaceButton = 1;
															printf("Press Lower octave button.\n");
													}
													break;
											case 3:
													if(pedalDown != 1){
															pedalDown = 1;
															printf("Pedal Down.\n");
													}
													break;
											case 4:
													printf("Speed knob forward.\n");
													break;
											case 5:
													printf("Speed knob backward.\n");
													break;
													
												
											}
									}
								
								
									
							}
							
							
							
							
							
							
							
							
							
							
							
					}
					else{
							if(i == 14){
									switch(j){
									case 1:
											if(sensitiveButton == 1){
													sensitiveButton = 0;
													printf("Release Sensitive button.\n");
											}
											break;
									case 2:
											if(sustainButton == 1){
													sustainButton = 0;
													printf("Release Sustain button.\n");
											}
											break;
									case 3:
											if(speedButton == 1){
													speedButton = 0;
													printf("Release Speed button.\n");
											}
											break;
									}
							}
							else if(i ==15){
									switch(j){
									case 0:
											if(pauseButton == 1){
													pauseButton = 0;
													printf("Release Pause button.\n");
											}
											break;
									case 1:
											if(raiseOctaceButton == 1){
													raiseOctaceButton = 0;
													printf("Release Raise octave button.\n");
											}
											break;
									case 2:
											if(lowerOctaceButton == 1){
													lowerOctaceButton = 0;
													printf("Release Lower octave button.\n");
											}
											break;
									case 3:
											if(pedalDown == 1){
													pedalDown = 0;
													printf("Pedal Up.\n");
											}
											break;
									}
							}
					}
				}
			}
		}
		
		
		
}

void SetIndicatorLights(int lightNumber, int on){
		
		
		
		
		
		
		
		
		
		
		
	
		uint8_t i, j;
		
		if(on == 1)
			indicatorLights[lightNumber] = 1;
		else if(on == 0)
			indicatorLights[lightNumber] = 0;
	
	
		for(i = 0; i < 16; i++){
				(*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(1))) + ((4)<<2)))) = 0;
				for(j = 15; j < 16; j--){
						(*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(1))) + ((7)<<2)))) = 0;
						
						if(indicatorLights[j] == 1)
							(*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(1))) + ((5)<<2)))) = 1;
						else
							(*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(1))) + ((5)<<2)))) = 0;
						
						(*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(1))) + ((7)<<2)))) = 1;
				}
				(*((volatile uint32_t *)(((((( uint32_t)0x50000000) + 0x4000) + 0x0200)+(0x20*(1))) + ((4)<<2)))) = 1;
			
		}
}



 
void SetPwmLightRing(int position){
	
		
		
		
		
		
		
		uint16_t brightnessArray[8] = {0};
		uint16_t i;
		
		for(i = 0;i < 8; i++){
				if(i > position)
					brightnessArray[i] = 0x0;
				else if(i == position)
					brightnessArray[i] = 0xFF0;
				else
					brightnessArray[i] = 0x100;
		}
		
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR0 = brightnessArray[7];
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR1 = brightnessArray[6];
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR2 = brightnessArray[5];
		((PWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CMR3 = brightnessArray[4];
								
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR0 = brightnessArray[3];
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR1 = brightnessArray[2];
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR2 = brightnessArray[1];
		((PWM_T *) ((( uint32_t)0x40100000) + 0x40000))->CMR3 = brightnessArray[0];
}

 
 
 
int main(void)
{
    int32_t i32Err;
		int count = 0;
		int lightPos = 0;
		

     
    SYS_UnlockReg();

     
    
		SYS_Init();
		
     
    SYS_LockReg();

     
    UART0_Init();
	










 

	
	
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|        All Panel Function Sample Code           |\n");
    printf("+-------------------------------------------------+\n\n");
		
		setPwm();
		
		setGpio();
		
		setI2c();

    setTimer();
		
		
		 
		
		while(1){
			
			ReadPanel();
			
			if(timerCount % 10 == 0)
				AdcContScanModeTest();
			
			
				
			
			
			if(count % 1000 == 50){
					SetPwmLightRing(lightPos / 2);
					SetIndicatorLights(lightPos, 1);
					SetIndicatorLights(lightPos - 1 < 0 ? 15 : lightPos - 1, 0);
					if(++lightPos == 16)
						lightPos = 0;
			}
			
			if(++count > 10000)
				count = 0;
			
			
			
			
		}
			
		
		
		

    
}

 
