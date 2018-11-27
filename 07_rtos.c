// RTOS Framework - Fall 2018
// J Losh

// Student Name: Manoj Mysore Veere Gowda.
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// 07_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART


// Device includes, defines, and assembler directives
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define PB_0        (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PB_1        (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB_2        (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB_3        (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PB_4        (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))
#define YELLOW_LED  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define GREEN_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define ORANGE_LED  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

#define MAX_CHARS      90

char str[MAX_CHARS+1],string_type[100],strref[MAX_CHARS+1];
char* strg;

static uint32_t  First_Parameter;
static uint32_t* Second_Parameter;
static uint32_t  Third_Parameter;

bool      Priority_flag=1,Default_flag=0,Flag_First=1,temp_flag=1,Preemption_Off=1,PIOFF_FLAG=0;                                       //Flags used
uint8_t   string_i=0,string_look,string_a,string_index,string_field,string_position[30],SVC_Case_Number,string_count=0;
uint32_t  rt_1,rt_2,rt_4,rt_8,rt_16,rt_t,Sys_stack,CurrentTask_v,Task_timer,numb[3],systick_timmer=0,percent_idle;
uint64_t  Total_time;

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 15=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  uint8_t variable;              // variable to keep track of priority
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
  uint64_t Timer;                // stores the total time that the process has run
  uint64_t Percent;              // CPU percent for 1sec.
  uint64_t CPU_Percent;          // CUP percent after IIR filter.
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

//-----------------------------------------------------------------------------
// Defined functions
//-----------------------------------------------------------------------------
void setsp(uint32_t x)                                //function used to set the Stack Pointer value
{
    __asm (" ADD SP, #8");
    __asm (" MOV SP, R0");
    __asm (" SUB SP, #8");
}

int getsp()                                          //function used to get the Stack Pointer value
{
    __asm (" MOV R0, SP");
}

int getSVCN()                                        //function used to get the case number in svcisr
{

}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
  TIMER1_TAV_R=0;
  // REQUIRED: initialize systick for 1ms system timer
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
    if(Default_flag==true||Priority_flag==true)                                            //It is is to set the Priority as default.
    {
        if(tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
        {
            if(tcb[task].variable == tcb[task].currentPriority)
            {
              ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
              tcb[task].variable = 0;
             }
            else
              tcb[task].variable++;
        }
    }
    if(Default_flag==false)                                                               // It is to run all the un-run tasks first Before setting the priority.
    {
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        if(task==9)
          Default_flag=true;
    }
    if(Priority_flag==false)                                                              // It is to set the Priority off.
    {
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
  }
  return task;

}

void rtosStart()
{
    // Configure systicIsr                                                                 // Triggering Systic.
    NVIC_ST_CTRL_R= 0x00000007;
    NVIC_ST_RELOAD_R= 40000;
    __asm(" ADD SP,#8");
    taskCurrent = rtosScheduler();
    Sys_stack=getsp();
    setsp(tcb[taskCurrent].sp);
    _fn fn;
    fn=(_fn)tcb[taskCurrent].pid;
    (*fn)();
  // Add code to initialize the SP with tcb[task_current].sp;
}

bool createThread(_fn fn, char name[], int priority)
{
    __asm (" SVC #05");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm (" SVC #06");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i;
    for(i=0;i<9;i++)
    {
        if(tcb[i].pid==fn)
        {
            tcb[i].priority= priority;
            tcb[i].currentPriority= priority;
        }
    }
}

struct semaphore* createSemaphore(uint8_t count)
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
  }
  return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
  // push registers, call scheduler, pop registers, return to new function
    __asm (" SVC #01");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm (" SVC #02");
  // push registers, set state to delayed, store timeout, call scheduler, pop registers,
  // return to new function (separate unrun or ready processing)
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm (" SVC #03");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm (" SVC #04");
}

void Timer1Isr()
{

}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i=0;
    while(i<MAX_TASKS)
    {
        if(tcb[i].state==STATE_DELAYED)
        {
            tcb[i].ticks--;
            if(tcb[i].ticks==0)
              tcb[i].state=STATE_READY;
        }
        i++;
    }
    if(systick_timmer==100)                                                      //Calculation of CPU% for every 1sec using IIR filters
    {
        systick_timmer = 0;
        for(i=0;i<9;i++)
        {
            Total_time=Total_time+tcb[i].Timer;
        }
        for(i=0;i<9;i++)
        {
            tcb[i].Percent=(10000*tcb[i].Timer)/Total_time;
            tcb[i].CPU_Percent= ((99*tcb[i].CPU_Percent)+(tcb[i].Percent))/100;
            tcb[i].Timer=0;
        }
        Total_time =0;
    }
    systick_timmer++;
    if(Preemption_Off==1)
     NVIC_INT_CTRL_R = 0x10000000;
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{//pendSvIsr is not pushing anything.
   __asm(" PUSH {R4-R11}");                                                 //Manualy pushing the registers
   tcb[taskCurrent].sp= getsp();
   setsp(Sys_stack);
   if(Flag_First==0)
   {
       tcb[taskCurrent].Timer=tcb[taskCurrent].Timer+TIMER1_TAV_R;
   }
   Flag_First = 0;
   taskCurrent = rtosScheduler();
   TIMER1_TAV_R=0;
   setsp(tcb[taskCurrent].sp);

   if (tcb[taskCurrent].state==STATE_UNRUN)                                //Pushing the register to look like there was already a process run if its unrun
   {
       CurrentTask_v= 0x41000200;
       __asm(" PUSH {R0}");
       CurrentTask_v=tcb[taskCurrent].pid;
       __asm(" PUSH {R0} ");
       __asm(" SUB SP,#64");
       tcb[taskCurrent].state = STATE_READY;
   }

   if (tcb[taskCurrent].state == STATE_READY)                             //POPING back all the regiters
   {
       __asm(" POP {R4-R11}");
       __asm(" POP {R3,LR}");
   }

   __asm(" MOV R0, #0xFFFF");
   __asm(" LSL R0, #0x10");
   __asm(" MOV R1, #0xFFF9");
   __asm(" ORR R0, R1");

   __asm(" MOV LR, R0");
   __asm(" BX LR");
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
       __asm(" MOV R4, #0x2000");                                                     //Getting the First parameter value from R0 and writing it in a variable memory
       __asm(" LSL R4, #0x10");
       __asm(" MOV R5, #0x2C98");
       __asm(" ORR R4, R5");
       __asm(" STR R0, [R4] ");

       __asm(" MOV R4, #0x2000");                                                    //Getting the Second parameter value from R1 and writing it in a variable memory
       __asm(" LSL R4, #0x10");
       __asm(" MOV R5, #0x2C9C");
       __asm(" ORR R4, R5");
       __asm(" STR R1, [R4] ");

       __asm(" MOV R4, #0x2000");                                                    //Getting the Third parameter value from R2 and writing it in a variable memory
       __asm(" LSL R4, #0x10");
       __asm(" MOV R5, #0x2CA0");
       __asm(" ORR R4, R5");
       __asm(" STR R2, [R4] ");

       __asm (" MOV R0, SP");                                                       //Setting the SP value to the right position after yield
       __asm (" ADD R0, #56");
       __asm (" LDR R0, [R0]");
       __asm (" SUB R0, #2");
       __asm (" LDR R0, [R0]");

     SVC_Case_Number= getSVCN();                                                    //Getting the Case Number
     switch (SVC_Case_Number)
     {
         case 1:                                                                        //yield
         {
             tcb[taskCurrent].state=STATE_READY;
             NVIC_INT_CTRL_R = 0x10000000 ;
             break;
         }
         case 2:                                                                        //sleep
         {
             tcb[taskCurrent].ticks= First_Parameter;
             tcb[taskCurrent].state=STATE_DELAYED;
             NVIC_INT_CTRL_R = 0x10000000;
             break;
         }
         case 3:                                                                       //wait
         {
             uint8_t i;
             struct semaphore *pSema;
             pSema=(struct semaphore*) First_Parameter;
             if(pSema->count==0)
             {
                 if(pSema->queueSize<MAX_QUEUE_SIZE)
                 {
                     pSema->processQueue[pSema->queueSize]=tcb[taskCurrent].pid;
                     tcb[taskCurrent].semaphore = pSema;
                 }
                 pSema->queueSize++;
                 tcb[taskCurrent].state=STATE_BLOCKED;
             }
             if(pSema->count>0)
                 pSema->count--;
             if(PIOFF_FLAG==0)
             {
                for(i=0;i<MAX_TASKS;i++)
                 {
                     if(tcb[taskCurrent].semaphore==tcb[i].semaphore)
                     {
                         if(tcb[taskCurrent].currentPriority<tcb[i].currentPriority)
                            tcb[i].currentPriority=tcb[taskCurrent].currentPriority;
                     }

                 }
             }
             NVIC_INT_CTRL_R = 0x10000000;
             break;
         }
         case 4:                                                                       //post
         {
              uint8_t i;
              struct semaphore *pSema;
              pSema=(struct semaphore*) First_Parameter;
              uint8_t g;
              pSema->count++;
              if(pSema->queueSize>0)
              {
                  pSema->count--;
                  pSema->queueSize--;
                  for(g=0;g<MAX_TASKS;g++)
                  {
                      if(tcb[g].state!=STATE_INVALID)
                      {
                          if(tcb[g].pid==pSema->processQueue[0])
                          {

                              tcb[g].state=STATE_READY;
                              pSema->processQueue[0]=0;
                          }
                          if(PIOFF_FLAG==0)
                          {
                              for(i=0;i<MAX_TASKS;i++)
                              {
                                  if(tcb[taskCurrent].semaphore==tcb[i].semaphore)
                                  {
                                      tcb[i].currentPriority= tcb[i].priority;
                                  }
                              }
                          }
                      }
                  }
              }

              NVIC_INT_CTRL_R = 0x10000000;
              break;
         }
         case 5:                                                                      //create Thread
         {
               _fn temp_fn;
               char temp_name[15];
               int temp_priority;
               temp_fn=(_fn) First_Parameter;
               temp_priority=(int) Third_Parameter;
               bool ok = false;
               uint8_t i = 0;
               bool found = false;
               // REQUIRED: store the thread name
               // add task if room in task list
               if (taskCount < MAX_TASKS)
               {
                 // make sure fn not already in list (prevent reentrancy)
                 while (!found && (i < MAX_TASKS))
                 {
                   found = (tcb[i++].pid ==  temp_fn);
                 }
                 if (!found)
                 {
                   // find first available tcb record
                   i = 0;
                   while (tcb[i].state != STATE_INVALID) {i++;}
                   tcb[i].state = STATE_UNRUN;
                   tcb[i].pid = temp_fn;
                   tcb[i].sp = &stack[i][255];
                   tcb[i].priority = temp_priority;
                   tcb[i].currentPriority = temp_priority;
                   tcb[i].variable = 0;
                   strcpy(tcb[i].name,Second_Parameter);
                   // increment task count
                   taskCount++;
                   ok = true;
                 }
               }
               // REQUIRED: allow tasks switches again
               return ok;
               break;
         }
         case 6:                                                                           //Destroy Thread
         {
             _fn fn_temp;
             struct semaphore *semap_temp = &semaphores[0];
             fn_temp=(_fn) First_Parameter;
             uint8_t i,j,k;
             for(i=0;i<9;i++)
             {
                 if(tcb[i].pid==(void*)fn_temp)
                 {
                      tcb[i].state=STATE_INVALID;
                      tcb[i].pid=0;
                      tcb[i].ticks=0;
                      taskCount--;
                      putsUart0("Process Killed!\r");
                      putsUart0("\r\n");
                      tcb[i].Percent=0;
                      tcb[i].CPU_Percent=0;
                      tcb[i].Timer=0;
                 }
             }
             for(i=0;i<4;i++)
             {
                 semap_temp = &semaphores[i];
                     if(semap_temp->processQueue[0]==(uint32_t)fn_temp)
                     {
                         semap_temp->processQueue[0]=0;
                         semap_temp->queueSize--;
                     }
             }
             break;
         }

     }
}



//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (nssot needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and B peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF;

    // Configure LED and pushbutton pins
    GPIO_PORTA_DIR_R = 0xE0;  // bits 7,6 and 5 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R = 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0xFC;  // enable LEDs and pushbuttons
    GPIO_PORTA_PUR_R = 0x1C;  // PULL UP pushbuttons

    // Configure LED and pushbutton pins
    GPIO_PORTB_DIR_R = 0x10;  // bits 4 outputs, other pins are inputs
    GPIO_PORTB_DR2R_R = 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R = 0xD0;  // enable LEDs and pushbuttons
    GPIO_PORTB_PUR_R = 0xC0;  // PULL UP pushbuttons

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x04;  // bits 4 outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x04;  // enable LEDs and pushbuttons

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
   UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
   UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
   UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
   UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
   UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
   UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

  // Configure Timer 1 as the time base
   SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
   TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
   TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
   TIMER1_TAMR_R = 0x00000012;                      // configure for periodic mode (count down)
   TIMER1_TAILR_R = 0x9C40;                         // set load value to 40e3 for 1 Hz interrupt rate
   TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
  rt_1=0;rt_2=0;rt_4=0;rt_8=0;rt_16=0;
  if(PB_0==0)
      rt_1=1;
  if(PB_1==0)
      rt_2=2;
  if(PB_2==0)
      rt_4=4;
  if(PB_3==0)
      rt_8=8;
  if(PB_4==0)
    rt_16=16;
  rt_t=rt_1+rt_2+rt_4+rt_8+rt_16;
  return rt_t;
}

// Blocking function that returns with serial data once the buffer is not emptychar getcUart0()
char getcUart0()
{
    while(UART0_FR_R & UART_FR_RXFE)
        yield();
    return UART0_DR_R & 0xFF;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

void putnUart0(uint32_t a)                                                  //function to write integer
{
    uint8_t i;
    char xn[4];
    for(i=3;i>0;i--)
    {
       numb[i]=a%10;
       a=a/10;
    }
   numb[0]=a%10;
   a=a/10;
   for(i=0;i<4;i++)
   {
        if(numb[i]==0)
         xn[i]='0';
        if(numb[i]==1)
         xn[i]='1';
        if(numb[i]==2)
         xn[i]='2';
        if(numb[i]==3)
         xn[i]='3';
        if(numb[i]==4)
         xn[i]='4';
        if(numb[i]==5)
         xn[i]='5';
        if(numb[i]==6)
         xn[i]='6';
        if(numb[i]==7)
         xn[i]='7';
        if(numb[i]==8)
         xn[i]='8';
        if(numb[i]==9)
         xn[i]='9';
   }
    xn[4]=0;
    putsUart0(xn);
}

void putdnUart0(uint32_t a)                                                       //function to write floating integer
{
   uint8_t i;
   uint32_t dnumb[5];
   char dxn[5];
   dnumb[4]=a%10;
    a=a/10;
   dnumb[3]=a%10;
    a=a/10;
   dnumb[1]=a%10;
    a=a/10;
   dnumb[0]=a%10;
    a=a/10;
   for(i=0;i<5;i++)
   {
        if(dnumb[i]==0)
         dxn[i]='0';
        if(dnumb[i]==1)
         dxn[i]='1';
        if(dnumb[i]==2)
         dxn[i]='2';
        if(dnumb[i]==3)
         dxn[i]='3';
        if(dnumb[i]==4)
         dxn[i]='4';
        if(dnumb[i]==5)
         dxn[i]='5';
        if(dnumb[i]==6)
         dxn[i]='6';
        if(dnumb[i]==7)
         dxn[i]='7';
        if(dnumb[i]==8)
         dxn[i]='8';
        if(dnumb[i]==9)
         dxn[i]='9';
   }
   dxn[2]='.';
   dxn[5]=0;
   putsUart0(dxn);
}

int getnumber(uint16_t field)                                                            //Function to get number from is_command.
{
    if (string_type[field]=='n')
    {
        uint16_t c;
        c = atoi(&str[string_position[field]]);
        return c;
    }
    else
    {
        return 0;
    }
}

int iscommand(char strp[],int number)
{

    if ( (strcmp(strp, &str[string_position[0]])==0) && string_field>=number+1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

//Function to get string from is_command.
char* getstring(uint8_t field)
{
if (string_type[field]=='a')
{
    return &str[string_position[field]];
}
else
{
    return 0;
}
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}

void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}

void oneshot()
{
  while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash4Hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
    }
    if ((buttons & 16) != 0)
    {
      setThreadPriority(lengthyFn, 4);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}

void important()
{
    while(true)
    {
      wait(resource);
      BLUE_LED = 1;
      sleep(1000);
      BLUE_LED = 0;
      post(resource);
    }
}

void shell()
{
  putsUart0("\rREADY\r\n");
  while (true)
  {
      for(string_i=0; string_i<MAX_CHARS; string_i++)
          {
              str[string_i]=0;
          }
      for(string_count=0;string_count<(MAX_CHARS);string_count++)
      {
          str[string_count]=getcUart0();
          if(str[string_count]==8)
          {
              if(string_count>0)
                  string_count=string_count-2;
              if(string_count==0)
                  string_count=0;
          }
          else if(str[string_count]==32)
          {
              string_count=string_count++;
          }
          else if(str[string_count]<32 && str[string_count]!=8 && str[string_count]!=13)
          {
              putsUart0("Enter a valid character\r\n");
          }
          else if(str[string_count]==13)
          {
              str[string_count]=0x00;
              putsUart0(str);
              putsUart0("\r\n");
              break;
          }
          if(string_count>85)
           {
             putsUart0("\r\n");
             break;
           }
      }
      string_index=0;
      string_a=0;
      string_look = 1;
      string_field = 0;
      for(string_i=0;string_i<85;string_i++)
      {
          if(!((str[string_i]>=48 && str[string_i]<=57)||(str[string_i]>=65 && str[string_i]<=90)||(str[string_i]>=97 && str[string_i]<=122)||(str[string_i]==38)))
          {
              string_look=1;
          }
          if(((str[string_i]>=48 && str[string_i]<=57)||(str[string_i]>=65 && str[string_i]<=90)||(str[string_i]>=97 && str[string_i]<=122)||(str[string_i]==38)) && string_look==1)
          {
              string_position[string_index] = string_i;
              if((str[string_i]>=65 && str[string_i]<=90)||(str[string_i]>=97 && str[string_i]<=122)||(str[string_i]==38))
              {

                  string_type[string_index] = 'a';
                  string_index++;

              }
              if((str[string_i]>=48 && str[string_i]<=57))
              {
                  string_type[string_index] = 'n';
                  string_index++;
              }
              string_look= 0;
              string_field++;
          }

      }

      for(string_i=0;string_i<85;string_i++)
      {
          if(((str[string_i]>=48 && str[string_i]<=57)||(str[string_i]>=65 && str[string_i]<=90)||(str[string_i]>=97 && str[string_i]<=122)||(str[string_i]==38)))
          {
              strref[string_a]=tolower(str[string_i]);
              string_a++;
          }
          else
          {
              str[string_i] = '\0';
              strref[string_a]=str[string_i];
              string_a++;
          }
          str[string_i]=tolower(str[string_i]);
      }

      if (iscommand("priority",1))                                                             //command for priority
      {
          strg=getstring(1);
          if(strcmp(strg,"on")==0)
          {
                 Priority_flag=1;
                 putsUart0("Priority is set to on\r\n");
          }
          if(strcmp(strg,"off")==0)
          {
                 Priority_flag=0;
                 putsUart0("Priority is set to off\r\n");
          }
      }

      if (iscommand("preempt",1))                                                            //command for preempt
      {
          strg=getstring(1);
          if(strcmp(strg,"on")==0)
          {
              Preemption_Off=1;
              putsUart0("Preemption is on\r\n");
          }
          if(strcmp(strg,"off")==0)
          {
              Preemption_Off=0;
              putsUart0("Preemption is off\r\n");
          }
      }

      if (iscommand("pi",1))                                                                //command for priority inheritance
        {
            strg=getstring(1);
            if(strcmp(strg,"on")==0)
            {
                PIOFF_FLAG=0;
                putsUart0("Priority Inheritance is on\r\n");
            }
            if(strcmp(strg,"off")==0)
            {
                PIOFF_FLAG=1;
                putsUart0("Priority Inheritance is off\r\n");
            }
        }

      if (iscommand("reboot",0))                                                          //command to reset
      {
          strg=getstring(0);
          if(strcmp(strg,"reboot")==0)
          NVIC_APINT_R= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
      }

      if (iscommand("pidof",1))                                                          //command to get the pid of process
      {
          strg=getstring(1);
          if(strcmp(strg,"idle")==0)
              putnUart0((uint16_t)tcb[0].pid);
          if(strcmp(strg,"lengthyfn")==0)
              putnUart0((uint16_t)tcb[1].pid);
          if(strcmp(strg,"flash4hz")==0)
              putnUart0((uint16_t)tcb[2].pid);
          if(strcmp(strg,"oneshot")==0)
              putnUart0((uint16_t)tcb[3].pid);
          if(strcmp(strg,"readkeys")==0)
              putnUart0((uint16_t)tcb[4].pid);
          if(strcmp(strg,"debounce")==0)
              putnUart0((uint16_t)tcb[5].pid);
          if(strcmp(strg,"important")==0)
              putnUart0((uint16_t)tcb[6].pid);
          if(strcmp(strg,"uncoop")==0)
              putnUart0((uint16_t)tcb[7].pid);
          if(strcmp(strg,"shell")==0)
              putnUart0((uint16_t)tcb[8].pid);
          putsUart0("\r\n");
       }

      if (iscommand("kill",1))                                                         //command to kill the process
      {
          uint16_t Kill_PID;
          Kill_PID=getnumber(1);
          destroyThread(Kill_PID);
      }

      if(iscommand("flags",0))                                                        //command to see the flags
      {
          if(Priority_flag==1)
              putsUart0("PRIORITY   - on\r\n");
          if(Priority_flag==0)
              putsUart0("PRIORITY   - off\r\n");
          if(Preemption_Off==1)
              putsUart0("PREEMPTION - on\r\n");
          if(Preemption_Off==0)
              putsUart0("PPREEMPTION - off\r\n");
          if(PIOFF_FLAG==0)
              putsUart0("PI         - on\r\n");
          if(PIOFF_FLAG==1)
              putsUart0("PI         - off\r\n");
      }

      if(iscommand("ipcs",0))                                                       //command for ipcs
      {
          uint8_t i;
          struct semaphore *sem_temp;
          putsUart0("Semaphore\t  Count\t        Process\t        Queuesize\r\n");
          putsUart0("-------------------------------------------------------\r\n");
          putsUart0("keyPressed\t  ");
          sem_temp = &semaphores[0];
          putnUart0(sem_temp->count);
          putsUart0("\t          ");
          for(i=0;i<5;i++)
          {
              if(sem_temp->processQueue[i]==0)
                  break;
              putnUart0(sem_temp->processQueue[i]);
              putsUart0(" ");
          }
          putsUart0("\t          ");
          putnUart0(sem_temp->queueSize);
          putsUart0("\r\n");
          putsUart0("keyReleased\t  ");
          sem_temp = &semaphores[1];
          putnUart0(sem_temp->count);
          putsUart0("\t          ");
          for(i=0;i<5;i++)
          {
              if(sem_temp->processQueue[i]==0)
                break;
              putnUart0(sem_temp->processQueue[i]);
              putsUart0(" ");
          }
          putsUart0("\t          ");
          putnUart0(sem_temp->queueSize);
          putsUart0("\r\n");
          putsUart0("flashReq\t  ");
          sem_temp = &semaphores[2];
          putnUart0(sem_temp->count);
          putsUart0("\t          ");
          for(i=0;i<5;i++)
          {
              if(sem_temp->processQueue[i]==0)
              break;
              putnUart0(sem_temp->processQueue[i]);
              putsUart0(" ");
          }
          putsUart0("\t          ");
          putnUart0(sem_temp->queueSize);
          putsUart0("\r\n");
          putsUart0("resource\t  ");
          sem_temp = &semaphores[3];
          putnUart0(sem_temp->count);
          putsUart0("\t          ");
          for(i=0;i<5;i++)
          {
              if(sem_temp->processQueue[i]==0)
              break;
              putnUart0(sem_temp->processQueue[i]);
              putsUart0(" ");
          }
          putsUart0("\t          ");
          putnUart0(sem_temp->queueSize);
          putsUart0("\r\n-------------------------------------------------------\r\n");
      }

      if(iscommand("ps",0))                                                                      //command to see process status
      {
          uint8_t i;
          struct semaphore *sem_temp;
          putsUart0("PID\tTHREAD NAME\tSTATE\t\t BLOCKED BY     CPU%\r\n");
          putsUart0("-------------------------------------------------------------\r\n");
          for(i=0;i<9;i++)
          {
              putnUart0(tcb[i].pid);
              putsUart0("\t");
              putsUart0(tcb[i].name);
              if(i==0||i==3||i==7||i==8)
                  putsUart0("\t");
                  putsUart0("\t");
              if(tcb[i].state==0)
                  putsUart0("STATE_INVALID\t\t\t");
              if(tcb[i].state==1)
                  putsUart0("STATE_UNRUN\t\t\t");
              if(tcb[i].state==2)
                  putsUart0("STATE_READY\t\t\t");
              if(tcb[i].state==3)
              {
                 sem_temp = &semaphores[0];
                 if(sem_temp->processQueue[0]==tcb[i].pid)
                 {
                     putsUart0("STATE_BLOCKED "); putsUart0("   KEYPRESSED\t");
                 }
                 sem_temp = &semaphores[1];
                 if(sem_temp->processQueue[0]==tcb[i].pid)
                 {
                     putsUart0("STATE_BLOCKED "); putsUart0("   KEYRELEASED\t");
                 }
                 sem_temp = &semaphores[2];
                  if(sem_temp->processQueue[0]==tcb[i].pid)
                  {
                     putsUart0("STATE_BLOCKED "); putsUart0("   FLASHREQ\t");
                  }
                 sem_temp = &semaphores[3];
                  if(sem_temp->processQueue[0]==tcb[i].pid)
                  {
                     putsUart0("STATE_BLOCKED "); putsUart0("   RESOURCE\t");
                  }
              }
              if(tcb[i].state==4)
                  putsUart0("STATE_DELAYED\t\t\t");
              putdnUart0(tcb[i].CPU_Percent);
              putsUart0("\r\n");
          }
          putsUart0("-------------------------------------------------------------\r\n");
      }

      if (strcmp(strref,"idle&")==0)                                                           //commands to create process
      {
          createThread(idle, "Idle", 15);
          putsUart0("Idle process is created\r\n");
      }
      if (strcmp(strref,"lengthyfn&")==0)
      {
          createThread(lengthyFn, "LengthyFn", 12);
          putsUart0("LengthyFn process is created\r\n");
      }
      if (strcmp(strref,"flash4hz&")==0)
      {
            createThread(flash4Hz, "Flash4Hz", 4);
            putsUart0("Flash4Hz process is created\r\n");
      }
      if (strcmp(strref,"oneshot&")==0)
      {
          createThread(oneshot, "OneShot", 4);
          putsUart0("Oneshot process is created\r\n");
      }
      if (strcmp(strref,"readkeys&")==0)
      {
          createThread(readKeys, "ReadKeys", 12);
          putsUart0("Readkeys process is created\r\n");
      }
      if (strcmp(strref,"debounce&")==0)
      {
          createThread(debounce, "Debounce", 12);
          putsUart0("Debounce process is created\r\n");
      }
      if (strcmp(strref,"important&")==0)
      {
          createThread(important, "Important", 0);
          putsUart0("Important process is created\r\n");
      }
      if (strcmp(strref,"uncoop&")==0)
      {
          createThread(uncooperative, "Uncoop", 10);
          putsUart0("Uncoop process is created\r\n");
      }
      if (strcmp(strref,"shell&")==0)
      {
          createThread(shell, "Shell", 8);
          putsUart0("Shell process is created\r\n");
      }
      putsUart0("\r");
  }
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  initHw();
  bool ok;

  // Initialize hardware

  rtosInit();

  // Power-up flash
  GREEN_LED = 1;
  waitMicrosecond(250000);
  GREEN_LED = 0;
  waitMicrosecond(250000);



  // Initialize semaphores
  keyPressed = createSemaphore(1);
  keyReleased = createSemaphore(0);
  flashReq = createSemaphore(5);
  resource = createSemaphore(1);

  // Add required idle process
  ok =  createThread(idle, "Idle", 15);

  // Add other processes
  ok &= createThread(lengthyFn, "LengthyFn", 12);
  ok &= createThread(flash4Hz, "Flash4Hz", 4);
  ok &= createThread(oneshot, "OneShot", 4);
  ok &= createThread(readKeys, "ReadKeys", 12);
  ok &= createThread(debounce, "Debounce", 12);
  ok &= createThread(important, "Important", 0);
  ok &= createThread(uncooperative, "Uncoop", 10);
  ok &= createThread(shell, "Shell", 8);

  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
}
