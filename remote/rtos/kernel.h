/***********************************************************************
  Kernel.h and Kernel.c contains the backend of the RTOS.
  It contains the underlying Kernel that process all requests coming in from OS syscalls.
  Most of Kernel's functions are not directly usable, but a few helpers are provided for the OS for convenience and for booting purposes.
  ***********************************************************************/

#ifndef KERNEL_H_
#define KERNEL_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "os.h"

//Global configurations
#define TICK_LENG 625			//The length of a tick = 10ms, using 16Mhz clock and /256 prescsaler
#define MAX_EVENT_SIG_MISS 1	//The maximum number of missed signals to record for an event. 0 = unlimited
#define LOWEST_PRIORITY 10		//The largest number to represent the lowest task priority. 0 will always be the highest priority.

//Misc macros
#define Disable_Interrupt()		asm volatile ("cli"::)
#define Enable_Interrupt()		asm volatile ("sei"::)

  
//Definitions for potential errors the RTOS may come across
typedef enum error_codes
{
	NO_ERR  = 0,
	INVALID_ARG_ERR,
	INVALID_KERNET_REQUEST_ERR,
	KERNEL_INACTIVE_ERR,
	MAX_PROCESS_ERR,
	PID_NOT_FOUND_ERR,
	SUSPEND_NONRUNNING_TASK_ERR,
	RESUME_NONSUSPENDED_TASK_ERR,
	MAX_EVENT_ERR,
	EVENT_NOT_FOUND_ERR,
	EVENT_ALREADY_OWNED_ERR,
	SIGNAL_UNOWNED_EVENT_ERR,
	MAX_MUTEX_ERR,
	MUTEX_NOT_FOUND_ERR
} ERROR_TYPE;

  
typedef enum process_states 
{ 
   DEAD = 0, 
   READY, 
   RUNNING,
   SUSPENDED,
   SLEEPING,
   WAIT_EVENT,
   WAIT_MUTEX 
} PROCESS_STATES;


typedef enum kernel_request_type 
{
   NONE = 0,
   CREATE_T,								//Create a task
   YIELD,
   TERMINATE,
   SUSPEND,
   RESUME,
   SLEEP,
   CREATE_E,							//Initialize an event object
   WAIT_E,
   SIGNAL_E,
   CREATE_M,							//Initialize a mutex object
   LOCK_M,
   UNLOCK_M	
} KERNEL_REQUEST_TYPE;


/*Process descriptor for a task*/
typedef struct ProcessDescriptor 
{
   PID pid;									//An unique process ID for this task.
   PRIORITY pri;							//The priority of this task, from 0 (highest) to 10 (lowest).
   PROCESS_STATES state;					//What's the current state of this task?
   PROCESS_STATES last_state;				//What's the PREVIOUS state of this task? Used for task suspension/resume.
   KERNEL_REQUEST_TYPE request;				//What the task want the kernel to do (when needed).
   int request_arg;							//What value is needed for the specified kernel request.
   int arg;									//Initial argument for the task (if specified).
   unsigned char *sp;						//stack pointer into the "workSpace".
   unsigned char workSpace[WORKSPACE];		//Data memory allocated to this process.
   voidfuncptr  code;						//The function to be executed when this process is running.
} PD;


//For the ease of manageability, we're making a new event data type. The old EVENT type defined in OS.h will simply serve as an identifier.
typedef struct event_type
{
	EVENT id;								//An unique identifier for this event. 0 = uninitialized
	PID owner;								//Who's currently waiting for this event this?
	unsigned int count;						//How many unhandled events has been collected?
} EVENT_TYPE;

//For the ease of manageability, we're making a new mutex data type. The old MUTEX type defined in OS.h will simply serve as an identifier.
typedef struct mutex_type
{
	MUTEX id;								//unique id for this mutex, 0 = uninitialized
	PID owner;								//the current owner of the event, 0 = free
	unsigned int count;						//mutex can be recursively locked
	PID blocked_stack [MAXTHREAD];			//stack for blocked
	PRIORITY priority_stack [MAXTHREAD];	//priority of the processes
	unsigned int order[MAXTHREAD];			//order of task came into the stack
	unsigned int num_of_process;			//number of processes waiting on the mutex
	unsigned int total_num;					//total number of process has waitted on this mutex
	PRIORITY own_pri;						//original priority of the owner
} MUTEX_TYPE;


/*Kernel functions accessible by the OS*/
void OS_Init();
void OS_Start();
void Kernel_Create_Task(voidfuncptr f, PRIORITY py, int arg);
void Kernel_Create_Event();
void Kernel_Create_Mutex();
int findPIDByFuncPtr(voidfuncptr f);
int getEventCount(EVENT e);

/*Kernel variables accessible by the OS*/
extern volatile PD* Cp;
extern volatile unsigned char *KernelSp;
extern volatile unsigned char *CurrentSp;
extern volatile unsigned int KernelActive;
extern volatile ERROR_TYPE err;
extern volatile unsigned int Last_PID;
extern volatile unsigned int Last_EventID;
extern volatile unsigned int Last_MutexID;


#endif /* KERNEL_H_ */