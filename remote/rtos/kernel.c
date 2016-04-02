#include "kernel.h"

/*Context Switching functions defined in cswitch.s*/
extern void CSwitch();
extern void Exit_Kernel();

/*System variables used by the kernel only*/
volatile static PD Process[MAXTHREAD];			//Contains the process descriptor for all tasks, regardless of their current state.
volatile static EVENT_TYPE Event[MAXEVENT];		//Contains all the event objects 
volatile static MUTEX_TYPE Mutex[MAXMUTEX];		//Contains all the mutex objects

volatile static unsigned int NextP;				//Which task in the process queue to dispatch next.
volatile static unsigned int Task_Count;		//Number of tasks created so far.
volatile static unsigned int Event_Count;		//Number of events created so far.
volatile static unsigned int Mutex_Count;		//Number of Mutexes created so far.
volatile static unsigned int Tick_Count;		//Number of timer ticks missed

/*Variables accessible by OS*/
volatile PD* Cp;		
volatile unsigned char *KernelSp;				//Pointer to the Kernel's own stack location.
volatile unsigned char *CurrentSp;				//Pointer to the stack location of the current running task. Used for saving into PD during ctxswitch.						//The process descriptor of the currently RUNNING task. CP is used to pass information from OS calls to the kernel telling it what to do.
volatile unsigned int KernelActive;				//Indicates if kernel has been initialzied by OS_Start().
volatile unsigned int Last_PID;					//Last (also highest) PID value created so far.
volatile unsigned int Last_EventID;				//Last (also highest) EVENT value created so far.
volatile unsigned int Last_MutexID;				//Last (also highest) MUTEX value created so far.
volatile ERROR_TYPE err;						//Error code for the previous kernel operation (if any)


/************************************************************************/
/*						  KERNEL-ONLY HELPERS                           */
/************************************************************************/

/*Returns the pointer of a process descriptor in the global process list, by searching for its PID*/
PD* findProcessByPID(int pid)
{
	int i;
	
	//Valid PIDs must be greater than 0.
	if(pid <=0)
	return NULL;
	
	for(i=0; i<MAXTHREAD; i++)
	{
		if (Process[i].pid == pid)
		return &(Process[i]);
	}
	
	//No process with such PID
	return NULL;
}

EVENT_TYPE* findEventByEventID(EVENT e)
{
	int i;
	
	//Ensure the request event ID is > 0
	if(e <= 0)
	{
		#ifdef OS_DEBUG
		printf("findEventByID: The specified event ID is invalid!\n");
		#endif
		err = INVALID_ARG_ERR;
		return NULL;
	}
	
	//Find the requested Event and return its pointer if found
	for(i=0; i<MAXEVENT; i++)
	{
		if(Event[i].id == e) 
			return &Event[i];
	}
	
	//Event wasn't found
	//#ifdef OS_DEBUG
	//printf("findEventByEventID: The requested event %d was not found!\n", e);
	//#endif
	err = EVENT_NOT_FOUND_ERR;
	return NULL;
}

MUTEX_TYPE* findMutexByMutexID(MUTEX m)
{
	int i;
	
	//Ensure the request mutex ID is > 0
	if(m <= 0)
	{
		#ifdef OS_DEBUG
		printf("findMutexByID: The specified mutex ID is invalid!\n");
		#endif
		err = INVALID_ARG_ERR;
		return NULL;
	}
	
	//Find the requested Mutex and return its pointer if found
	for(i=0; i<MAXMUTEX; i++)
	{
		if(Mutex[i].id == m)
		return &Mutex[i];
	}
	
	//mutex wasn't found
	//#ifdef OS_DEBUG
	//printf("findMutexByEventID: The requested mutex %d was not found!\n", m);
	//#endif
	err = MUTEX_NOT_FOUND_ERR;
	return NULL;
}

/************************************************************************/
/*				   		       OS HELPERS                               */
/************************************************************************/

/*Returns the PID associated with a function's memory address*/
int findPIDByFuncPtr(voidfuncptr f)
{
	int i;
	
	for(i=0; i<MAXTHREAD; i++)
	{
		if (Process[i].code == f)
			return Process[i].pid;
	}
	
	//No process with such PID
	return -1;
}

/*Only useful if our RTOS allows more than one missed event signals to be recorded*/
int getEventCount(EVENT e)
{
	EVENT_TYPE* e1 = findEventByEventID(e);
	
	if(e1 == NULL) 
		return 0;
		
	return e1->count;	
}

/************************************************************************/
/*                  ISR FOR HANDLING SLEEP TICKS                        */
/************************************************************************/

//Timer tick ISR
ISR(TIMER1_COMPA_vect)
{
	++Tick_Count;
}

//Processes all tasks that are currently sleeping and decrement their sleep ticks when called. Expired sleep tasks are placed back into their old state
void Kernel_Tick_Handler()
{
	int i;
	
	//No ticks has been issued yet, skipping...
	if(Tick_Count == 0)
		return;
	
	for(i=0; i<MAXTHREAD; i++)
	{
		//Process any active tasks that are sleeping
		if(Process[i].state == SLEEPING)
		{
			//If the current sleeping task's tick count expires, put it back into its READY state
			Process[i].request_arg -= Tick_Count;
			if(Process[i].request_arg <= 0)
			{
				Process[i].state = READY;
				Process[i].request_arg = 0;
			}
		}
		
		//Process any SUSPENDED tasks that were previously sleeping
		else if(Process[i].last_state == SLEEPING)
		{
			//When task_resume is called again, the task will be back into its READY state instead if its sleep ticks expired.
			Process[i].request_arg -= Tick_Count;
			if(Process[i].request_arg <= 0)
			{
				Process[i].last_state = READY;
				Process[i].request_arg = 0;
			}
		}
	}
	Tick_Count = 0;
}

/************************************************************************/
/*                   TASK RELATED KERNEL FUNCTIONS                      */
/************************************************************************/

/* Handles all low level operations for creating a new task */
void Kernel_Create_Task(voidfuncptr f, PRIORITY py, int arg)
{
	int x;
	unsigned char *sp;
	PD *p;

	#ifdef OS_DEBUG
	int counter = 0;
	#endif
	
	//Make sure the system can still have enough resources to create more tasks
	if (Task_Count == MAXTHREAD)
	{
		#ifdef OS_DEBUG
		printf("Task_Create: Failed to create task. The system is at its process threshold.\n");
		#endif
		
		err = MAX_PROCESS_ERR;
		return;
	}

	//Find a dead or empty PD slot to allocate our new task
	for (x = 0; x < MAXTHREAD; x++)
	if (Process[x].state == DEAD) break;
	
	++Task_Count;
	p = &(Process[x]);
	
	/*The code below was agglomerated from Kernel_Create_Task_At;*/
	
	//Initializing the workspace memory for the new task
	sp = (unsigned char *) &(p->workSpace[WORKSPACE-1]);
	memset(&(p->workSpace),0,WORKSPACE);

	//Store terminate at the bottom of stack to protect against stack underrun.
	*(unsigned char *)sp-- = ((unsigned int)Task_Terminate) & 0xff;
	*(unsigned char *)sp-- = (((unsigned int)Task_Terminate) >> 8) & 0xff;
	*(unsigned char *)sp-- = 0x00;

	//Place return address of function at bottom of stack
	*(unsigned char *)sp-- = ((unsigned int)f) & 0xff;
	*(unsigned char *)sp-- = (((unsigned int)f) >> 8) & 0xff;
	*(unsigned char *)sp-- = 0x00;

	//Allocate the stack with enough memory spaces to save the registers needed for ctxswitch
	#ifdef OS_DEBUG
	 //Fill stack with initial values for development debugging
	 for (counter = 0; counter < 34; counter++)
	 {
		 *(unsigned char *)sp-- = counter;
	 }
	#else
	 //Place stack pointer at top of stack
	 sp = sp - 34;
	#endif
	
	//Build the process descriptor for the new task
	p->pid = ++Last_PID;
	p->pri = py;
	p->arg = arg;
	p->request = NONE;
	p->state = READY;
	p->sp = sp;					/* stack pointer into the "workSpace" */
	p->code = f;				/* function to be executed as a task */
	
	//No errors occured
	err = NO_ERR;
}

/*TODO: Check for mutex ownership. If PID owns any mutex, ignore this request*/
static void Kernel_Suspend_Task() 
{
	//Finds the process descriptor for the specified PID
	PD* p = findProcessByPID(Cp->request_arg);
	
	//Ensure the PID specified in the PD currently exists in the global process list
	if(p == NULL)
	{
		#ifdef OS_DEBUG
			printf("Kernel_Suspend_Task: PID not found in global process list!\n");
		#endif
		err = PID_NOT_FOUND_ERR;
		return;
	}
	
	//Ensure the task is not in a unsuspendable state
	if(p->state == DEAD || p->state == SUSPENDED)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Suspend_Task: Trying to suspend a task that's in an unsuspendable state %d!\n", p->state);
		#endif
		err = SUSPEND_NONRUNNING_TASK_ERR;
		return;
	}
	
	//Ensure the task is not currently owning a mutex
	for(int i=0; i<MAXMUTEX; i++) {
		if (Mutex[i].owner == p->pid) {
			#ifdef OS_DEBUG
			printf("Kernel_Suspend_Task: Trying to suspend a task that currently owns a mutex\n");
			#endif
			err = SUSPEND_NONRUNNING_TASK_ERR;
			return;
		}
	}
	
	//Save its current state and set it to SUSPENDED
	p->last_state = p->state;
	p->state = SUSPENDED;
	err = NO_ERR;
}

static void Kernel_Resume_Task()
{
	//Finds the process descriptor for the specified PID
	PD* p = findProcessByPID(Cp->request_arg);
	
	//Ensure the PID specified in the PD currently exists in the global process list
	if(p == NULL)
	{
		#ifdef OS_DEBUG
			printf("Kernel_Resume_Task: PID not found in global process list!\n");
		#endif
		err = PID_NOT_FOUND_ERR;
		return;
	}
	
	//Ensure the task is currently in the SUSPENDED state
	if(p->state != SUSPENDED)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Resume_Task: Trying to resume a task that's not SUSPENDED!\n");
		printf("CURRENT STATE: %d\n", p->state);
		#endif
		err = RESUME_NONSUSPENDED_TASK_ERR;
		return;
	}
	
	//Restore the previous state of the task
	p->state = p->last_state;
	p->last_state = SUSPENDED;			
	err = NO_ERR;
}

/************************************************************************/
/*                  EVENT RELATED KERNEL FUNCTIONS                      */
/************************************************************************/

void Kernel_Create_Event(void)
{
	int i;
	
	//Make sure the system's events are not at max
	if(Event_Count >= MAXEVENT)
	{
		#ifdef OS_DEBUG
		printf("Event_Init: Failed to create Event. The system is at its max event threshold.\n");
		#endif
		err = MAX_EVENT_ERR;
		return;
	}
	
	//Find an uninitialized Event slot
	for(i=0; i<MAXEVENT; i++)
		if(Event[i].id == 0) break;
	
	//Assign a new unique ID to the event. Note that the smallest valid Event ID is 1.
	Event[i].id = ++Last_EventID;
	Event[i].owner = 0;
	++Event_Count;
	err = NO_ERR;
	
	#ifdef OS_DEBUG
	printf("Event_Init: Created Event %d!\n", Last_EventID);
	#endif
}

static void Kernel_Wait_Event(void)
{
	EVENT_TYPE* e = findEventByEventID(Cp->request_arg);
	
	if(e == NULL)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Wait_Event: Error finding requested event!\n");
		#endif
		return;
	}
	
	//Ensure no one else is waiting for this same event
	if(e->owner > 0 && e->owner != Cp->pid)
	{
		#ifdef OS_DEBUG
			printf("Kernel_Wait_Event: The requested event is already being waited by PID %d\n", e->owner);
		#endif
		err = EVENT_NOT_FOUND_ERR;
		return;
	}
	
	//Has this event been signaled already? If yes, "consume" event and keep executing the same task
	if(e->count > 0)
	{
		e->owner = 0;
		e->count = 0;
		e->id = 0;
		--Event_Count;	
		return;
	}
	
	//Set the owner of the requested event to the current task and put it into the WAIT EVENT state
	e->owner = Cp->pid;
	Cp->state = WAIT_EVENT;
	err = NO_ERR;
}

static void Kernel_Signal_Event(void)
{
	EVENT_TYPE* e = findEventByEventID(Cp->request_arg);
	PD *e_owner;
	
	if(e == NULL)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Signal_Event: Error finding requested event!\n");
		#endif
		return;
	}
	
	//Increment the event counter if needed 
	if(MAX_EVENT_SIG_MISS == 0 || e->count < MAX_EVENT_SIG_MISS)
		e->count++;
	
	//If the event is unowned, return
	if(e->owner == 0)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Signal_Event: *WARNING* The requested event is not being waited by anyone!\n");
		#endif
		err = SIGNAL_UNOWNED_EVENT_ERR;
		return;
	}
	
	//Fetch the owner's PD and ensure it's still valid
	e_owner = findProcessByPID(e->owner);
	if(e_owner == NULL)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Signal_Event: Event owner's PID not found in global process list!\n");
		#endif
		err = PID_NOT_FOUND_ERR;
		return;
	}
	
	//Wake up the owner of the event by setting its state to READY if it's active. The event is "consumed"
	if(e_owner->state == WAIT_EVENT)
	{
		e->owner = 0;
		e->count = 0;
		e->id = 0;
		--Event_Count;
		e_owner->state = READY;
	}
}

/************************************************************************/
/*                  MUTEX RELATED KERNEL FUNCTIONS                      */
/************************************************************************/

void Kernel_Create_Mutex(void)
{
	int i;
	
	//Make sure the system's mutexes are not at max
	if(Mutex_Count >= MAXMUTEX)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Create_Mutex: Failed to create Mutex. The system is at its max mutex threshold.\n");
		#endif
		err = MAX_MUTEX_ERR;
		return;
	}
	
	//Find an uninitialized Mutex slot
	for(i=0; i<MAXMUTEX; i++)
		if(Mutex[i].id == 0) break;
	
	//Assign a new unique ID to the mutex. Note that the smallest valid mutex ID is 1.
	Mutex[i].id = ++Last_MutexID;
	Mutex[i].owner = 0;		// note when mutex's owner is 0, it is free
	// init priority stack
	for (int j=0; j<MAXTHREAD; j++) {
		Mutex[i].priority_stack[j] = LOWEST_PRIORITY+1;
		Mutex[i].blocked_stack[j] = -1;
		Mutex[i].order[j] = 0;
	}
	Mutex[i].num_of_process = 0;
	Mutex[i].total_num = 0;
	++Mutex_Count;
	err = NO_ERR;
	
	#ifdef OS_DEBUG
	printf("Kernel_Create_Mutex: Created Mutex %d!\n", Last_MutexID);
	#endif
}

static void Dispatch();

static void Kernel_Lock_Mutex(void)
{
	MUTEX_TYPE* m = findMutexByMutexID(Cp->request_arg);
	PD *m_owner = findProcessByPID(m->owner);
	
	if(m == NULL)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Lock_Mutex: Error finding requested mutex!\n");
		#endif
		return;
	}
	
	// if mutex is free
	if(m->owner == 0)
	{
		m->owner = Cp->pid;
		m->count = 1;
		m->own_pri = Cp->pri;				// keep track of the original priority of the owner
		return;
	} else if (m->owner == Cp->pid) {
		// if it has locked by the current process
		++(m->count);
		return;
	} else {
		Cp->state = WAIT_MUTEX;								//put cp into state wait mutex
		//enqueue cp to stack
		++(m->num_of_process);
		++(m->total_num);
		for (int i=0; i<MAXTHREAD; i++) {
			if (m->blocked_stack[i] == -1){
				m->blocked_stack[i] = Cp->pid;
				m->order[i] = m->total_num;
				m->priority_stack[i] = Cp->pri;
				break;	
			}
		}
		// end of enqueue
		
		//if cp's priority is higher than the owner
		if (Cp->pri < m_owner->pri) {
			m_owner->pri = Cp->pri;				// the owner gets cp's priority
		}
		Dispatch();
	}
}

static void Kernel_Unlock_Mutex(void)
{
	MUTEX_TYPE* m = findMutexByMutexID(Cp->request_arg);
	PD *m_owner = findProcessByPID(m->owner);
	
	if(m == NULL)
	{
		#ifdef OS_DEBUG
		printf("Kernel_Unlock_Mutex: Error finding requested mutex!\n");
		#endif
		return;
	}
	
	if(m->owner != Cp->pid){
		#ifdef OS_DEBUG
		printf("Kernel_Unlock_Mutex: The owner is not the current process\n");
		#endif
		return;
	} else if (m->count > 1) {
		// M is locked more than once
		--(m->count);
	} else if (m->num_of_process > 0) {
		// there are tasks waiting on the mutex
		// deque the task with highest priority
		PID p_dequeue = 0;
		unsigned int temp_order = m->total_num + 1;
		PRIORITY temp_pri = LOWEST_PRIORITY + 1;
		int i;
		for (i=0; i<MAXTHREAD; i++) {
			if (m->priority_stack[i] < temp_pri) {
				// found a task with higher priority
				temp_pri = m->priority_stack[i];
				temp_order = m->order[i];
				p_dequeue = m->blocked_stack[i];
			} else if (m->priority_stack[i] == temp_pri && temp_order < m->order[i]) {
				// same priority and came into the queue earlier
				temp_order = m->order[i];
				p_dequeue = m->blocked_stack[i];
			}
		}
		//dequeue index i
		m->blocked_stack[i] = -1;
		m->priority_stack[i] = LOWEST_PRIORITY+1;
		m->order[i] = 0;
		--(m->num_of_process);
		PD* target_p = findProcessByPID(p_dequeue);
		m_owner->pri = m->own_pri;		//reset owner's priority
		m->owner = p_dequeue;
		m->own_pri = temp_pri;			//keep track of new owner's priority;
		target_p->state = READY;
		Cp->state = READY;
		Dispatch();
		return;
	} else {
		m->owner = 0;
		m->count = 0;
		m_owner->pri = m->own_pri;		//reset owner's priority
		return;
	}
}

/************************************************************************/
/*                     TASK TERMINATE FUNCTION                         */
/************************************************************************/

static void Kernel_Terminate_Task(void)
{
	MUTEX_TYPE* m;
	// go through all mutex check if it owns a mutex
	int index;
	for (index=0; index<MAXMUTEX; index++) {
		if (Mutex[index].owner == Cp->pid) {
			// it owns a mutex unlock the mutex
			if (Mutex[index].num_of_process > 0) {
				printf("something is waiting\n");
				// if there are other process waiting on the mutex
				PID p_dequeue = 0;
				unsigned int temp_order = Mutex[index].total_num + 1;
				PRIORITY temp_pri = LOWEST_PRIORITY + 1;
				int i;
				for (i=0; i<MAXTHREAD; i++) {
					if (Mutex[index].priority_stack[i] < temp_pri) {
						// found a task with higher priority
						temp_pri = Mutex[index].priority_stack[i];
						temp_order = Mutex[index].order[i];
						p_dequeue = Mutex[index].blocked_stack[i];
						} else if (Mutex[index].priority_stack[i] == temp_pri && temp_order < Mutex[index].order[i]) {
						// same priority and came into the queue earlier
						temp_order = Mutex[index].order[i];
						p_dequeue = Mutex[index].blocked_stack[i];
					}
				}
				//dequeue index i
				Mutex[index].blocked_stack[i] = -1;
				Mutex[index].priority_stack[i] = LOWEST_PRIORITY+1;
				Mutex[index].order[i] = 0;
				--(Mutex[index].num_of_process);
				PD* target_p = findProcessByPID(p_dequeue);
				Mutex[index].owner = p_dequeue;
				Mutex[index].own_pri = temp_pri;			//keep track of new owner's priority;
				target_p->state = READY;
				printf("target p is readd\n");
			} else {
				Mutex[index].owner = 0;
				Mutex[index].count = 0;
			}
		}
	}
	Cp->state = DEAD;			//Mark the task as DEAD so its resources will be recycled later when new tasks are created
	--Task_Count;
}

/************************************************************************/
/*                     KERNEL SCHEDULING FUNCTIONS                      */
/************************************************************************/

/* This internal kernel function is a part of the "scheduler". It chooses the next task to run, i.e., Cp. */
static void Dispatch()
{
	unsigned int i = 0;
	int highest_pri = LOWEST_PRIORITY + 1;
	int highest_pri_index = -1;
	
	//Find the next READY task with the highest priority by iterating through the process list ONCE
	for(i=0; i<MAXTHREAD; i++)
	{
		//Increment process index
		NextP = (NextP + 1) % MAXTHREAD;
		
		//Select the READY process with the highest priority
		if(Process[NextP].state == READY && Process[NextP].pri < highest_pri)
		{
			highest_pri = Process[NextP].pri;
			highest_pri_index = NextP;
		}
	}
		
	//When none of the tasks in the process list is ready
	if(highest_pri_index == -1)
	{
		//We'll temporarily re-enable interrupt in case if one or more task is waiting on events/interrupts or sleeping
		Enable_Interrupt();
		
		//Looping through the process list until any process becomes ready
		while(Process[NextP].state != READY)
		{
			//Increment process index
			NextP = (NextP + 1) % MAXTHREAD;
			
			//Check if any timer ticks came in
			Kernel_Tick_Handler();	
		}
		
		//Now that we have a ready task, interrupts must be disabled for the kernel to function properly again.
		Disable_Interrupt();
	}
	else
		NextP = highest_pri_index;

	//Load the next selected task's process descriptor into Cp
	Cp = &(Process[NextP]);
	CurrentSp = Cp->sp;
	Cp->state = RUNNING;
}

/**
  * This internal kernel function is the "main" driving loop of this full-served
  * model architecture. Basically, on OS_Start(), the kernel repeatedly
  * requests the next user task's next system call and then invokes the
  * corresponding kernel function on its behalf.
  *
  * This is the main loop of our kernel, called by OS_Start().
  */
static void Next_Kernel_Request() 
{
	Dispatch();	//Select an initial task to run

	//After OS initialization, THIS WILL BE KERNEL'S MAIN LOOP!
	//NOTE: When another task makes a syscall and enters the loop, it's still in the RUNNING state!
	while(1) 
	{
		//Clears the process' request fields
		Cp->request = NONE;
		//Cp->request_arg is not reset, because task_sleep uses it to keep track of remaining ticks

		//Load the current task's stack pointer and switch to its context
		CurrentSp = Cp->sp;
		Exit_Kernel();

		/* if this task makes a system call, it will return to here! */

		//Save the current task's stack pointer and proceed to handle its request
		Cp->sp = CurrentSp;
		
		//Check if any timer ticks came in
		Kernel_Tick_Handler();

		switch(Cp->request)
		{
			case CREATE_T:
			Kernel_Create_Task(Cp->code, Cp->pri, Cp->arg);
			break;
			
			case TERMINATE:
			Kernel_Terminate_Task();
			Dispatch();					//Dispatch is only needed if the syscall requires running a different task  after it's done
			break;
		   
			case SUSPEND:
			Kernel_Suspend_Task();
			if(Cp->state != RUNNING) Dispatch();
			break;
			
			case RESUME:
			Kernel_Resume_Task();
			Dispatch();
			break;
			
			case SLEEP:
			Cp->state = SLEEPING;
			Dispatch();					
			break;
			
			case CREATE_E:
			Kernel_Create_Event();
			break;
			
			case WAIT_E:
			Kernel_Wait_Event();	
			if(Cp->state != RUNNING) Dispatch();	//Don't dispatch to a different task if the event is already siganlled
			break;
			
			case SIGNAL_E:
			Kernel_Signal_Event();
			Dispatch();
			break;
			
			case CREATE_M:
			Kernel_Create_Mutex();
			break;
			
			case LOCK_M:
			Kernel_Lock_Mutex();
			//Maybe add a dispatch() here if lock fails?
			break;
			
			case UNLOCK_M:
			Kernel_Unlock_Mutex();
			//Does this need dispatch under any circumstances?
			break;
		   
			case YIELD:
			case NONE:					// NONE could be caused by a timer interrupt
			Cp->state = READY;
			Dispatch();
			break;
       
			//Invalid request code, just ignore
			default:
				err = INVALID_KERNET_REQUEST_ERR;
			break;
       }
    } 
}

	
/************************************************************************/
/* KERNEL BOOT                                                          */
/************************************************************************/

/*Sets up the timer needed for task_sleep*/
void Timer_init()
{
	/*Timer1 is configured for the task*/
	
	//Use Prescaler = 256
	TCCR1B |= (1<<CS12);
	TCCR1B &= ~((1<<CS11)|(1<<CS10));
	
	//Use CTC mode (mode 4)
	TCCR1B |= (1<<WGM12);
	TCCR1B &= ~((1<<WGM13)|(1<<WGM11)|(1<<WGM10));
	
	OCR1A = TICK_LENG;			//Set timer top comparison value to ~10ms
	TCNT1 = 0;					//Load initial value for timer
	TIMSK1 |= (1<<OCIE1A);      //enable match for OCR1A interrupt
	
	#ifdef OS_DEBUG
	printf("Timer initialized!\n");
	#endif
}

/*This function initializes the RTOS and must be called before any othersystem calls.*/
void OS_Init()
{
	int x;
	
	Task_Count = 0;
	Event_Count = 0;
	KernelActive = 0;
	Tick_Count = 0;
	NextP = 0;
	Last_PID = 0;
	Last_EventID = 0;
	Last_MutexID = 0;
	err = NO_ERR;
	
	//Clear and initialize the memory used for tasks
	memset(Process, 0, MAXTHREAD*sizeof(PD));
	for (x = 0; x < MAXTHREAD; x++) {
		Process[x].state = DEAD;
	}
	
	//Clear and initialize the memory used for Events
	memset(Event, 0, MAXEVENT*sizeof(EVENT_TYPE));
	for (x = 0; x < MAXEVENT; x++) {
		Event[x].id = 0;
	}
	
	//Clear and initialize the memory used for Mutex
	memset(Mutex, 0, MAXMUTEX*sizeof(MUTEX_TYPE));
	for (x = 0; x < MAXMUTEX; x++) {
		Event[x].id = 0;
	}
	
	#ifdef OS_DEBUG
	printf("OS initialized!\n");
	#endif
}

/* This function starts the RTOS after creating a few tasks.*/
void OS_Start()
{
	if ( (! KernelActive) && (Task_Count > 0))
	{
		Disable_Interrupt();
		
		/* we may have to initialize the interrupt vector for Enter_Kernel() here. */
			/* here we go...  */
		KernelActive = 1;
		
		/*Initialize and start Timer needed for sleep*/
		Timer_init();
		
		#ifdef OS_DEBUG
		printf("OS begins!\n");
		#endif
		
		Next_Kernel_Request();
		/* NEVER RETURNS!!! */
	}
}
