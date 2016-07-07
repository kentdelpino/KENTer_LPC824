# A Minimum Inter-process Communication Kernel (MICK)  

By Kent del Pino; Valencia - Nov 2015  -- early stage --

 (-: For future inspiration in the very small, I look at suzuha and or TI-RTOS :-)


A minimum and basic software-base for task/thread management that include handling of tasks with 
the same priority and high priority tasks and with some means of communication between tasks/ 
processes, limited Inter-Process Communication (IPC). Minimum IPC because we also want minimum 
code-footprint (less than 3K byte) on our ARM Cortex-M0. 

We want same-priority tasks to time-slice. Working with string-formats as in formatting of
and decoding of JSON for IO, can take time, so We go slow on those tasks, report busy if 
necessary. The algorithm is priority driven time-slicing on a preemptive scheduler.



--- The future
Also important is it that a task release it resource consumption when the task is done (change 
state) and allows the kanel/system to enter a sleep-mode (low-power). 
-----

Note on minimum:
 Super Simple Tasker (SST) / scheduler, require that all tasks run to completion. Can be preempted 
 by higher priority tasks. SST is good for very small stacks(RAM). We actually have some RAM for 
 stacking and want multi-tasking and to be able to pause (delay) a tasks in a sequential process.


Links:
https://en.wikipedia.org/wiki/Microkernel
http://www.embedded.com/design/prototyping-and-development/4025691/Build-a-Super-Simple-Tasker


