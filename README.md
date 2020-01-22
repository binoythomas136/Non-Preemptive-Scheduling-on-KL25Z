# Non-Preemptive-Scheduling-on-KL25Z
This project was completed on the Freedom board KL25Z

Non preemptive scheduling was used to improve the scheduling process and the LED flashes when the on-board accelorometer detects motion

The program was converted into a finite state machine inorder to structure the code and to ensure that the scheduler was properly utilized without much time wasting The project file with non_preemptive scheduling consits of the project with this code

Finally the code was converted to an event triggered code such that each task is an event and subsequent tasks are completed at periodic events. The project file with non_preemptive scheduling with event triggered operation consits of the project with this code
