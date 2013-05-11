//Instead of writing an interface function to the RTAI functions in pascal,
//it is easier to write a simple library that uses the functions I need.
//This avoids the problem of working out the C definitions for the RTAI
//header files
//Started March 16, 2011
//Last updated March 29, 2011

//The standard  libraries
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <math.h>

//The RTAI libraries
#include <rtai_lxrt.h>
#include <rtai_msg.h>
#include <rtai_comedi.h>
#include <rtai_spm.h>


//define some constants
#define CPUMAP 0xF
#define NCHAN 1
#define PID_cutoff_N 10

//some types


// The functions contained in this file are pretty dummy
// and are included only as a placeholder. Nevertheless,
// they *will* get included in the shared library if you
// don't remove them :)
//
// Obviously, you 'll have to write yourself the super-duper
// functions to include in the resulting library...
// Also, it's not necessary to write every function in this file.
// Feel free to add more files in this project. They will be
// included in the resulting library.



//static pthread_t pid_thread; // Points to where the thread ID will be stored
static RT_TASK *PIDloop_Task; // Pid  task

//static pthread_t sine_thread; // Points to where the thread ID will be stored
static RT_TASK *Sinewaveloop_Task; // Pid  task
static RTIME expected;
static double start_time;
static double old_time;
static double current_time;
static RT_TASK *GlobalTask;


//*****************************************************************************

int StartMainTask( int priority)
{
 //function to start a real time task with the given priority
 //StartMainTask is designed to be called from the Pascal Program to start the Main (i.e., Pascal) rt program


     if(!(GlobalTask = rt_task_init_schmod(nam2num( "SomeTask" ), // Name
                                        priority, // Priority
                                        0, // Stack Size
                                        0, //, // max_msg_size
                                        SCHED_FIFO, // Policy
                                        CPUMAP ))) // cpus_allowed
        {
            return 1;
        }
       else {
           return 0;
       }
}


//***************************************************************************************
int EndMainTask()
{
    //EndMainTask is designed to be called from the Pascal Program to end the Main (i.e., Pascal) rt program
    rt_task_delete(GlobalTask);
   return 0;
}

//****************************************************************************************

int sineoutput()
 {
    //Initial test function to try out Real time stuff.
    int m, i=0;
    lsampl_t data_to_card;
    static comedi_t * dev;

    dev = comedi_open(device_names[AnalogOutputChannel.board_number]);

    if(!(Sinewaveloop_Task = rt_task_init_schmod(nam2num( "Sinewave" ), // Name
                                        2, // Priority
                                        0, // Stack Size
                                        0, //, // max_msg_size
                                        SCHED_FIFO, // Policy
                                        CPUMAP ))) // cpus_allowed
        {
            printf("ERROR: Cannot initialize sinewave task\n");
            exit(1);
        }

    //specify that this is to run on one CPU
    rt_set_runnable_on_cpuid(Sinewaveloop_Task, 0);
    //Convert samp_time, which is in nanoseconds, to tick time
    //sampling_interval = nano2count(SAMP_TIME);  //Converts a value from
                                                //nanoseconds to internal count units.
    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_make_hard_real_time();
    sampling_interval =nano2count(SAMP_TIME);
     rt_printk("Sampling interval is %f12 \n",count2nano((float) sampling_interval));
    // Let's make this task periodic..
    expected = rt_get_time() + 100*sampling_interval;



    rt_task_make_periodic(Sinewaveloop_Task, expected, sampling_interval); //period in counts
    //rt_task_resume(Sinewaveloop_Task);
    sine_loop_running=1;



    // Concurrent function Loop


     rt_printk("SineWaveAmplitude is is %f \n",SineWaveAmplitude);
     rt_printk("SineWaveFrequency is %f \n",SineWaveFrequency);
     rt_printk("sine_loop_running is %d \n",sine_loop_running);
     rt_printk("SAMP_TIME is %d \n",SAMP_TIME);
     start_time = (float)rt_get_time_ns()/1E9; //in seconds
     old_time = start_time;
     rt_printk("AnalogOutputChannel board_it is %p \n",AnalogOutputChannel.board_id);
     rt_printk("AnalogOutputChannel devicename is %p \n",*(AnalogOutputChannel.devicename));
     rt_printk("AnalogOutputChannel boardname is %p \n",*(AnalogOutputChannel.boardname));
     rt_printk("AnalogOutputChannel subdevice is %d \n",AnalogOutputChannel.subdevice);
     rt_printk("AnalogOutputChannel channel is %d \n",AnalogOutputChannel.channel);
     OutputValue = 1;
     //sine_loop_running = 0;  //set this to 0 for testing
     while(sine_loop_running)
     {
        i++; // Count Loops.
        current_time = (float)rt_get_time_ns()/1E9;
        //rt_printk("LOOP %d,-- Period time: %f12 %f12\n",i, current_time - old_time,count2nano((float)sampling_interval)/1E9);
        OutputValue = SineWaveAmplitude*sin(2*PI*SineWaveFrequency*(current_time-start_time));
        //OutputValue = -1*OutputValue;
        //rt_printk("OutputValue is %f12 \n",OutputValue);
        data_to_card = (lsampl_t) nearbyint(((OutputValue - MinOutputVoltage)/OutputRange)*MaxOutputBits);
        //m=rt_comedi_command_data_write(AnalogOutputChannel.board_id, AnalogOutputChannel.subdevice, NCHAN, data_to_card);
        comedi_lock(dev, AnalogOutputChannel.subdevice);
        m=comedi_data_write(dev, AnalogOutputChannel.subdevice, AnalogOutputChannel.channel, AO_RANGE, AREF_DIFF, data_to_card);
        comedi_unlock(dev, AnalogOutputChannel.subdevice);
//        m=comedi_data_write(AnalogOutputChannel.board_id, AnalogOutputChannel.subdevice,
//               AnalogOutputChannel.channel, AO_RANGE, AREF_GROUND, data_to_card);
        //rt_printk("Data_to_card is %d; result from rt_comedi_command_data_write is %d \n",data_to_card, m);
        //rt_printk("LOOP %d,-- AO Out time: %f12 \n",i, (float)rt_get_time_ns()/1E9 - current_time);
        //rt_printk("Data_to_card is %d \n",data_to_card);
        //old_time = current_time;
/*        if (i== 100000)
        {
            sine_loop_running = 0;
            //printf("LOOP -- run: %d %d\n ",keep_on_running,&keep_on_running);
            //printf("RTAI LOOP -- run: %d \n ",i);
            break;
        }
*/
        rt_task_wait_period(); // And waits until the end of the period.

    }
    rt_make_soft_real_time();
    comedi_close(dev);
    rt_task_delete(Sinewaveloop_Task); //Self termination at end.

    pthread_exit(NULL);
    return 0;
 }


//*******************************************************************************
int pid_loop()

{
//This is the function to output a PID loop
//PID algorithm taken from Control System Desgin, by Karl Johan Astrom
//Chapter 6
//This algorithm is supposed to include integral wind-up and bumpless transition

    int m;
    static double AntiWindup_Time;
    lsampl_t data_to_card, data_from_card;
    static comedi_t * dev_output, * dev_input;
    static double bi, ad, bd, a0, b; //PID coefficients
    static double Pcontrib, Icontrib, Dcontrib; //individual PID contributions
    static double FeedbackReading, OldFeedbackReading; //Readings of the error chann
    static double TempSetPoint; //Temporary Set Point
    static double TempSetPointStep; //change in the set point during bumpless transition
    static int BumplessTransitionSteps; //Number of steps to increas
    static double v; //u is the actuator output, and v is the calculated output
    static int j = 0;
    static int i = 0;
    static double SummedPIDOutput; //Summed PID Output




    rt_printk("Control channel subdevice %d and channel %d \n", ControlChannel.subdevice, ControlChannel.channel);
    rt_printk("Feedback channel subdevice %d and channel %d \n", FeedbackChannel.subdevice, FeedbackChannel.channel);
    //dev_output is the channel that is to be controlled
    dev_output = comedi_open(device_names[ControlChannel.board_number]);
    //dev_input is the channel from which the error signal is read
    dev_input = comedi_open(device_names[FeedbackChannel.board_number]);

    //initialize the task
    if(!(PIDloop_Task = rt_task_init_schmod(nam2num( "PIDLoop" ), // Name
                                        2, // Priority
                                        0, // Stack Size
                                        0, //, // max_msg_size
                                        SCHED_FIFO, // Policy
                                        CPUMAP ))) // cpus_allowed
        {
            rt_printk("ERROR: Cannot initialize PIDLoop task\n");
            exit(1);
        }

    //specify that this is to run on one CPU
    rt_set_runnable_on_cpuid(PIDloop_Task, 0);


    //lock memory and make hard real time
    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_make_hard_real_time();

    //Convert PIDLoop_time, which is in nanoseconds, to tick time (sampling_interval, in counts)
    sampling_interval =nano2count(PIDLoop_Time);

    // Let's make this task periodic..
    expected = rt_get_time() + 100*sampling_interval;
    rt_task_make_periodic(PIDloop_Task, expected, sampling_interval); //period in counts


    pid_loop_running = 1; //set the pid loop running flag to FALSE

    //Initialize the controller coefficients
    b = 0.5;
    AntiWindup_Time = sqrt(IntTime*DiffTime);
    rt_printk("AntiWindup time is %f \n", AntiWindup_Time);
    bi = PropCoff*PIDLoop_Time/IntTime;  //integral gain
    rt_printk("bi is %f \n", bi);
    ad = (2*DiffTime- PID_cutoff_N*PIDLoop_Time)/(2*DiffTime+PID_cutoff_N*PIDLoop_Time);
    rt_printk("ad is %f \n", ad);
    bd = 2*PropCoff*PID_cutoff_N*DiffTime/(2*DiffTime + PID_cutoff_N*PIDLoop_Time);    //derivative gain
    rt_printk("bd is %f \n", bd);
    a0 = PIDLoop_Time/AntiWindup_Time;
    rt_printk("a0 is %f \n", a0);
    Dcontrib = 0;
    Icontrib = 0;

   // First a procedure for bumpless transitions
   if(FirstPIDPass) //if this is the first time this PID cycle is starting
     {  //First read the input reading
         comedi_lock(dev_input,FeedbackChannel.subdevice);
         m = comedi_data_read(dev_input, FeedbackChannel.subdevice, FeedbackChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
         comedi_unlock(dev_input,FeedbackChannel.subdevice);

         //Convert to a voltage reading
         FeedbackReading = (float) ((((int) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage);

         //Set the time taken to get to transition to the requested set point to be 10 times the integral time constant
         BumplessTransitionSteps = nearbyint((1000*IntTime)/PIDLoop_Time);


         rt_printk("Number of transition steps is %d \n", BumplessTransitionSteps);

         TempSetPointStep = (SetPoint - FeedbackReading)/BumplessTransitionSteps;
         TempSetPoint = FeedbackReading;

         rt_printk("Temporary set point is %f \n", TempSetPoint);
         rt_printk("Integration time is %f \n", IntTime);
         rt_printk("PID loop time is %d \n", PIDLoop_Time);
         rt_printk("pid_loop_running is %d \n", pid_loop_running);

         rt_printk("Got here 1 \n");

         i =0;
         OldFeedbackReading = FeedbackReading;
         while(i<BumplessTransitionSteps)
          {
            rt_printk("Got here 2 no. %d \n", i);
            i++;
            TempSetPoint+=TempSetPointStep;  //change the temporary set point

            //rt_printk("Temporary set point is %f \n", TempSetPoint);

            //First read the input reading
            comedi_lock(dev_input, FeedbackChannel.subdevice);
            m = comedi_data_read(dev_input, FeedbackChannel.subdevice, FeedbackChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
            comedi_unlock(dev_input, FeedbackChannel.subdevice);

            //Convert to a voltage reading
            FeedbackReading = (float) ((((int) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage);
            Pcontrib = PropCoff*(b*TempSetPoint - FeedbackReading);
            Dcontrib = ad*Dcontrib - bd*(FeedbackReading - OldFeedbackReading);
            rt_printk("PContrib is. %f \n", Pcontrib);
            rt_printk("IContrib is. %f \n", Icontrib);
            rt_printk("DContrib is. %f \n", Dcontrib);
            rt_printk("Setpoint is %f \n", TempSetPoint);
            rt_printk("y is %f \n", FeedbackReading);
            v = Pcontrib + Icontrib + Dcontrib;
            //next, take care of saturation of the output....anti-windup
            rt_printk("v is. %f \n", v);
            PIDOutput = v;
            rt_printk("PIDOutput is. %f \n", PIDOutput);
            PIDOutput =(PIDOutput>MaxOutputVoltage)? MaxOutputVoltage:PIDOutput;
            rt_printk("PIDOutput is. %f \n", PIDOutput);
            PIDOutput =(PIDOutput<MinOutputVoltage)? MinOutputVoltage:PIDOutput;
            rt_printk("PIDOutput is. %f \n", PIDOutput);
            //send the control signal
            data_to_card = (lsampl_t) nearbyint(((OutputPhase*PIDOutput - MinOutputVoltage)/OutputRange)*MaxOutputBits);
            comedi_lock(dev_output, ControlChannel.subdevice);
            m=comedi_data_write(dev_output, ControlChannel.subdevice, ControlChannel.channel, AO_RANGE, AREF_DIFF, data_to_card);
            comedi_unlock(dev_output, ControlChannel.subdevice);
            Icontrib += bi*(TempSetPoint-FeedbackReading) + a0*(PIDOutput-v); //antiwindup update of
            Icontrib =(Icontrib>100)? 100:Icontrib; //limit IContrib to be 100
            Icontrib =(Icontrib<-100)? -100:Icontrib;
            OldFeedbackReading = FeedbackReading;
            AveragedPIDOutput = PIDOutput;  //No averages after initially turning on the system

            // And wait until the end of the period.
            rt_task_wait_period();

          }  //end of i<=BumplessTransitions
         FirstPIDPass = 0; //set to TRUE
         rt_printk("pid_loop_running is %d \n", pid_loop_running);
     }   //end of if(NotFirstPIDPass

    //Now do the regular loop
    SummedPIDOutput = 0; //initialize parameters if we take averages
    j=1;
    while(pid_loop_running)
      {
      //rt_printk("Got here 1 \n");
      //check to see if the PID parameters have changed
      if(PIDParametersChanged)
        {
          //update the PID coefficients
          rt_printk("Got here 2 \n");
          AntiWindup_Time = sqrt(IntTime*DiffTime);
          bi = PropCoff*PIDLoop_Time/IntTime;  //integral gain
          ad = (2*DiffTime- PID_cutoff_N*PIDLoop_Time)/(2*DiffTime+PID_cutoff_N*PIDLoop_Time);
          bd = 2*PropCoff*PID_cutoff_N*DiffTime/(2*DiffTime + PID_cutoff_N*PIDLoop_Time);    //derivative gain
          a0 = PIDLoop_Time/AntiWindup_Time;
          PIDParametersChanged = 0;
        } //end of if(PIDParametersChanged)

      //continue with the rest of the loop

      //Read the input reading
      comedi_lock(dev_input, FeedbackChannel.subdevice);
      m = comedi_data_read(dev_input, FeedbackChannel.subdevice, FeedbackChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
      comedi_unlock(dev_input, FeedbackChannel.subdevice);

      //Convert to a voltage reading
      FeedbackReading = (float) ((((int) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage);
      Pcontrib = PropCoff*(b*SetPoint - FeedbackReading);
      Dcontrib = ad*Dcontrib - bd*(FeedbackReading - OldFeedbackReading);
      v = Pcontrib + Icontrib + Dcontrib;

      //next, take care of saturation of the output....anti-windup
      PIDOutput = v;
      PIDOutput =(PIDOutput>MaxOutputVoltage)? MaxOutputVoltage:PIDOutput;
      PIDOutput =(PIDOutput<MinOutputVoltage)? MinOutputVoltage:PIDOutput;

      //send the control signal
      //rt_printk("FeedbackReading is %f \n", FeedbackReading);
      //rt_printk("v is %f \n", v);
      //rt_printk("PID output should be %f \n", PIDOutput);
      data_to_card = (lsampl_t) nearbyint(((OutputPhase*PIDOutput - MinOutputVoltage)/OutputRange)*MaxOutputBits);
      comedi_lock(dev_output, ControlChannel.subdevice);
      m=comedi_data_write(dev_output, ControlChannel.subdevice, ControlChannel.channel, AO_RANGE, AREF_DIFF, data_to_card);
      comedi_unlock(dev_output, ControlChannel.subdevice);
      Icontrib += bi*(SetPoint-FeedbackReading) + a0*(PIDOutput-v); //antiwindup update of
      Icontrib =(Icontrib>100)? 100:Icontrib; //limit IContrib to be 100
      Icontrib =(Icontrib<-100)? -100:Icontrib;
      OldFeedbackReading = FeedbackReading;


      //Next part is to take the averaged PID output for recording if j>PID_averages and PID_averages>1
      SummedPIDOutput+=PIDOutput;
      j++;
      AveragedPIDOutput=((PID_averages>1)&&(j>PID_averages))?(SummedPIDOutput/PID_averages):AveragedPIDOutput;
      SummedPIDOutput=(j>PID_averages)? 0:SummedPIDOutput;
      j=(j>PID_averages)? 1:j;


         // And wait until the end of the period.
        rt_task_wait_period();

       }

    rt_printk("Got here 3 \n");
    rt_printk("pid_loop_running is %d \n", pid_loop_running);
    rt_make_soft_real_time();
    comedi_close(dev_input);
    comedi_close(dev_output);
    rt_task_delete(PIDloop_Task); //Self termination at end.

    pthread_exit(NULL);
    return 0;


}

