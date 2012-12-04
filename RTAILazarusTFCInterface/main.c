//Instead of writing an interface function to the RTAI functions in pascal,
//it is easier to write a simple library that uses the functions I need.
//This avoids the problem of working out the C definitions for the RTAI
//header files
//Started March 16, 2011
//Last updated March 29, 2011
//This version is modified for the tuning fork simulator, started Dec 6, 2011

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
#define PID_cutoff_N 20
#define MaxvOutput 1000
#define MinvOutput -1000


//the queue.ch file
#include <queue.c>

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
static RT_TASK *TFloop_Task;  //task for the Tuning fork simulator
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
                                        0, // Priority
                                        0, // Stack Size
                                        0, //, // max_msg_size
                                        SCHED_FIFO, // Policy
                                        CPUMAP ))) // cpus_allowed
        {
            printf("ERROR: Cannot initialize sinewave task\n");
            exit(1);
        }

    //specify that this is to run on one CPU
    rt_set_runnable_on_cpuid(Sinewaveloop_Task, 2);
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

//****************************************************************************************

int tf_loop()
 {
    //Initial test function to try out Real time stuff.
    int m, i=0;
    lsampl_t data_to_card, data_from_card;
    static comedi_t * dev_in, * dev_out, * dev_x, * dev_y;
    double XModPosition, YModPosition; //Scan positions, modulo the FeaturePeriod
    double delta_X; //height of tip above feature

    dev_in = comedi_open(device_names[InputChannel.board_number]);
    dev_out = comedi_open(device_names[OutputChannel.board_number]);
    dev_x = comedi_open(device_names[XChannel.board_number]);
    dev_y = comedi_open(device_names[YChannel.board_number]);

    if(!(TFloop_Task = rt_task_init_schmod(nam2num( "TFloop" ), // Name
                                        0, // Priority
                                        0, // Stack Size
                                        0, //, // max_msg_size
                                        SCHED_FIFO, // Policy
                                        CPUMAP ))) // cpus_allowed
        {
            printf("ERROR: Cannot initialize TF loop task\n");
            exit(1);
        }

    //specify that this is to run on one CPU
    rt_set_runnable_on_cpuid(TFloop_Task, 0);
    //Convert samp_time, which is in nanoseconds, to tick time
    //sampling_interval = nano2count(SAMP_TIME);  //Converts a value from
                                                //nanoseconds to internal count units.
    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_make_hard_real_time();
    sampling_interval =nano2count(SAMP_TIME);
     rt_printk("Sampling interval is %f12 \n",count2nano((float) sampling_interval));
    // Let's make this task periodic..
    expected = rt_get_time() + 100*sampling_interval;



    rt_task_make_periodic(TFloop_Task, expected, sampling_interval); //period in counts
    //rt_task_resume(Sinewaveloop_Task);
    tf_loop_running=1;



    // Concurrent function Loop


     rt_printk("SAMP_TIME is %d \n",SAMP_TIME);
     start_time = (float)rt_get_time_ns()/1E9; //in seconds
     old_time = start_time;
     rt_printk("Got here 1 \n");
     rt_printk("InputChannel board_it is %p \n",InputChannel.board_id);
     rt_printk("InputChannel devicename is %p \n",*(InputChannel.devicename));
     rt_printk("InputChannel boardname is %p \n",*(InputChannel.boardname));
     rt_printk("InputChannel subdevice is %d \n",InputChannel.subdevice);
     rt_printk("InputChannel channel is %d \n",InputChannel.channel);
     rt_printk("TestControlSignal is %d \n",TestControlSignal);
     rt_printk("ExternalScan is %d \n",ExternalScan);
     rt_printk("x_06 is %f12 \n",x_06);
     rt_printk("FeatureHeight is %f12 \n", FeatureHeight);
     rt_printk("FeatureSize is %f12 \n",FeatureSize);
     rt_printk("OutputConversionFactor is %f12 \n",OutputConversionFactor);
     rt_printk("AmplifierGainSign is %f12 \n",AmplifierGainSign);
     OutputValue = 2;
     //tf_loop_running = 0;  //set this to 0 for testing
     while(tf_loop_running)
     {
        i++; // Count Loops.
        current_time = (float)rt_get_time_ns()/1E9;
        //rt_printk("LOOP %d,-- Period time: %f12 %f12\n",i, current_time - old_time,count2nano((float)sampling_interval)/1E9);
        if (TestControlSignal==0) //then read the input channel
           {
            //read the input channel
            comedi_lock(dev_in, InputChannel.subdevice);
            m = comedi_data_read(dev_in, InputChannel.subdevice, InputChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
            comedi_unlock(dev_in, InputChannel.subdevice);

            //Convert the voltage read to a displacement in nanometers
            TubeDisplacement = ControlMultFact*((((float) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage) + ControlAddFact;

           } // else TubeDisplacement is determined by the main program


        if (ExternalScan!=0) // read the x and y scan input
           {
            //read the x channel
            comedi_lock(dev_x, XChannel.subdevice);
            m = comedi_data_read(dev_x, XChannel.subdevice, XChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
            comedi_unlock(dev_x, XChannel.subdevice);

            //Convert the voltage read to a displacement in nanometers
            XPosition = ((((float) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage);

            //read the y channel
            comedi_lock(dev_y, YChannel.subdevice);
            m = comedi_data_read(dev_y, YChannel.subdevice, YChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
            comedi_unlock(dev_y, YChannel.subdevice);

            //Convert the voltage read to a displacement in nanometers

            YPosition = ((((float) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage);
           }  //else both X position and Y Position are provided by the main program

        //Now we determine whether the XPosition and YPosition are such that the height should be modified
        XModPosition = XPosition - floor(XPosition/FeaturePeriod);
        YModPosition = YPosition - floor(YPosition/FeaturePeriod);
        XHeight = 0.0;

        if ((XModPosition<=((FeatureSize/2)+FeatureCenter))&&(XModPosition>=(FeatureCenter-(FeatureSize/2)))&&(YModPosition<=((FeatureSize/2)+FeatureCenter))&&(YModPosition>=(FeatureCenter-(FeatureSize/2))))
            {
             XHeight= FeatureHeight;
            }
        delta_X =((TubeDisplacement-XHeight)<min_delta_x)? min_delta_x:(TubeDisplacement-XHeight);
        FrequencyShift = (double) LJ_FreqShiftMultFact*((13/powl(delta_X, 14)) - (7/(x_06*powl(delta_X,8))));
        //FrequencyShift = (double) (LJ_FreqShiftMultFact*powl(delta_X, 14))*(13 - (7/x_06)*powl(delta_X,6));
        OutputValue = AmplifierGainSign*OutputConversionFactor*FrequencyShift;
        OutputValue = (OutputValue>MaxOutputVoltage)? MaxOutputVoltage:OutputValue;
        OutputValue = (OutputValue<MinOutputVoltage)? MinOutputVoltage:OutputValue;
        //OutputValue = 10;
        //rt_printk("OutputValue is %f12 \n",OutputValue);
        //rt_printk("Digital value is %f \n", nearbyint(((OutputValue - MinOutputVoltage)/OutputRange)*MaxOutputBits));
        data_to_card = (lsampl_t) nearbyint(((OutputValue - MinOutputVoltage)/OutputRange)*MaxOutputBits);

        //m=rt_comedi_command_data_write(AnalogOutputChannel.board_id, AnalogOutputChannel.subdevice, NCHAN, data_to_card);
        comedi_lock(dev_out, OutputChannel.subdevice);
        m=comedi_data_write(dev_out, OutputChannel.subdevice, OutputChannel.channel, AO_RANGE, AREF_DIFF, data_to_card);
        comedi_unlock(dev_out, OutputChannel.subdevice);
 //       m=comedi_data_write(AnalogOutputChannel.board_id, AnalogOutputChannel.subdevice,
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
    comedi_close(dev_in);
    comedi_close(dev_out);
    comedi_close(dev_x);
    comedi_close(dev_y);
    rt_task_delete(TFloop_Task); //Self termination at end.

    pthread_exit(NULL);
    return 0;
 }
//*******************************************************************************
int pid_loop()

//Modified on May 8 to take into account a moving average, and a moving variance
//and also to remove the retraction of the piezo except on the first pass.

{
//This is the function to output a PID loop
//PID algorithm taken from Control System Desgin, by Karl Johan Astrom
//Chapter 6
//This algorithm is supposed to include integral wind-up and bumpless transition

    int m;
    lsampl_t data_to_card, data_from_card;
    static comedi_t * dev_output, * dev_input;
    static double bi, ad, bd; //PID coefficients
    static double Pcontrib, Icontrib, Dcontrib; //individual PID contributions
    static double FeedbackReading; //Readings of the error chann
    static double v; //u is the actuator output, and v is the calculated output
    static int j = 0;
    static double LastDiffContrib;
    static double Error;
    static double LastError =0;
    static double SecondLastError =0;
    static double LastOutput =0;
    //static double SummedPIDOutput; //Summed PID Output
    static double SummedFeedbackReading; //Summed FeedbackReading
    //static double SummedVariance;
    static double M2_n;
    static double delta;
    static double alpha;
    static struct queue PIDOutput_queue;//these are two queues to calculate the moving mean and variance
    static struct queue FeedbackReadingVar_queue;
    static struct queue FeedbackReading_queue;
    static int NumbFirstSteps;
    static double InitialStepSizeVoltage = 0.1;
    static double InitialVoltageStep;
    double last_mean, last_var, new_var; //popped values of mean and variance



    //Initialize the queues
    init_queue(&PIDOutput_queue);
    init_queue(&FeedbackReadingVar_queue);
    init_queue(&FeedbackReading_queue);

    //rt_printk("Control channel device name is %s \n",device_names[ControlChannel.board_number]);
    //rt_printk("Control channel subdevice %d and channel %d \n", ControlChannel.subdevice, ControlChannel.channel);

    //rt_printk("Feedback channel device name is %s \n",device_names[FeedbackChannel.board_number]);
    //rt_printk("Feedback channel subdevice %d and channel %d \n", FeedbackChannel.subdevice, FeedbackChannel.channel);

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

    //retract the tip completely, if it is the first PID pass
    if(FirstPIDPass)
      {
        //data_to_card = (lsampl_t) 0;
        //MaxZVoltage corresponds to the fully retracted piezo
        rt_printk("MaxZVoltage is %f \n", MaxZVoltage);
        rt_printk("MinZVoltage is %f \n", MinZVoltage);
        rt_printk("MinOutputVoltage is %f \n", MinOutputVoltage);
        rt_printk("PIDOutput is %f \n", PIDOutput);
        rt_printk("AmplifierGainSign is %i \n", AmplifierGainSign);
        rt_printk("OutputPhase is %i \n", OutputPhase);
        NumbFirstSteps = (nearbyint((MaxZVoltage-PIDOutput)/InitialStepSizeVoltage))-1;
       //NumbFirstSteps = ((MaxZVoltage - PIDOutput)/InitialStepSizeVoltage)); //-1 to  be safe
        //Set the direction of the voltage step
        //PIDOutput = CurrentZVoltage;
        if (MaxZVoltage>=PIDOutput)
          {InitialVoltageStep=InitialStepSizeVoltage;}
         else {InitialVoltageStep=-InitialStepSizeVoltage;};

        if (NumbFirstSteps>1)
          {
            for(j=0;j<NumbFirstSteps;j++)
              {  PIDOutput+=InitialVoltageStep;
                 data_to_card = (lsampl_t) nearbyint(((PIDOutput - MinOutputVoltage)/OutputRange)*MaxOutputBits);
                 //rt_printk("Data_to_card is %i \n", data_to_card);
                 comedi_lock(dev_output, ControlChannel.subdevice);
                 m=comedi_data_write(dev_output, ControlChannel.subdevice, ControlChannel.channel, AO_RANGE, AREF_DIFF, data_to_card);
                 comedi_unlock(dev_output, ControlChannel.subdevice);
                // And wait until the end of the period.
                rt_task_wait_period();
              }
          }
        //Initialize the errors
        LastError = 0;
        SecondLastError = 0;
        LastOutput = PIDOutput;
        LastDiffContrib =0;
        Dcontrib = 0;
        Icontrib = 0;
        AveragedPIDOutput=LastOutput;  //This is what the main program will actually read
        FirstPIDPass = 0;
      }




    //rt_printk("AntiWindup time is %f \n", AntiWindup_Time);
    bi = PropCoff*PIDLoop_Time/IntTime;  //integral gain
    rt_printk("PropCoff is %f \n", PropCoff);
    rt_printk("IntTime is %f \n", IntTime);
    //in Astrom's article, ad is defined as below in the code, but the actual
    //derivation gives the coefficient we actually use
    //ad = (2*DiffTime- PID_cutoff_N*PIDLoop_Time)/(2*DiffTime+PID_cutoff_N*PIDLoop_Time);
    ad = (DiffTime)/(DiffTime+PID_cutoff_N*PIDLoop_Time);
    rt_printk("DiffTime is %f \n", DiffTime);
    //same comment about bd
    //bd = 2*PropCoff*PID_cutoff_N*DiffTime/(2*DiffTime + PID_cutoff_N*PIDLoop_Time);    //derivative gain
    bd = PropCoff*PID_cutoff_N*DiffTime/(DiffTime + PID_cutoff_N*PIDLoop_Time);
    //rt_printk("MaxZVoltage is %f \n", MaxZVoltage);


    //Now calculate the initial means and variances
    //SummedPIDOutput = 0; //initialize parameters if we take averages
    //First means
    SummedFeedbackReading =0;
    //j=1;
    alpha =  ((float) 1)/(PID_averages+1);
    for (j=0;j<PID_averages;j++)
      {

        //make a first reading
        comedi_lock(dev_input, FeedbackChannel.subdevice);
        m = comedi_data_read(dev_input, FeedbackChannel.subdevice, FeedbackChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
        comedi_unlock(dev_input, FeedbackChannel.subdevice);

        //Convert to a voltage reading
        SummedFeedbackReading += ((((float) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage);
      }
    AveragedFeedbackReading =SummedFeedbackReading/PID_averages;


    //Since we are not changing the output, the mean has not changed, and the variance is 0
    M2_n = 0;
    PIDOutputVariance = 0;

    //Initialize the circular buffers
    for (j=0; j<PID_averages; j++)
      {
        push_queue(&FeedbackReading_queue, AveragedFeedbackReading);
        push_queue(&FeedbackReadingVar_queue, PIDOutputVariance);
        push_queue(&PIDOutput_queue, LastOutput);
      }

    //Now do the regular loop
    while(pid_loop_running)
      {
      //rt_printk("Got here 1 \n");
      //check to see if the PID parameters have changed
      if(PIDParametersChanged)
        {
          //update the PID coefficients
          bi = PropCoff*PIDLoop_Time/IntTime;  //integral gain
          ad = (DiffTime)/(DiffTime+PID_cutoff_N*PIDLoop_Time);
          bd = PropCoff*PID_cutoff_N*DiffTime/(DiffTime + PID_cutoff_N*PIDLoop_Time);
          PIDParametersChanged = 0;
        } //end of if(PIDParametersChanged)

      //continue with the rest of the loop

      //Read the input reading
      comedi_lock(dev_input, FeedbackChannel.subdevice);
      m = comedi_data_read(dev_input, FeedbackChannel.subdevice, FeedbackChannel.channel, AI_RANGE, AREF_DIFF, &data_from_card);
      comedi_unlock(dev_input, FeedbackChannel.subdevice);

      //Convert to a voltage reading
      FeedbackReading = ((((float) data_from_card)/MaxInputBits)*InputRange + MinInputVoltage);
      //rt_printk("Data from card is %d \n", data_from_card);
      //rt_printk("Feedback reading is %f \n", FeedbackReading);
      //rt_printk("Input m is %d \n", m);
      delta = (FeedbackReading - AveragedFeedbackReading);
      //AveragedFeedbackReading = alpha*FeedbackReading+(1-alpha)*AveragedFeedbackReading;  //running averange
      //PIDOutputVariance = alpha*(delta*delta) + (1-alpha)*PIDOutputVariance;
      Error = AmplifierGainSign*OutputPhase*(SetPoint - FeedbackReading);//multiply by OutputPhase+AmplifierGainSign
      Pcontrib = PropCoff*(Error - LastError);
      //Not sure of sign of second contribution in line below...should it be - ?
      Dcontrib = ad*LastDiffContrib - bd*(Error - 2*LastError + SecondLastError);
      v = LastOutput + Pcontrib + Icontrib + Dcontrib;

      //next, take care of saturation of the output....anti-windup
      PIDOutput = v;
      PIDOutput =(PIDOutput>MaxOutputVoltage)? MaxOutputVoltage:PIDOutput;
      PIDOutput =(PIDOutput<MinOutputVoltage)? MinOutputVoltage:PIDOutput;

      //Calculate the averaged quantities
      pop_queue(&FeedbackReading_queue, &last_mean);
      AveragedFeedbackReading += (FeedbackReading - last_mean)/PID_averages;
      push_queue(&FeedbackReading_queue, FeedbackReading);

      pop_queue(&FeedbackReadingVar_queue, &last_var);
      new_var = delta*delta;
      PIDOutputVariance += (new_var - last_var)/PID_averages;
      push_queue(&FeedbackReadingVar_queue, new_var);

      //send the control signal
      //rt_printk("FeedbackReading is %f \n", FeedbackReading);
      //rt_printk("v is %f \n", v);
      //rt_printk("PID output should be %f \n", PIDOutput);
      data_to_card = (lsampl_t) nearbyint(((PIDOutput - MinOutputVoltage)/OutputRange)*MaxOutputBits);
      //data_to_card = (lsampl_t) 0;
      comedi_lock(dev_output, ControlChannel.subdevice);
      m=comedi_data_write(dev_output, ControlChannel.subdevice, ControlChannel.channel, AO_RANGE, AREF_DIFF, data_to_card);
      comedi_unlock(dev_output, ControlChannel.subdevice);
      //rt_printk("Output m is %d \n", m);

      //Update the integral contribution after the loop
      Icontrib = bi*Error;

      //Update parameters
      LastError = Error;
      SecondLastError = LastError;
      LastDiffContrib = Dcontrib;
      LastOutput = PIDOutput;


      //rt_printk("PContrib is %f \n", Pcontrib);
      //rt_printk("IContrib is %f \n", Icontrib);
      //rt_printk("DContrib is %f \n", Dcontrib);
      //rt_printk("PIDOutput is %f \n", PIDOutput);

      //Next part is to take the averaged PID output for recording if j>PID_averages and PID_averages>1
      //SummedPIDOutput+=PIDOutput;
      //SummedFeedbackReading += FeedbackReading;
      //j++;
      //AveragedPIDOutput=((PID_averages>1)&&(j>PID_averages))?(SummedPIDOutput/PID_averages):AveragedPIDOutput;
      //AveragedFeedbackReading=((PID_averages>1)&&(j>PID_averages))?(SummedFeedbackReading/PID_averages):AveragedFeedbackReading;
      //SummedPIDOutput=(j>PID_averages)? 0:SummedPIDOutput;
      //SummedFeedbackReading=(j>PID_averages)? 0:SummedFeedbackReading;
      //j=(j>PID_averages)? 1:j;

      //Calculate moving exponential averages and variance
      //delta = PIDOutput - AveragedPIDOutput;
      //AveragedPIDOutput = alpha*PIDOutput + (1-alpha)*AveragedPIDOutput;
      //PIDOutputVariance = alpha*(delta*delta) + (1-alpha)*PIDOutputVariance;
      //PIDOutputVariance = alpha*abs(delta) + (1-alpha)*PIDOutputVariance;

      pop_queue(&PIDOutput_queue, &last_mean);
      AveragedPIDOutput += (PIDOutput - last_mean)/PID_averages;
      push_queue(&PIDOutput_queue, PIDOutput);
         // And wait until the end of the period.
        rt_task_wait_period();

       }

    //rt_printk("Got here 3 \n");
    //rt_printk("pid_loop_running is %d \n", pid_loop_running);
    rt_make_soft_real_time();
    comedi_close(dev_input);
    comedi_close(dev_output);
    rt_task_delete(PIDloop_Task); //Self termination at end.

    pthread_exit(NULL);
    return 0;


}

