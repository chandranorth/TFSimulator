//header file for the real time programs for the SPM C library

typedef struct  {
    //This is to define data acquisition channels for easy access
    char *devicename;                       //Definition corresponds to Pascal DAQ-Comedi-types
    char *boardname;
    comedi_t *board_id;
    int board_number;
    int subdevice;
    int channel;
}  Acquisition_channel;

volatile RTIME sample_time;

volatile Acquisition_channel AnalogOutputChannel; //Channel for a test sine wave outpu
volatile Acquisition_channel AnalogInputChannel; //Channel for a test sine wave outpu
volatile int hard_timer_running =1; //flag to check if hard timer is running.  Set initially to FALSE (1, not 0)
static RTIME sampling_interval;
volatile int AmplifierGainSign =-1;//Sign of the Amplifier Gain on the z axis
volatile double MaxZVoltage =-10;//manan changed it so that Max is 10 and Min is -10
volatile double MinZVoltage =10;
//volatile double CurrentZVoltage;


//PID parameters
volatile Acquisition_channel FeedbackChannel; //Channel for reading the input signal
volatile Acquisition_channel ControlChannel; //Channel for controlling the system
volatile int PIDParametersChanged =1; //Flag to signal that PID parameters have changed
volatile double PropCoff; //PID proportional coefficient
volatile double IntTime; //PID integral time constant
volatile double DiffTime; //PID differential time constant
RTIME PIDLoop_Time;
volatile int PID_averages =1; //number of averages to perform
volatile int pid_loop_running = 1;
volatile double PIDOutput; //output in volts of the PID
volatile double AveragedPIDOutput; //PID output averaged over PID_averagess
volatile double AveragedFeedbackReading; //Averaged Feedback reading
volatile double FeedbackReading;
volatile double SetPoint; //SetPoint for the PID
volatile int FirstPIDPass =1 ; //this tells us if this is the first pass, set to FALSE
volatile int OutputPhase = -1; //Sets the sign of the output, depending on response
volatile double PIDOutputVariance; //variance of the pid output


//RTAI test definitions
double SineWaveFrequency;
double SineWaveAmplitude;
int N_PNT;
volatile double OutputValue;
#define RUN_TIME  5
#define PI 3.14156

//definitions for TF loop
int tf_loop_running;
volatile Acquisition_channel InputChannel; //Channel for reading the control signal
volatile Acquisition_channel OutputChannel; //Channel for sending out the response
volatile Acquisition_channel XChannel; //Channel that determines the scan x position
volatile Acquisition_channel YChannel; //Channel that determines the scan y position
volatile int TFParametersChanged =1; //Flag to signal that TF parameters have changed
volatile int TestControlSignal = 1; //Set to False if using InputChannel to determine distance
volatile int ExternalScan =1; //Set to False if not scanning
volatile double LJ_FreqShiftMultFact; //Factor to multiply to calculate frequency shift, essentially 12A
volatile double x_06; //x_0 raised to the power 6, calculated by main program
volatile double ControlMultFact; //Factor to multiply control signal by, to convert it to nm
volatile double ControlAddFact; //Factor to add to control signal
volatile double TubeDisplacement; //Resulting displacement of tube in nm
volatile double OutputConversionFactor; //factor to convert frequency shift into voltage
volatile double OutputAddFactor; //additional offset to add to
volatile double FeatureHeight; //height of topographic features, in nm
volatile double FeatureSize; //Size of feature in units of volts on x and y scans (think of as microns)
volatile double FeatureCenter; //Center of the feature, in microns (see note in line above)
volatile double FeaturePeriod = 1; //Period of the feature, in volts (microns)
volatile double XPosition; //X scan position, in volts (read as microns)
volatile double YPosition; //Y scan position
volatile double XHeight; //actual current height
volatile double FrequencyShift; //resulting frequency shift

#define min_delta_x 0.1  // minimum value of height in nm, in order to avoid blow up of LJ potential



//Hardware related paramters
#define AO_RANGE  0
#define AI_RANGE 0
char *device_names[2] = {"/dev/comedi0", "/dev/comedi1"};
int MaxOutputBits = 65535;  //changed from 65536
int MaxInputBits = 65535;
double MaxOutputVoltage = 10.0; //in volts;
double OutputRange = 20.0; //in volts;
double InputRange = 20.0; //in volts
double MaxInputVoltage = 10.0; //in volts
double MinInputVoltage = -10.0;
double MinOutputVoltage = -10.0;


volatile float SAMP_FREQ;
RTIME SAMP_TIME;
#define TRIGSAMP  2048

int sine_loop_running;


//functions defined in queue.c

//functions that can be called;
double comedi_to_real(lsampl_t *data);
lsampl_t *real_to_comedi(double output);
int sineoutput();
int StartMainTask(int priority);
int EndMainTask();
int pid_loop();
int tf_loop();
