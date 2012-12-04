unit globalvariables;

{$MODE Delphi}

interface
 uses
   Forms, daq_comedi_types, Classes {, Windows};


const
  MaxBoards = 3; //maximum number of data acquisition boards
  MaxChannels = 32; //Maximum number of input or output channels
  MaxDevTypes =13;  //maximum number of subdevice types in comedi (currently)
  MaxDAQVoltage=10.0;  // Max and Min voltage output of D/A cards in
  MinDAQVoltage=-10.0;  //volts
  DAQVoltageRange=20.0; //in volts
  MaxBits = 65535; //resolution of boards in bits
  MaxFeedbackCorrection = DAQVoltageRange/50;  //maximum correction term


type
   Axis      = (XAxis, YAxis, ZAxis);
   ErrorSign = (Positive, negative);
   board_it_type = array[0..MaxBoards-1] of Pcomedi_t;
   device_name_type = array[0..MaxBoards-1] of PChar;
   Channel_array = array[0..MaxChannels-1] of Acquisition_channel;


var
//General variables
  GlobalTaskStarted     : boolean = FALSE;

//Variables associated with the DAQ cards
  BoardsPresent       : boolean = FALSE;
  InputChannelName        : string = 'NI6014/ai0';
  OutputChannelName       : string = 'NI6014/ao0';
  XChannelName            : string = 'NI6014/ai1';
  YChannelName            : string = 'NI6733/ai2';

  //Now the actual channels
  InputChannel,
  OutputChannel,
  XChannel,
  YChannel
                           : Acquisition_channel;

  board_it        : board_it_type;
  device_names    : device_name_type;   //filenames of the boards, e.g., /dev/comedi0
  board_names     : device_name_type;  //actual descriptions of the boards
  InputChannels   : Channel_array;
  OutputChannels  : Channel_array;


  StepDelay                    : real = 1.0; //delay between steps in milliseconds



  FeedBackSign            : integer = 1;
  FreqOutputScaleFact     : real;
  FreqShift               : real; //self evident
  InitialFrequency        : real; //Center frequency of the PLL when far from the surface
  CenterFrequency         : real; //current center frequency
  Frequency               : real; //actual frequency
  Noise                   : real = 0.2; //noise level for readings in volts, ignore changes if below this



  //Feedback Timer parameters
  FirstFeedbackRun        : boolean = TRUE;
  INFeedback              : boolean = FALSE;
  startTime64,
  endTime64,
  frequency64             : Int64;  //for use with the getperformance functions
  elapsedSeconds          : single;



implementation

initialization
   device_names[0]:='/dev/comedi0';
   device_names[1]:='/dev/comedi1';
   device_names[2]:='/dev/comedi2';
   //New(board_it[2]);
   board_it[2]:=nil;
   //New(board_it[0]);
   board_it[0]:= nil;
   //New(board_it[1]);
   board_it[1]:=nil;



finalization
   //Dispose(board_it[0]);
   //Dispose(board_it[1]);
   //Dispose(board_it[2]);

end.
