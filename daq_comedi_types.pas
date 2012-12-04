unit daq_comedi_types;
{This is an interface file to go between Pascal and the comedi and rtai header
files.  Instead of translating the entire headers using h2pas or something
equivalent (which give a lot of errors), I have chosen to translate only those
functions and type definitions as needed, copying them from comedi_short.pas
and comedilib_short.pas.

Last modification November 24, 2010}
{$mode objfpc}{$H+}
//{$link daq_interface.o}   legacy, not used
{$linklib libcomedi.a}
//{$linklib libkcomedilxrt.a}

interface

uses
  Classes, SysUtils;


const
     COMEDI_MAJOR = 98;
  {
     maximum number of minor devices.  This can be increased, although
     kernel structures are currently statically allocated, thus you
     don't want this to be much more than you actually use.
    }
     COMEDI_NDEVICES = 16;
  { number of config options in the config structure  }
     COMEDI_NDEVCONFOPTS = 32;
     COMEDI_NAMELEN = 20;

var
     subdevice_types : array[0..12] of string; //this defines an array of type definitions
                       //returned by the comedi_get_subdevice function


const
     CR_FLAGS_MASK = $fc000000;
     CR_ALT_FILTER = 1 shl 26;
     CR_DITHER = CR_ALT_FILTER;
     CR_DEGLITCH = CR_ALT_FILTER;
     CR_ALT_SOURCE = 1 shl 27;
     CR_EDGE = 1 shl 30;
     CR_INVERT = 1 shl 31;
  { analog ref = analog ground  }
     AREF_GROUND = $00;
  { analog ref = analog common  }
     AREF_COMMON = $01;
  { analog ref = differential  }
     AREF_DIFF = $02;
  { analog ref = other (undefined)  }
     AREF_OTHER = $03;
  { counters -- these are arbitrary values  }
     GPCT_RESET = $0001;
     GPCT_SET_SOURCE = $0002;
     GPCT_SET_GATE = $0004;
     GPCT_SET_DIRECTION = $0008;
     GPCT_SET_OPERATION = $0010;
     GPCT_ARM = $0020;
     GPCT_DISARM = $0040;
     GPCT_GET_INT_CLK_FRQ = $0080;
     GPCT_INT_CLOCK = $0001;
     GPCT_EXT_PIN = $0002;
     GPCT_NO_GATE = $0004;
     GPCT_UP = $0008;
     GPCT_DOWN = $0010;
     GPCT_HWUD = $0020;
     GPCT_SIMPLE_EVENT = $0040;
     GPCT_SINGLE_PERIOD = $0080;
     GPCT_SINGLE_PW = $0100;
     GPCT_CONT_PULSE_OUT = $0200;
     GPCT_SINGLE_PULSE_OUT = $0400;
  { instructions  }
     INSN_MASK_WRITE = $8000000;
     INSN_MASK_READ = $4000000;
     INSN_MASK_SPECIAL = $2000000;
     INSN_READ = 0 or INSN_MASK_READ;
     INSN_WRITE = 1 or INSN_MASK_WRITE;
     INSN_BITS = (2 or INSN_MASK_READ) or INSN_MASK_WRITE;
     INSN_CONFIG = (3 or INSN_MASK_READ) or INSN_MASK_WRITE;
     INSN_GTOD = (4 or INSN_MASK_READ) or INSN_MASK_SPECIAL;
     INSN_WAIT = (5 or INSN_MASK_WRITE) or INSN_MASK_SPECIAL;
     INSN_INTTRIG = (6 or INSN_MASK_WRITE) or INSN_MASK_SPECIAL;
  { trigger flags  }
  { These flags are used in comedi_trig structures  }
  { do the motions  }
     TRIG_BOGUS = $0001;
  { enable dithering  }
     TRIG_DITHER = $0002;
  { enable deglitching  }
     TRIG_DEGLITCH = $0004;
  {#define TRIG_RT       0x0008          /* perform op in real time */ }
  { perform configuration, not triggering  }
     TRIG_CONFIG = $0010;
  { wake up on end-of-scan events  }
     TRIG_WAKE_EOS = $0020;
  {#define TRIG_WRITE    0x0040          /* write to bidirectional devices */ }
  { command flags  }
  { These flags are used in comedi_cmd structures  }
  { try to use a real-time interrupt while performing command  }
     CMDF_PRIORITY = $00000008;
  { compatibility definition  }
     TRIG_RT = CMDF_PRIORITY;
     CMDF_WRITE = $00000040;
  { compatibility definition  }
     TRIG_WRITE = CMDF_WRITE;
     CMDF_RAWDATA = $00000080;
     COMEDI_EV_START = $00040000;
     COMEDI_EV_SCAN_BEGIN = $00080000;
     COMEDI_EV_CONVERT = $00100000;
     COMEDI_EV_SCAN_END = $00200000;
     COMEDI_EV_STOP = $00400000;
     TRIG_ROUND_MASK = $00030000;
     TRIG_ROUND_NEAREST = $00000000;
     TRIG_ROUND_DOWN = $00010000;
     TRIG_ROUND_UP = $00020000;
     TRIG_ROUND_UP_NEXT = $00030000;
  { trigger sources  }
     TRIG_ANY = $ffffffff;
     TRIG_INVALID = $00000000;
  { never trigger  }
     TRIG_NONE = $00000001;
  { trigger now + N ns  }
     TRIG_NOW = $00000002;
  { trigger on next lower level trig  }
     TRIG_FOLLOW = $00000004;
  { trigger at time N ns  }
     TRIG_TIME = $00000008;
  { trigger at rate N ns  }
     TRIG_TIMER = $00000010;
  { trigger when count reaches N  }
     TRIG_COUNT = $00000020;
  { trigger on external signal N  }
     TRIG_EXT = $00000040;
  { trigger on comedi-internal signal N  }
     TRIG_INT = $00000080;
  { driver defined  }
     TRIG_OTHER = $00000100;
  { subdevice flags  }
  { device is busy  }
     SDF_BUSY = $0001;
  { device is busy with your job  }
     SDF_BUSY_OWNER = $0002;
  { subdevice is locked  }
     SDF_LOCKED = $0004;
  { you own lock  }
     SDF_LOCK_OWNER = $0008;
  { maxdata depends on channel  }
     SDF_MAXDATA = $0010;
  { flags depend on channel  }
     SDF_FLAGS = $0020;
  { range type depends on channel  }
     SDF_RANGETYPE = $0040;
  { can do mode 0  }
     SDF_MODE0 = $0080;
  { can do mode 1  }
     SDF_MODE1 = $0100;
  { can do mode 2  }
     SDF_MODE2 = $0200;
  { can do mode 3  }
     SDF_MODE3 = $0400;
  { can do mode 4  }
     SDF_MODE4 = $0800;
  { can do commands (deprecated)  }
     SDF_CMD = $1000;
  { subdevice uses software calibration  }
     SDF_SOFT_CALIBRATED = $2000;
  { can do output commands  }
     SDF_CMD_WRITE = $4000;
  { can do input commands  }
     SDF_CMD_READ = $8000;
  { subdevice can be read (e.g. analog input)  }
     SDF_READABLE = $00010000;
  { subdevice can be written (e.g. analog output)  }
     SDF_WRITABLE = $00020000;
  { spelling error in API  }
     SDF_WRITEABLE = SDF_WRITABLE;
  { subdevice does not have externally visible lines  }
     SDF_INTERNAL = $00040000;
  { DEPRECATED: subdevice is RT capable  }
     SDF_RT = $00080000;
  { can do aref=ground  }
     SDF_GROUND = $00100000;
  { can do aref=common  }
     SDF_COMMON = $00200000;
  { can do aref=diff  }
     SDF_DIFF = $00400000;
  { can do aref=other  }
     SDF_OTHER = $00800000;
  { can do dithering  }
     SDF_DITHER = $01000000;
  { can do deglitching  }
     SDF_DEGLITCH = $02000000;
  { can do mmap()  }
     SDF_MMAP = $04000000;
  { subdevice is acquiring data  }
     SDF_RUNNING = $08000000;
  { subdevice uses 32-bit samples  }
     SDF_LSAMPL = $10000000;
  { subdevice can do packed DIO  }
     SDF_PACKED = $20000000;
  { re recyle these flags for PWM  }
  { PWM can automatically switch off  }
     SDF_PWM_COUNTER = SDF_MODE0;
  { PWM is signed (H-bridge)  }


  type
     comedi_subdevice_type = (COMEDI_SUBD_UNUSED,COMEDI_SUBD_AI,COMEDI_SUBD_AO,
       COMEDI_SUBD_DI,COMEDI_SUBD_DO,COMEDI_SUBD_DIO,
       COMEDI_SUBD_COUNTER,COMEDI_SUBD_TIMER,
       COMEDI_SUBD_MEMORY,COMEDI_SUBD_CALIB,
       COMEDI_SUBD_PROC,COMEDI_SUBD_SERIAL,
       COMEDI_SUBD_PWM);

  { configuration instructions  }
  {      INSN_CONFIG_WAVEFORM = 17, }
  {      INSN_CONFIG_TRIG = 18, }
  {      INSN_CONFIG_COUNTER = 19, }
  {ALPHA }  { Use CTR as single pulsegenerator }
  { Use CTR as pulsetraingenerator }
  { Use the counter as encoder }
  { Set gate source }
  { Get gate source }
  { Set master clock source }
  { Get master clock source }
  { Set other source }
  {      INSN_CONFIG_GET_OTHER_SRC = 2006,       // Get other source }
  { Get size in bytes of subdevice's on-board fifos used during streaming input/output }
  { deprecated  }
  { PWM  }
  { sets frequency  }
  { gets frequency  }
  { is it running?  }
  { sets H bridge: duty cycle and sign bit for a relay  at the same time }
  { gets H bridge data: duty cycle and the sign bit  }
     configuration_ids = (INSN_CONFIG_DIO_INPUT := 0,INSN_CONFIG_DIO_OUTPUT := 1,
       INSN_CONFIG_DIO_OPENDRAIN := 2,INSN_CONFIG_ANALOG_TRIG := 16,
       INSN_CONFIG_ALT_SOURCE := 20,INSN_CONFIG_DIGITAL_TRIG := 21,
       INSN_CONFIG_BLOCK_SIZE := 22,INSN_CONFIG_TIMER_1 := 23,
       INSN_CONFIG_FILTER := 24,INSN_CONFIG_CHANGE_NOTIFY := 25,
       INSN_CONFIG_SERIAL_CLOCK := 26,INSN_CONFIG_BIDIRECTIONAL_DATA := 27,
       INSN_CONFIG_DIO_QUERY := 28,INSN_CONFIG_PWM_OUTPUT := 29,
       INSN_CONFIG_GET_PWM_OUTPUT := 30,INSN_CONFIG_ARM := 31,
       INSN_CONFIG_DISARM := 32,INSN_CONFIG_GET_COUNTER_STATUS := 33,
       INSN_CONFIG_RESET := 34,INSN_CONFIG_GPCT_SINGLE_PULSE_GENERATOR := 1001,
       INSN_CONFIG_GPCT_PULSE_TRAIN_GENERATOR := 1002,
       INSN_CONFIG_GPCT_QUADRATURE_ENCODER := 1003,
       INSN_CONFIG_SET_GATE_SRC := 2001,INSN_CONFIG_GET_GATE_SRC := 2002,
       INSN_CONFIG_SET_CLOCK_SRC := 2003,INSN_CONFIG_GET_CLOCK_SRC := 2004,
       INSN_CONFIG_SET_OTHER_SRC := 2005,INSN_CONFIG_GET_HARDWARE_BUFFER_SIZE,
       INSN_CONFIG_SET_COUNTER_MODE := 4097,
       INSN_CONFIG_8254_SET_MODE := INSN_CONFIG_SET_COUNTER_MODE,INSN_CONFIG_8254_READ_STATUS := 4098,
       INSN_CONFIG_SET_ROUTING := 4099,INSN_CONFIG_GET_ROUTING := 4109,
       INSN_CONFIG_PWM_SET_PERIOD := 5000,INSN_CONFIG_PWM_GET_PERIOD := 5001,
       INSN_CONFIG_GET_PWM_STATUS := 5002,INSN_CONFIG_PWM_SET_H_BRIDGE := 5003,
       INSN_CONFIG_PWM_GET_H_BRIDGE := 5004
       );

     comedi_io_direction = (COMEDI_INPUT := 0,COMEDI_OUTPUT := 1,
       COMEDI_OPENDRAIN := 2);

     comedi_support_level = (COMEDI_UNKNOWN_SUPPORT := 0,COMEDI_SUPPORTED,
       COMEDI_UNSUPPORTED);

type

  Plsampl_t = ^lsampl_t;
  lsampl_t = dword;

  Psampl_t = ^sampl_t;
  sampl_t = word;


     Pcomedi_chaninfo_struct = ^comedi_chaninfo_struct;
     comedi_chaninfo_struct = record
          subdev : dword;
          maxdata_list : Plsampl_t;
          flaglist : Pdword;
          rangelist : Pdword;
          unused : array[0..3] of dword;
       end;
     Pcomedi_chaninfo  = ^comedi_chaninfo;
     comedi_chaninfo   = comedi_chaninfo_struct;

     Pcomedi_rangeinfo_struct = ^comedi_rangeinfo_struct;
     comedi_rangeinfo_struct = record
          range_type : dword;
          range_ptr : pointer;
       end;
     Pcomedi_rangeinfo  = ^comedi_rangeinfo;
     comedi_rangeinfo   = comedi_rangeinfo_struct;

// comedi_range_structure...again not found

//  typedef struct comedi_range_struct comedi_range;

//struct comedi_range_struct{
//  double min;
//  double max;
//  unsigned int unit;
//}comedi_range;

     Pcomedi_range_struct = ^comedi_range_struct;
     comedi_range_struct  = record
           min   : double;
           max   : double;
           units  : dword;    //note, changed from "unit" as this is a keyword
         end;
     Pcomedi_range = ^comedi_range;
     comedi_range  = comedi_range_struct;



  { fixed point, multiply by 1e-6  }
  { fixed point, multiply by 1e-6  }
     Pcomedi_krange_struct = ^comedi_krange_struct;
     comedi_krange_struct = record
          min : longint;
          max : longint;
          flags : dword;
       end;
     Pcomedi_krange = ^comedi_krange;
     comedi_krange  = comedi_krange_struct;

  { channel flags  }
  { lookup in kernel  }
  { see support_level enum for values }
     Pcomedi_subdinfo_struct = ^comedi_subdinfo_struct;
     comedi_subdinfo_struct = record
          _type : dword;
          n_chan : dword;
          subd_flags : dword;
          timer_type : dword;
          len_chanlist : dword;
          maxdata : lsampl_t;
          flags : dword;
          range_type : dword;
          settling_time_0 : dword;
          insn_bits_support : dword;
          unused : array[0..7] of dword;
       end;
     Pcomedi_subdinfo = ^comedi_subdinfo;
     comedi_subdinfo  = comedi_subdinfo_struct;

     Pcomedi_devinfo_struct = ^comedi_devinfo_struct;
     comedi_devinfo_struct = record
          version_code : dword;
          n_subdevs : dword;
          driver_name : array[0..(COMEDI_NAMELEN)-1] of char;
          board_name : array[0..(COMEDI_NAMELEN)-1] of char;
          read_subdevice : longint;
          write_subdevice : longint;
          unused : array[0..29] of longint;
       end;
     Pcomedi_devinfo  = ^comedi_devinfo;
     comedi_devinfo   = comedi_devinfo_struct;

     Pcomedi_devconfig_struct = ^comedi_devconfig_struct;
     comedi_devconfig_struct = record
          board_name : array[0..(COMEDI_NAMELEN)-1] of char;
          options : array[0..(COMEDI_NDEVCONFOPTS)-1] of longint;
       end;
     Pcomedi_devconfig  = ^comedi_devconfig;
     comedi_devconfig   = comedi_devconfig_struct;

     Pcomedi_bufconfig_struct = ^comedi_bufconfig_struct;
     comedi_bufconfig_struct = record
          subdevice : dword;
          flags : dword;
          maximum_size : dword;
          size : dword;
          unused : array[0..3] of dword;
       end;
       Pcomedi_bufconfig = ^comedi_bufconfig;
       comedi_bufconfig = comedi_bufconfig_struct;

     Pcomedi_bufinfo_struct = ^comedi_bufinfo_struct;
     comedi_bufinfo_struct = record
          subdevice : dword;
          bytes_read : dword;
          buf_write_ptr : dword;
          buf_read_ptr : dword;
          buf_write_count : dword;
          buf_read_count : dword;
          bytes_written : dword;
          unused : array[0..3] of dword;
       end;
     Pcomedi_bufinfo = ^comedi_bufinfo;
     comedi_bufinfo  = comedi_bufinfo_struct;

  { channel/range list  }
  { data list, size depends on subd flags  }
     Pcomedi_cmd_struct = ^comedi_cmd_struct;
     comedi_cmd_struct = record
          subdev : dword;
          flags : dword;
          start_src : dword;
          start_arg : dword;
          scan_begin_src : dword;
          scan_begin_arg : dword;
          convert_src : dword;
          convert_arg : dword;
          scan_end_src : dword;
          scan_end_arg : dword;
          stop_src : dword;
          stop_arg : dword;
          chanlist : Pdword;
          chanlist_len : dword;
          data : Psampl_t;
          data_len : dword;
       end;
     Pcomedi_cmd = ^comedi_cmd;
     comedi_cmd  = comedi_cmd_struct;

  //subdevice_struct...again not defined in headers
 // typedef struct subdevice_struct subdevice;

//struct subdevice_struct{
//  unsigned int type;
//  unsigned int n_chan;
//  unsigned int subd_flags;
//  unsigned int timer_type;
//  unsigned int len_chanlist;
//  lsampl_t maxdata;
//  unsigned int flags;
//  unsigned int range_type;

//  lsampl_t *maxdata_list;
//  unsigned int *range_type_list;
//  unsigned int *flags_list;

//  comedi_range *rangeinfo;
//  ccomedi_range **rangeinfo_list;

//  unsigned int has_cmd;
//  unsigned int has_insn_bits;

//  int cmd_mask_errno;
//  comedi_cmd *cmd_mask;
//  int cmd_timed_errno;
//  comedi_cmd *cmd_timed;
//};
  Psubdevice_struct = ^subdevice_struct;
  subdevice_struct = record
    device_type    : dword; //note name has changed from c variable name!
    n_chan         : dword;
    subd_flags     : dword;
    timer_type     : dword;
    len_chanlist   : dword;
    maxdata        : lsampl_t;
    flags          : dword;
    range_type     : dword;
    maxdata_list   : Plsampl_t;
    range_type_list: Pdword;
    flags_list     : Pdword;
    range_info     : Pcomedi_range;
    rangeinfolist  : ^Pcomedi_range;
    has_cmd        : dword;
    has_insn_bits  : dword;
    cmd_mask_errno : longint;
    cmd_mask       : Pcomedi_cmd;
    cmd_timed_errno: longint;
    cmd_timed      :  Pcomedi_cmd;
  end;
  Psubdevice  = ^subdevice;
  subdevice   = subdevice_struct;

//typedef struct comedi_t_struct comedi_t;

//struct comedi_t_struct{
//  int magic;        // driver-specific magic number, for identification
//  int fd;           // file descriptor, for open() and close()
//  int n_subdevices; // number of subdevices on this device
//  comedi_devinfo devinfo;
//  subdevice *subdevices; // pointer to subdevice list
//                         // filled in automatically at load time
//  unsigned int has_insnlist_ioctl; // can process instruction lists
//  unsigned int has_insn_ioctl;     // can process instructions
//};


  Pcomedi_t_struct  = ^comedi_t_struct;
  comedi_t_struct   = record
      magic                 : longint;
      fd                    : longint;
      n_subdevices          : longint;
      devinfo               : comedi_devinfo;
      subdevices            : Psubdevice;
      has_insnlist_ioctl    : dword;
      has_insn_ioctl        : dword;
  end;

  PPcomedi_t   = ^Pcomedi_t;
  Pcomedi_t    = ^comedi_t;
  comedi_t = comedi_t_struct;

type        //This is to define data acquisition channels for easy access
  Acquisition_channel = record
    devicename        : PChar;
    boardname         : PChar;
    board_id          : Pcomedi_t;
    board_number      : longint;
    subdevice         : longint;
    channel           : longint;
  end;


implementation

initialization

subdevice_types[0]:= 'unused';
subdevice_types[1]:= 'ai';
subdevice_types[2]:= 'ao';
subdevice_types[3]:= 'di';
subdevice_types[4]:= 'do';
subdevice_types[5]:= 'dio';
subdevice_types[6]:='counter';
subdevice_types[7]:='timer';
subdevice_types[8]:='memory';
subdevice_types[9]:='calib';
subdevice_types[10]:='proc';
subdevice_types[11]:='serial';
subdevice_types[12]:='pwm' ;
end.

