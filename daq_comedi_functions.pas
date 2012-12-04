unit daq_comedi_functions;

{$mode objfpc}{$H+}
{$linklib libcomedi.a}
//{$linklib libkcomedilxrt.a}

interface

uses
  Classes, SysUtils, daq_comedi_types, GlobalVariables;

//functions that are defined in the c header files, hence are labeled external
//No implementation part
  const
      MaxOutputBits = 65536;
      MaxInputBits = 65536;
      MaxOutputVoltage = 9.995; //in volts;
      OutputRange = 20; //in volts;
      InputRange = 20; //in volts
      MaxInputVoltage = 9.995; //in volts
      MinInputVoltage = -10.0;
      MinOutputVoltage = -10.0;

  function comedi_to_real(data: Plsampl_t): double;

  function real_to_comedi(output: double): Plsampl_t;

  function comedi_get_maxdata(device : Pcomedi_t; subdevice : dword; channel: dword): lsampl_t; cdecl; external;

  function comm_check(t_number: integer): integer; cdecl; external;

  function get_board_name(dev_identifier:PChar): PChar; cdecl; external;

  function comedi_open(filename : PChar): Pcomedi_t; cdecl; external;

  function comedi_close(it : PComedi_t): longint; cdecl; external;

  function comedi_get_n_subdevices(it: Pcomedi_t):   longint; cdecl; external;

  function comedi_get_subdevice_type(it : Pcomedi_t; subdevice: dword): longint; cdecl; external;

  function comedi_get_n_channels(it: Pcomedi_t; subdevice: dword): longint; cdecl; external;

  function comedi_data_read(it: Pcomedi_t;  subdevice: dword; channel: dword; range : dword; aref : dword; data : Plsampl_t): longint; cdecl; external;

  function comedi_data_write(it: Pcomedi_t; subdevice: dword; channel: dword; range: dword; aref : dword; data : lsampl_t): longint; cdecl; external;


//functions that are defined in Pascal
  function get_board_names(var board_it : board_it_type; device_names:device_name_type; var board_names: device_name_type): boolean;

implementation

  function get_board_names(var board_it : board_it_type; device_names:device_name_type; var board_names: device_name_type): boolean;
    var i : integer;

    begin
      for i:=0 to MaxBoards -1 do
        begin
          board_it[i]:= nil;
          board_it[i]:=comedi_open(device_names[i]);
          if board_it[i]<>nil then board_names[i]:=board_it[i]^.devinfo.board_name;
        end;
      get_board_names:=TRUE;
    end;
  function comedi_to_real(data: Plsampl_t): double;
    begin
        comedi_to_real:=(integer(data^)/MaxInputBits)*InputRange + MinInputVoltage;
    end;

  function real_to_comedi(output: double): Plsampl_t;
    var
      Calc_output : Plsampl_t;
    begin
       new(Calc_output);
       Calc_output^:= lsampl_t(round(((output-MinOutputVoltage)/OutputRange)*MaxOutputBits));
       real_to_comedi := Calc_output;
    end;

end.

