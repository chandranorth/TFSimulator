unit DAQFunctions;

{$MODE Delphi}

interface
  uses GlobalVariables, daq_comedi_types;

  var
    ScanXTaskHandle,
    ScanYTaskHandle,
    ScanZTaskHandle,
    ScanXYTaskHandle,
    Ch0TaskHandle,
    Ch1TaskHandle,
    Ch2TaskHandle,
    FeedbackInputTaskHandle
                        : longint;

procedure InitializeBoards;


implementation
  uses daq_comedi_functions,  Forms, rtai_comedi_types;


{-----------------------------------------------------------}
procedure DisplayErrorMessage(TheMessage:string);
  begin
    Application.MessageBox(PChar(TheMessage+'.  Configure system!'), 'Error!', 0);
  end;

{-------------------------------------------------------------------------------}
procedure InitializeBoards;

begin
  //Create the three output channels
   get_board_names(board_it,device_names, board_names);
end;


end.
