unit globalfunctions;

{$MODE Delphi}

interface
  uses Forms, {Windows,} GlobalVariables, Controls, LCLIntf, Graphics, Extctrls;

{-----------------------------------------------------------------------------------------------------------}
procedure delay(msecs:dword);
procedure fastdelay(msecs:real);

implementation
  uses SysUtils, MainForm;


{-------------------------------------------------------------------------}
procedure delay(msecs:dword);
  var
    FirstTickCount:Dword;
  begin
    FirstTickCount:=GetTickCount;
    repeat
      Application.ProcessMessages; {allowing access to other
                                      controls, etc.}
    until ((GetTickCount-FirstTickCount) >= msecs);
  end;

{-----------------------------------------------------------------------}
procedure fastdelay(msecs: real);
  var
    OldTime :extended;
  begin
    with TuningForkForm.FastTimer do
      begin
        Clear;
        Start;
        OldTime:=Elapsed;
        while ((Elapsed - OldTime)*1000)<msecs do
          Application.ProcessMessages;
        Stop;
      end;
  end;



end.
