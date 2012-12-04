program TuningForkSimulator;
{$linklib stdc++}
{$MODE Delphi}
//{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Interfaces, // this includes the LCL widgetset
  Forms, MainForm, globalfunctions, globalvariables, rtai_types, rtai_functions,
  rtai_comedi_types, rtai_comedi_functions, DAQFunctions, daq_comedi_types,
  daq_comedi_functions, etpackage, LResources, TAChartLazarusPkg
  { you can add units after this };

{$IFDEF WINDOWS}{$R TuningForkSimulator.rc}{$ENDIF}

begin
  {$I TuningForkSimulator.lrs}
  Application.Initialize;
  Application.CreateForm(TTuningForkForm, TuningForkForm);
  Application.Run;
end.

