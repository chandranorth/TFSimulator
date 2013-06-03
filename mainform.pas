unit MainForm;

{$mode objfpc}{$H+}

interface

uses
  {$IFNDEF LCL}
  Windows, Messages,
  {$ELSE}
  LclIntf, LMessages, LclType,
  {$ENDIF}
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ExtCtrls, Spin, ComCtrls, EpikTimer, TAGraph, TASeries,
  daq_comedi_functions, DAQFunctions in 'DAQFunctions.pas',
  daq_comedi_types, GlobalVariables, rtai_comedi_functions, rtai_comedi_types,
  rtai_functions, rtai_types;

type

  { TTuningForkForm }

  TTuningForkForm = class(TForm)
    AmpGainSpinEdit: TSpinEdit;
    EFMCheckBox: TCheckBox;
    EFMGroupBox: TGroupBox;
    Label13: TLabel;
    TipRadiusSpinEdit: TFloatSpinEdit;
    TipVoltageSpinEdit: TFloatSpinEdit;
    Label4: TLabel;
    StartSimulatorBtn: TButton;
    FastTimer: TEpikTimer;
    OutputSignalOutputLabel: TLabel;
    InternalPositionGroupBox: TGroupBox;
    DataTimer: TTimer;
    YPositionSpinEdit: TFloatSpinEdit;
    ExternalScanCheckBox: TCheckBox;
    ControlSignalSpinEdit: TFloatSpinEdit;
    ArrayParameterGroupBox: TGroupBox;
    Label12: TLabel;
    HeightLabel: TLabel;
    Label7: TLabel;
    SizeofFeatureSpinEdit: TFloatSpinEdit;
    CenteredAtSpinEdit: TFloatSpinEdit;
    HeightSpinEdit: TFloatSpinEdit;
    Label10: TLabel;
    Label11: TLabel;
    Label2: TLabel;
    Label5: TLabel;
    XPositionSpinEdit: TFloatSpinEdit;
    XChannelIndicator: TLabel;
    XChannelComboBox: TComboBox;
    ScanningInputGroupBox: TGroupBox;
    Label1: TLabel;
    Memo5: TMemo;
    TestControlCheckBox: TCheckBox;
    TestControlSignalGroupBox: TGroupBox;
    InputControlComboBox: TComboBox;
    OutputChannelComboBox: TComboBox;
    FreqShiftLabel: TLabel;
    Label3: TLabel;
    Label9: TLabel;
    LabeledEdit1: TLabeledEdit;
    OutputConvFactEdit: TLabeledEdit;
    OutputAddVoltEdit: TLabeledEdit;
    LJPotGroupBox: TGroupBox;
    Memo4: TMemo;
    OutputTransducerGroupBox: TGroupBox;
    InputGroupBox: TGroupBox;
    ControlSignalOutputLabel: TLabel;
    TubeDisplacementLabel: TLabel;
    ResultDispLabel: TLabel;
    DeltaKLabel: TLabel;
    Label6: TLabel;
    Label8: TLabel;
    InputControlMultEdit: TLabeledEdit;
    InputSignalAddEdit: TLabeledEdit;
    MaxFreqShiftEdit: TLabeledEdit;
    LJPlotMinEdit: TLabeledEdit;
    TFForceConstEdit: TLabeledEdit;
    TFFreqEdit: TLabeledEdit;
    Memo1: TMemo;
    Memo2: TMemo;
    Memo3: TMemo;
    YChannelComboBox: TComboBox;
    YChannelIndicator: TLabel;
    procedure AmpGainSpinEditChange(Sender: TObject);
    procedure CenteredAtSpinEditChange(Sender: TObject);
    procedure ControlSignalSpinEditChange(Sender: TObject);
    procedure DataTimerTimer(Sender: TObject);
    procedure EFMCheckBoxClick(Sender: TObject);
    procedure ExternalScanCheckBoxClick(Sender: TObject);
    procedure HeightSpinEditChange(Sender: TObject);
    procedure InputControlComboBoxSelect(Sender: TObject);
    procedure InputControlMultEditKeyPress(Sender: TObject; var Key: char);
    procedure InputSignalAddEditKeyPress(Sender: TObject; var Key: char);
    procedure LJPlotMinEditKeyPress(Sender: TObject; var Key: char);
    procedure MaxFreqShiftEditKeyPress(Sender: TObject; var Key: char);
    procedure OutputAddVoltEditKeyPress(Sender: TObject; var Key: char);
    procedure OutputChannelComboBoxSelect(Sender: TObject);
    procedure OutputConvFactEditKeyPress(Sender: TObject; var Key: char);
    procedure SizeofFeatureSpinEditChange(Sender: TObject);
    procedure StartSimulatorBtnClick(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure TestControlCheckBoxClick(Sender: TObject);
    procedure TFForceConstEditKeyPress(Sender: TObject; var Key: char);
    procedure TFFreqEditKeyPress(Sender: TObject; var Key: char);
    procedure TipRadiusSpinEditChange(Sender: TObject);
    procedure TipVoltageSpinEditChange(Sender: TObject);
    procedure XChannelComboBoxSelect(Sender: TObject);
    procedure XPositionSpinEditChange(Sender: TObject);
    procedure YChannelComboBoxSelect(Sender: TObject);
    procedure YPositionSpinEditChange(Sender: TObject);
  private
    { private declarations }
  public
    { public declarations }
  end; 

var
  TuningForkForm: TTuningForkForm;

implementation
uses math;
{ TTuningForkForm }
var
   InOperation: boolean = FALSE;

   OutputChannelNames     : TStringList;
   InputChannelNames      : TStringList;
   SomethingChanged       : boolean=FALSE;
   x_0        : double;
   TFForceConstant,
   TFBaseFrequency        : double;
   delta_f0 : double; //frequency shift from far away to minimum of Freq_shift(x)

 type
  tc = class(TThread)
    procedure execute; override;
  end;
  procedure tc.execute;
  begin
  end;

  procedure UpdateFrequencyParameters;
  begin
    LJ_FreqShiftMultFact:= 1.605*delta_f0*intpower(x_0,14);
    EFM_FreqShiftMultFact:=-1.3908E-2*TipRadius*TipRadius*TipVoltage*TipVoltage*
                             (TFBaseFrequency/TFForceConstant);
  end;

{ TTuningForkForm }

procedure TTuningForkForm.FormCreate(Sender: TObject);

var
  n_subdevices, subdevtype, n_chans : longint;
  type_str      : string;
  i,j,k,
  InputChannelIndex,
  OutputChannelIndex
             : integer;

begin
    with tc.create(False) do
    begin
      waitfor;
      free;
    end;

  //PID parameters
  SAMP_TIME := 50000; // 50 us, in nanoseconds
  //TFLoop_time := SAMP_TIME;
  tf_loop_running := 0; //not running

  //Now get information about what boards and channels are available
   OutputChannelNames:=TStringList.Create;
   InputChannelNames:=TStringList.Create;

   //Now check to see if the boards are actually there
   get_board_names(board_it,device_names, board_names);

   //if the board was not present, then its corresponding board_it should still be nil
   if board_it[0]=nil then Application.MessageBox(PChar('No board present!'), 'Error!', 0);

   //Fill both the input and output channel list
   InputChannelIndex:=0;
   OutputChannelIndex:=0;
   for i:=0 to MaxBoards-1 do
      begin
      if (board_it[i] <> nil) then  //board is present
        begin
           //check to see whether we have an analog input device
           //first check the number of subdevices
           n_subdevices:=comedi_get_n_subdevices(board_it[i]);
           for j:=0 to n_subdevices-1 do
             begin
               subdevtype:=comedi_get_subdevice_type(board_it[i], j);
               if subdevtype<MaxDevTypes-1 then
                 begin
                   type_str:=subdevice_types[subdevtype];
                   //First if it is an analog input channel
                   if type_str = 'ai' then
                     begin
                      n_chans:=comedi_get_n_channels(board_it[i],j);
                      for k:=0 to (n_chans div 2)-1 do  //note we have div 2 because of diff input
                        begin
                         InputChannelNames.Add(board_names[i]+'/'+type_str+IntToStr(k));
                         InputChannels[InputChannelIndex].boardname:= board_names[i];
                         InputChannels[InputChannelIndex].devicename:= device_names[i];
                         InputChannels[InputChannelIndex].board_id := board_it[i];
                         InputChannels[InputChannelIndex].board_number := i;
                         InputChannels[InputChannelIndex].subdevice := j;
                         InputChannels[InputChannelIndex].channel := k;
                         Inc(InputChannelIndex);
                        end;
                     end;
                   //or an analog output channel
                   if type_str = 'ao' then
                     begin
                      n_chans:=comedi_get_n_channels(board_it[i],j);
                      for k:=0 to n_chans-1 do
                        begin
                         OutputChannelNames.Add(board_names[i]+'/'+type_str+IntToStr(k));
                         OutputChannels[OutputChannelIndex].boardname:= board_names[i];
                         OutputChannels[OutputChannelIndex].devicename:= device_names[i];
                         OutputChannels[OutputChannelIndex].board_id := board_it[i];
                         OutputChannels[OutputChannelIndex].board_number := i;
                         OutputChannels[OutputChannelIndex].subdevice := j;
                         OutputChannels[OutputChannelIndex].channel := k;
                         Inc(OutputChannelIndex);
                        end;
                     end;

                 end;
             end;
        end;
    end;

  //Initialize all the channel names
  InputChannel.devicename:='';
  OutputChannel.devicename:='';
  XChannel.devicename:='';
  YChannel.devicename:='';
  InitializeBoards;

  InputControlComboBox.Items:=InputChannelNames;
  InputControlComboBox.ItemIndex:=0; //first one in
  InputChannel:=InputChannels[InputControlComboBox.ItemIndex];

  XChannelComboBox.Items:=InputChannelNames;
  XChannelComboBox.ItemIndex:=1; //second one in
  XChannel:=InputChannels[XChannelComboBox.ItemIndex];

  YChannelComboBox.Items:=InputChannelNames;
  YChannelComboBox.ItemIndex:=2; //second one in
  YChannel:=InputChannels[YChannelComboBox.ItemIndex];

  OutputChannelComboBox.Items:=OutputChannelNames;
  OutputChannelComboBox.ItemIndex:=0; //second one in
  OutputChannel:=OutputChannels[OutputChannelComboBox.ItemIndex];

  //Initialize all the other edit boxes
  ControlMultFact:=100; //nm/Volt
  InputControlMultEdit.Text:=FloatToStr(ControlMultFact);

  ControlAddFact:=0;
  InputSignalAddEdit.Text:=FloatToStr(ControlAddFact);

  TubeDisplacement:=0;
  ControlSignalSpinEdit.Value:=TubeDisplacement;

  TestControlSignal:=0; //False
  TestControlCheckBox.Checked:=FALSE;
  ControlSignalSpinEdit.Enabled:=FALSE;

  EFM_Mode:=0; //False
  EFMCheckBox.Checked:=FALSE;
  TipRadius:=20; // in nm
  TipVoltage:=0; // in volts
  TipRadiusSpinEdit.Value:=TipRadius;
  TipVoltageSpinEdit.Value:=TipVoltage;

  ExternalScan:=0; //False
  ExternalScanCheckBox.Checked:=FALSE;

  delta_f0:=3; //maximum frequency shift, in Hz
  MaxFreqShiftEdit.Text:=FloatToStr(delta_f0);

  x_0:=2; //distance for mininum in LJ potential, in nm
  x_06:=intpower(x_0, 6);
  LJPlotMinEdit.Text:=FloatToStr(x_0);

  EFM_Mode:=0; //Not in EFM Mode
  TipRadius:= 20; //in nanometers
  TipVoltage:=0;

  TFForceConstant:=1800; //Tuning fork force constant, in N/m
  TFForceConstEdit.Text:=FloatToStr(TFForceConstant);

  TFBaseFrequency:=32768; //tuning fork base frequency, in Hz
  TFFreqEdit.Text:=FloatToStr(TFBaseFrequency);

  OutputAddFactor:=0;
  OutputAddVoltEdit.Caption:=FloatToStr(OutputAddFactor);

  AmplifierGainSign:=-1;
  AmpGainSpinEdit.Value:=AmplifierGainSign;

  OutputConversionFactor:=18.31; //Volts/Hz
  OutputConvFactEdit.Caption:=FloatToStr(OutputConversionFactor);

  FeatureHeight:=5; //nm height of topographic features
  FeatureCenter:=0.5; //um center in cell
  FeatureSize:=0.5; //um
  FeaturePeriod:=1.0; //um
  SizeOfFeatureSpinEdit.Value:=FeatureSize;
  CenteredAtSpinEdit.Value:=FeatureCenter;
  HeightSpinEdit.Value:=FeatureHeight;

  XPosition:=0;
  XPositionSpinEdit.Value:=XPosition;

  YPosition:=0;
  YPositionSpinEdit.Value:=YPosition;

  DataTimer.Enabled:=FALSE;

  UpdateFrequencyParameters;





  //allow nonroot access
  //rt_allow_nonroot_hrt;
  //start the maitask as a soft real time task with a priority of 10, higher
  //than any other real-time task
  GlobalTaskStarted:=StartMainTask(1);
  //SysConfig.Show;
end;


procedure TTuningForkForm.TestControlCheckBoxClick(Sender: TObject);
begin
  if TestControlSignal=0 then //false, not under test control
    begin
      TestControlCheckBox.Checked:=TRUE;
      TestControlSignal:=1;
      ControlSignalSpinEdit.Enabled:=TRUE;
    end
   else
    begin
      TestControlCheckBox.Checked:=FALSE;
      TestControlSignal:=0;
      ControlSignalSpinEdit.Enabled:=FALSE;
    end;
end;

procedure TTuningForkForm.TFForceConstEditKeyPress(Sender: TObject;
  var Key: char);
begin
  if Key=Chr(13) then
    begin
     TFForceConstant:=StrToFloat(TFForceConstEdit.Text);
     UpdateFrequencyParameters;
    end;
end;

procedure TTuningForkForm.TFFreqEditKeyPress(Sender: TObject; var Key: char);
begin
  if Key=Chr(13) then
    begin
      TFBaseFrequency:=StrToFloat(TFFreqEdit.Text);
      UpdateFrequencyParameters;
    end;
end;

procedure TTuningForkForm.TipRadiusSpinEditChange(Sender: TObject);
begin
  TipRadius:=TipRadiusSpinEdit.Value;
  UpdateFrequencyParameters;
end;

procedure TTuningForkForm.TipVoltageSpinEditChange(Sender: TObject);
begin
  TipVoltage:=TipVoltageSpinEdit.Value;
  UpdateFrequencyParameters;
end;

procedure TTuningForkForm.XChannelComboBoxSelect(Sender: TObject);
begin
  XChannelName:=XChannelComboBox.Text;
  XChannel:=InputChannels[XChannelComboBox.ItemIndex];
end;

procedure TTuningForkForm.XPositionSpinEditChange(Sender: TObject);
begin
  XPosition:=XPositionSpinEdit.Value;
end;

procedure TTuningForkForm.YChannelComboBoxSelect(Sender: TObject);
begin
  YChannelName:=YChannelComboBox.Text;
  YChannel:=InputChannels[YChannelComboBox.ItemIndex];
end;

procedure TTuningForkForm.YPositionSpinEditChange(Sender: TObject);
begin
  YPosition:=YPositionSpinEdit.Value;
end;

procedure TTuningForkForm.StartSimulatorBtnClick(Sender: TObject);
begin
    if InOperation then
    begin  //stop the feedback
      DataTimer.Enabled:=FALSE;
      InOperation:=FALSE;
      StartSimulatorBtn.Caption:='Start Simulator';
      tf_loop_running:=0;
      EndTFSimulator;
    end
   else
    begin  //start Feedback timer
      DataTimer.Enabled:=TRUE;
      InOperation:=TRUE;
      StartSimulatorBtn.Caption:='Stop Simulator';
      tf_loop_running:=1;
      StartTFSimulator;
    end;

end;

procedure TTuningForkForm.InputControlMultEditKeyPress(Sender: TObject;
  var Key: char);
begin
    if Key=Chr(13) then ControlMultFact:=StrToFloat(InputControlMultEdit.Text);
end;

procedure TTuningForkForm.InputControlComboBoxSelect(Sender: TObject);
begin
  InputChannelName:=InputControlComboBox.Text;
  InputChannel:=InputChannels[InputControlComboBox.ItemIndex];
end;

procedure TTuningForkForm.ControlSignalSpinEditChange(Sender: TObject);
begin
  TubeDisplacement:=ControlMultFact*ControlSignalSpinEdit.Value + ControlAddFact;
end;

procedure TTuningForkForm.DataTimerTimer(Sender: TObject);
begin
   XChannelIndicator.Caption:='X: '+ FloatToStrF(XPosition, ffFixed, 10, 5);
   YChannelIndicator.Caption:='Y: '+ FloatToStrF(YPosition, ffFixed, 10, 5);
   FreqShiftLabel.Caption:='Frequency shift (Hz): ' + FloatToStrF(FrequencyShift, ffFixed, 10, 5);
   DeltaKLabel.Caption:='Delta K (N/m): ' + FloatToStrF(2*FrequencyShift*(TFForceConstant/TFBaseFrequency), ffFixed, 10, 5);
   OutputSignalOutputLabel.Caption:= 'Output signal (V): ' + FloatToStrF(OutputValue, ffFixed, 10, 5);
   HeightLabel.Caption:='Height (nm): ' + FloatToStrF(XHeight, ffFixed, 10, 5);
   TubeDisplacementLabel.Caption:='Tube displacement (nm): ' + FloatToStrF(TubeDisplacement, ffFixed, 10, 5);
end;

procedure TTuningForkForm.EFMCheckBoxClick(Sender: TObject);
begin
  if EFM_Mode = 0 then //we are not in EFM mode
    begin
       EFM_Mode:=1;
       EFMCheckBox.Checked:=TRUE;
    end
   else
    begin
      EFM_Mode:=0;
      EFMCheckBox.Checked:=FALSE;
    end;
end;



procedure TTuningForkForm.ExternalScanCheckBoxClick(Sender: TObject);
begin
  if ExternalScan=0 then //we are not on external scan
    begin
      ExternalScan:=1;
      InternalPositionGroupBox.Enabled:=FALSE;
      ExternalScanCheckBox.Checked:=TRUE;
    end
   else
    begin
      ExternalScan:=0;
      InternalPositionGroupBox.Enabled:=TRUE;
      ExternalScanCheckBox.Checked:=FALSE;
    end;
end;

procedure TTuningForkForm.HeightSpinEditChange(Sender: TObject);
begin
  FeatureHeight:=HeightSpinEdit.Value;
end;

procedure TTuningForkForm.AmpGainSpinEditChange(Sender: TObject);
begin
  AmplifierGainSign:=AmpGainSpinEdit.Value;
end;

procedure TTuningForkForm.CenteredAtSpinEditChange(Sender: TObject);
begin
  FeatureCenter:=CenteredAtSpinEdit.Value;
end;

procedure TTuningForkForm.InputSignalAddEditKeyPress(Sender: TObject;
  var Key: char);
begin
    if Key=Chr(13) then ControlAddFact:=StrToFloat(InputSignalAddEdit.Text);
end;

procedure TTuningForkForm.LJPlotMinEditKeyPress(Sender: TObject; var Key: char);
begin
   if Key=Chr(13) then
     begin
       x_0:=StrToFloat(LJPlotMinEdit.Text);
       x_06:=intpower(x_0, 6);
       UpdateFrequencyParameters;
     end;
end;

procedure TTuningForkForm.MaxFreqShiftEditKeyPress(Sender: TObject;
  var Key: char);
begin
  if Key=Chr(13) then
    begin
     delta_f0:=StrToFloat(MaxFreqShiftEdit.Text);
     UpdateFrequencyParameters;
    end;
end;

procedure TTuningForkForm.OutputAddVoltEditKeyPress(Sender: TObject;
  var Key: char);
begin
  if Key=Chr(13) then OutputAddFactor:=StrToFloat(OutputAddVoltEdit.Text);
end;

procedure TTuningForkForm.OutputChannelComboBoxSelect(Sender: TObject);
begin
  OutputChannelName:=OutputChannelComboBox.Text;
  OutputChannel:=OutputChannels[OutputChannelComboBox.ItemIndex];
end;

procedure TTuningForkForm.OutputConvFactEditKeyPress(Sender: TObject;
  var Key: char);
begin
  if Key=Chr(13) then OutputConversionFactor:=StrToFloat(OutputConvFactEdit.Text);
end;

procedure TTuningForkForm.SizeofFeatureSpinEditChange(Sender: TObject);
begin
  FeatureSize:=SizeOfFeatureSpinEdit.Value;
end;

initialization
  {$I mainform.lrs}

end.

