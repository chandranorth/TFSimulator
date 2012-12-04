
unit rtai_types;
interface

{
  Automatically converted by H2Pas 1.0.0 from /home/chandra/LazarusPrograms/InterfaceTests/rtai_types.tmp.h
  The following command line parameters were used:
    -p
    -D
    -w
    -o
    /home/chandra/LazarusPrograms/InterfaceTests/rtai_types.pas
    /home/chandra/LazarusPrograms/InterfaceTests/rtai_types.tmp.h
}

  const
    External_library='kernel32'; {Setup as you need}

  { Pointers to basic pascal types, inserted by h2pas conversion program.}
  {Type
    PLongint  = ^Longint;
    PSmallInt = ^SmallInt;
    PByte     = ^Byte;
    PWord     = ^Word;
    PDWord    = ^DWord;
    PDouble   = ^Double;}

  {Type
  Ppt_regs  = ^pt_regs;
  Prt_task_struct  = ^rt_task_struct;
  Prt_times  = ^rt_times;
  PRTIME  = ^RTIME;}
{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}


  {
   * Copyright (C) 1999-2003 Paolo Mantegazza <mantegazza@aero.polimi.it>
   *
   * This program is free software; you can redistribute it and/or
   * modify it under the terms of the GNU General Public License as
   * published by the Free Software Foundation; either version 2 of the
   * License, or (at your option) any later version.
   *
   * This program is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   * GNU General Public License for more details.
   *
   * You should have received a copy of the GNU General Public License
   * along with this program; if not, write to the Free Software
   * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
    }


  const
     PRIO_Q = 0;     
     FIFO_Q = 4;     
     RES_Q = 3;     
     BIN_SEM = 1;     
     CNT_SEM = 2;     
     RES_SEM = 3;     
     RESEM_RECURS = 1;     
     RESEM_BINSEM = 0;     
     RESEM_CHEKWT = -(1);     
     RT_SCHED_FIFO = 0;     
     RT_SCHED_RR = 1;     

  type
     Ppt_regs = ^pt_regs;
     pt_regs = record
         {undefined structure}
       end;

     Prt_task_struct = ^rt_task_struct;
     rt_task_struct = record
         {undefined structure}
       end;


     PRTIME = ^RTIME;
     RTIME = int64;

     {RT_TRAP_HANDLER = function (_para1:longint; _para2:longint; _para3:Ppt_regs; _para4:pointer):longint;cdecl;}
     Prt_times = ^rt_times;
     rt_times = record
          linux_tick : longint;
          periodic_tick : longint;
          tick_time : RTIME;
          linux_time : RTIME;
          intr_time : RTIME;
       end;

  { !_RTAI_TYPES_H  }

implementation


end.
