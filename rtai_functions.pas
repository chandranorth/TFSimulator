unit rtai_functions;

{$mode objfpc}{$H+}
{$linklib liblxrt.a}
{$linklib libsciblk.a}

interface

uses
  Classes, SysUtils, rtai_types;

  procedure rt_allow_nonroot_hrt; cdecl; external;
  function rt_is_hard_timer_running: boolean; cdecl; external;
  function nano2count(period: RTIME): RTIME; cdecl; external;
  function count2nano(counts: RTIME): RTIME; cdecl; external;
  procedure rt_set_periodic_mode; cdecl; external;
  procedure rt_set_oneshot_mode; cdecl; external;
  procedure rt_set_external_mode; cdecl; external;
  function start_rt_timer(period: integer): RTIME; cdecl; external;
  procedure stop_rt_timer; cdecl; external;
  function rt_get_time: RTIME; cdecl; external;
  function rt_get_time_ns: RTIME; cdecl; external;
  function rt_get_cpu_time_ns: RTIME; cdecl; external;


implementation

end.

