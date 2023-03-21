unit JoyShockLibrary;

interface

const
  JS_TYPE_JOYCON_LEFT = 1;
  JS_TYPE_JOYCON_RIGHT = 2;
  JS_TYPE_PRO_CONTROLLER = 3;
  JS_TYPE_DS4 = 4;
  JS_TYPE_DS = 5;

  JS_SPLIT_TYPE_LEFT = 1;
  JS_SPLIT_TYPE_RIGHT = 2;
  JS_SPLIT_TYPE_FULL = 3;

  JSMASK_UP = $00001;
  JSMASK_DOWN = $00002;
  JSMASK_LEFT = $00004;
  JSMASK_RIGHT = $00008;
  JSMASK_PLUS = $00010;
  JSMASK_OPTIONS = $00010;
  JSMASK_MINUS = $00020;
  JSMASK_SHARE = $00020;
  JSMASK_LCLICK = $00040;
  JSMASK_RCLICK = $00080;
  JSMASK_L = $00100;
  JSMASK_R = $00200;
  JSMASK_ZL = $00400;
  JSMASK_ZR = $00800;
  JSMASK_S = $01000;
  JSMASK_E = $02000;
  JSMASK_W = $04000;
  JSMASK_N = $08000;
  JSMASK_HOME = $10000;
  JSMASK_PS = $10000;
  JSMASK_CAPTURE = $20000;
  JSMASK_TOUCHPAD_CLICK = $20000;
  JSMASK_MIC = $40000;
  JSMASK_SL = $40000;
  JSMASK_SR = $80000;

  JSOFFSET_UP = 0;
  JSOFFSET_DOWN = 1;
  JSOFFSET_LEFT = 2;
  JSOFFSET_RIGHT = 3;
  JSOFFSET_PLUS = 4;
  JSOFFSET_OPTIONS = 4;
  JSOFFSET_MINUS = 5;
  JSOFFSET_SHARE = 5;
  JSOFFSET_LCLICK = 6;
  JSOFFSET_RCLICK = 7;
  JSOFFSET_L = 8;
  JSOFFSET_R = 9;
  JSOFFSET_ZL = 10;
  JSOFFSET_ZR = 11;
  JSOFFSET_S = 12;
  JSOFFSET_E = 13;
  JSOFFSET_W = 14;
  JSOFFSET_N = 15;
  JSOFFSET_HOME = 16;
  JSOFFSET_PS = 16;
  JSOFFSET_CAPTURE = 17;
  JSOFFSET_TOUCHPAD_CLICK = 17;
  JSOFFSET_MIC = 18;
  JSOFFSET_SL = 18;
  JSOFFSET_SR = 19;

  // PS5 Player maps for the DS Player Lightbar;
  DS5_PLAYER_1 = 4;
  DS5_PLAYER_2 = 10;
  DS5_PLAYER_3 = 21;
  DS5_PLAYER_4 = 27;
  DS5_PLAYER_5 = 31;

type
  JOY_SHOCK_STATE = record
    buttons:integer;
    lTrigger:single;
    rTrigger:single;
    stickLX:single;
    stickLY:single;
    stickRX:single;
    stickRY:single;
  end;

  IMU_STATE = record
    accelX:single;
    accelY:single;
    accelZ:single;
    gyroX:single;
    gyroY:single;
    gyroZ:single;
  end;

  MOTION_STATE = record
    quatW:single;
    quatX:single;
    quatY:single;
    quatZ:single;
    accelX:single;
    accelY:single;
    accelZ:single;
    gravX:single;
    gravY:single;
    gravZ:single;
  end;

  TOUCH_STATE = record
    t0Id:integer;
    t1Id:integer;
    t0Down:boolean;
    t1Down:boolean;
    t0X:single;
    t0Y:single;
    t1X:single;
    t1Y:single;
  end;

const
  jsldll = 'JoyShockLibrary.dll';

type
  TJslCallback = procedure(deviceId:integer; PrevShockState,CurrentShockState:JOY_SHOCK_STATE; PrevIMUState,CurrentIMUSTate:IMU_STATE; DeltaTime:single); cdecl;
  TJslTouchCallback = procedure(deviceId:integer; PrevTouchState,CurrentTouchState:TOUCH_STATE; DeltaTime:single); cdecl;

  PJslCallback = ^TJslCallback;
  PJslTouchCallback = ^TJslTouchCallback;

function JslConnectDevices():integer; cdecl; external jsldll;
function JslGetConnectedDeviceHandles(deviceHandleArray:PInteger;size:integer):integer; cdecl; external jsldll;
procedure JslDisconnectAndDisposeAll(); cdecl; external jsldll;

// get buttons as bits in the following order, using North South East West to name face buttons to avoid ambiguity between Xbox and Nintendo layouts:
// $00001: up
// $00002: down
// $00004: left
// $00008: right
// $00010: plus
// $00020: minus
// $00040: left stick click
// $00080: right stick click
// $00100: L
// $00200: R
// ZL and ZR are reported as analogue inputs (GetLeftTrigger, GetRightTrigger), because DS4 and XBox controllers use analogue triggers, but we also have them as raw buttons
// $00400: ZL
// $00800: ZR
// $01000: S
// $02000: E
// $04000: W
// $08000: N
// $10000: home / PS
// $20000: capture / touchpad-click
// $40000: SL
// $80000: SR
// These are the best way to get all the buttons/triggers/sticks, gyro/accelerometer (IMU), orientation/acceleration/gravity (Motion), or touchpad
function JslGetSimpleState(deviceId:integer):JOY_SHOCK_STATE; cdecl; external jsldll ;
function JslGetIMUState(deviceId:integer):IMU_STATE; cdecl; external jsldll;
function JslGetMotionState(deviceId:integer):MOTION_STATE; cdecl; external jsldll;
function JslGetTouchState(deviceId:integer):TOUCH_STATE; cdecl; external jsldll;

function JslGetButtons(deviceId:integer):integer; cdecl; external jsldll;
// get thumbsticks
function JslGetLeftX(deviceId:integer):single; cdecl; external jsldll;
function JslGetLeftY(deviceId:integer):single; cdecl; external jsldll;
function JslGetRightX(deviceId:integer):single; cdecl; external jsldll;
function JslGetRightY(deviceId:integer):single; cdecl; external jsldll;

// get triggers. Switch controllers don't have analogue triggers, but will report 0.0 or 1.0 so they can be used in the same way as others
function JslGetLeftTrigger(deviceId:integer):single; cdecl; external jsldll;
function JslGetRightTrigger(deviceId:integer):single; cdecl; external jsldll;

// get gyro
function JslGetGyroX(deviceId:integer):single; cdecl; external jsldll;
function JslGetGyroY(deviceId:integer):single; cdecl; external jsldll;
function JslGetGyroZ(deviceId:integer):single; cdecl; external jsldll;

// get accelerometor
function JslGetAccelX(deviceId:integer):single; cdecl; external jsldll;
function JslGetAccelY(deviceId:integer):single; cdecl; external jsldll;
function JslGetAccelZ(deviceId:integer):single; cdecl; external jsldll;

// get touchpad
function JslGetTouchId(deviceId:integer; secondTouch:boolean = false):integer; cdecl; external jsldll;
function JslGetTouchDown(deviceId:integer; secondTouch:boolean = false):boolean; cdecl; external jsldll;

function JslGetTouchX(deviceId:integer; secondTouch:boolean = false):single; cdecl; external jsldll;
function JslGetTouchY(deviceId:integer; secondTouch:boolean = false):single; cdecl; external jsldll;

// analog parameters have different resolutions depending on device
function JslGetStickStep(deviceId:integer):single; cdecl; external jsldll;
function JslGetTriggerStep(deviceId:integer):single; cdecl; external jsldll;
function JslGetPollRate(deviceId:integer):single; cdecl; external jsldll;

// calibration
procedure JslResetContinuousCalibration(deviceId:integer); cdecl; external jsldll;
procedure JslStartContinuousCalibration(deviceId:integer); cdecl; external jsldll;
procedure JslPauseContinuousCalibration(deviceId:integer); cdecl; external jsldll;
procedure JslGetCalibrationOffset(deviceId:integer; var xOffset, yOffset, zOffset: single); cdecl; external jsldll;
procedure JslSetCalibrationOffset(deviceId:integer; xOffset, yOffset, zOffset: single);cdecl; external jsldll;

// this function will get called for each input event from each controller callback(int, JOY_SHOCK_STATE, JOY_SHOCK_STATE, IMU_STATE, IMU_STATE, float)
procedure JslSetCallback(callback:PJslCallback); cdecl; external jsldll;
// this function will get called for each input event, even if touch data didn't update callback(int, TOUCH_STATE, TOUCH_STATE, float)
procedure JslSetTouchCallback(callback:PJslTouchCallback); cdecl; external jsldll;

// what kind of controller is this?
function JslGetControllerType(deviceId:integer):integer; cdecl; external jsldll;
// is this a left, right, or full controller?
function JslGetControllerSplitType(deviceId:integer):integer; cdecl; external jsldll;
// what colour is the controller (not all controllers support this; those that don't will report white)
function JslGetControllerColour(deviceId:integer):integer; cdecl; external jsldll;
// set controller light colour (not all controllers have a light whose colour can be set, but that just means nothing will be done when this is called -- no harm)
procedure JslSetLightColour(deviceId:integer; colour:integer); cdecl; external jsldll;
// set controller rumble
procedure JslSetRumble(deviceId:integer; smallRumble,bigRumble:integer); cdecl; external jsldll;
// set controller player number indicator (not all controllers have a number indicator which can be set, but that just means nothing will be done when this is called -- no harm)
procedure JslSetPlayerNumber(deviceId:integer; number:integer); cdecl; external jsldll;

implementation
end.
