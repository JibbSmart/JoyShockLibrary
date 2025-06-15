// JoyShockLibrary.h - Contains declarations of functions
#pragma once
#include <widemath.h>
#include <vector>

#if _MSC_VER // this is defined when compiling with Visual Studio
#define JOY_SHOCK_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define JOY_SHOCK_API // XCode does not need annotating exported functions, so define is empty
#endif

#define JS_TYPE_JOYCON_LEFT 1
#define JS_TYPE_JOYCON_RIGHT 2
#define JS_TYPE_PRO_CONTROLLER 3
#define JS_TYPE_DS4 4
#define JS_TYPE_DS 5

#define JS_SPLIT_TYPE_LEFT 1
#define JS_SPLIT_TYPE_RIGHT 2
#define JS_SPLIT_TYPE_FULL 3

#define JSMASK_UP 0x000001
#define JSMASK_DOWN 0x000002
#define JSMASK_LEFT 0x000004
#define JSMASK_RIGHT 0x000008
#define JSMASK_PLUS 0x000010
#define JSMASK_OPTIONS 0x000010
#define JSMASK_MINUS 0x000020
#define JSMASK_SHARE 0x000020
#define JSMASK_LCLICK 0x000040
#define JSMASK_RCLICK 0x000080
#define JSMASK_L 0x000100
#define JSMASK_R 0x000200
#define JSMASK_ZL 0x000400
#define JSMASK_ZR 0x000800
#define JSMASK_S 0x001000
#define JSMASK_E 0x002000
#define JSMASK_W 0x004000
#define JSMASK_N 0x008000
#define JSMASK_HOME 0x010000
#define JSMASK_PS 0x010000
#define JSMASK_CAPTURE 0x020000
#define JSMASK_TOUCHPAD_CLICK 0x020000
#define JSMASK_MIC 0x040000
#define JSMASK_SL 0x080000
#define JSMASK_SR 0x100000
#define JSMASK_FNL 0x200000
#define JSMASK_FNR 0x400000

#define JSOFFSET_UP 0
#define JSOFFSET_DOWN 1
#define JSOFFSET_LEFT 2
#define JSOFFSET_RIGHT 3
#define JSOFFSET_PLUS 4
#define JSOFFSET_OPTIONS 4
#define JSOFFSET_MINUS 5
#define JSOFFSET_SHARE 5
#define JSOFFSET_LCLICK 6
#define JSOFFSET_RCLICK 7
#define JSOFFSET_L 8
#define JSOFFSET_R 9
#define JSOFFSET_ZL 10
#define JSOFFSET_ZR 11
#define JSOFFSET_S 12
#define JSOFFSET_E 13
#define JSOFFSET_W 14
#define JSOFFSET_N 15
#define JSOFFSET_HOME 16
#define JSOFFSET_PS 16
#define JSOFFSET_CAPTURE 17
#define JSOFFSET_TOUCHPAD_CLICK 17
#define JSOFFSET_MIC 18
#define JSOFFSET_SL 19
#define JSOFFSET_SR 20
#define JSOFFSET_FNL 21
#define JSOFFSET_FNR 22

// PS5 Player maps for the DS Player Lightbar
#define DS5_PLAYER_1 4
#define DS5_PLAYER_2 10
#define DS5_PLAYER_3 21
#define DS5_PLAYER_4 27
#define DS5_PLAYER_5 31

#define DS5_TRIGGER_OFF 0x05
#define DS5_TRIGGER_FEEDBACK 0x21 
#define DS5_TRIGGER_WEAPON 0x25 
#define DS5_TRIGGER_VIBRATION 0x26 
#define DS5_TRIGGER_BOW 0x22
#define DS5_TRIGGER_GALLOPING 0x23 
#define DS5_TRIGGER_MACHINE 0x27

typedef struct JOY_SHOCK_STATE {
	int buttons = 0;
	float lTrigger = 0.f;
	float rTrigger = 0.f;
	float stickLX = 0.f;
	float stickLY = 0.f;
	float stickRX = 0.f;
	float stickRY = 0.f;
} JOY_SHOCK_STATE;

typedef struct IMU_STATE {
	float accelX = 0.f;
	float accelY = 0.f;
	float accelZ = 0.f;
	float gyroX = 0.f;
	float gyroY = 0.f;
	float gyroZ = 0.f;
} IMU_STATE;

typedef struct MOTION_STATE {
	float quatW = 0.f;
	float quatX = 0.f;
	float quatY = 0.f;
	float quatZ = 0.f;
	float accelX = 0.f;
	float accelY = 0.f;
	float accelZ = 0.f;
	float gravX = 0.f;
	float gravY = 0.f;
	float gravZ = 0.f;
} MOTION_STATE;

typedef struct TOUCH_STATE {
	int t0Id = 0;
	int t1Id = 0;
	bool t0Down = false;
	bool t1Down = false;
	float t0X = 0.f;
	float t0Y = 0.f;
	float t1X = 0.f;
	float t1Y = 0.f;
} TOUCH_STATE;

typedef struct JSL_AUTO_CALIBRATION {
	float confidence = 0.f;
	bool autoCalibrationEnabled = false;
	bool isSteady = false;
} JSL_AUTO_CALIBRATION;

typedef struct JSL_SETTINGS {
	int gyroSpace = 0;
	int colour = 0;
	int playerNumber = 0;
	int controllerType = 0;
	int splitType = 0;
	bool isCalibrating = false;
	bool autoCalibrationEnabled = false;
	bool isConnected = false;
	char path[256];
} JSL_SETTINGS;

struct brcm_hdr {
    uint8_t cmd;
    uint8_t timer;
    uint8_t rumble_l[4];
    uint8_t rumble_r[4];
};

struct brcm_cmd_01 {
    uint8_t subcmd;
    union {
        struct {
            uint32_t offset;
            uint8_t size;
        } spi_data;

        struct {
            uint8_t arg1;
            uint8_t arg2;
        } subcmd_arg;

        struct {
            uint8_t mcu_cmd;
            uint8_t mcu_subcmd;
            uint8_t mcu_mode;
        } subcmd_21_21;

        struct {
            uint8_t  mcu_cmd;
            uint8_t  mcu_subcmd;
            uint8_t  no_of_reg;
            uint16_t reg1_addr;
            uint8_t  reg1_val;
            uint16_t reg2_addr;
            uint8_t  reg2_val;
            uint16_t reg3_addr;
            uint8_t  reg3_val;
            uint16_t reg4_addr;
            uint8_t  reg4_val;
            uint16_t reg5_addr;
            uint8_t  reg5_val;
            uint16_t reg6_addr;
            uint8_t  reg6_val;
            uint16_t reg7_addr;
            uint8_t  reg7_val;
            uint16_t reg8_addr;
            uint8_t  reg8_val;
            uint16_t reg9_addr;
            uint8_t  reg9_val;
        } subcmd_21_23_04;

        struct {
            uint8_t  mcu_cmd;
            uint8_t  mcu_subcmd;
            uint8_t  mcu_ir_mode;
            uint8_t  no_of_frags;
            uint16_t mcu_major_v;
            uint16_t mcu_minor_v;
        } subcmd_21_23_01;
    };
};


struct ds5_trigger_effect
{
    bool bDirty = false;
    unsigned char motorMode = 0;
    unsigned char startResistance = 0;
    unsigned char effectForce = 0;
    unsigned char rangeForce = 0;
    unsigned char nearReleaseStrength = 0;
    unsigned char nearMiddleStrength = 0;
    unsigned char pressedStrength = 0;
    unsigned char P6 = 0;
    unsigned char P7 = 0;
    unsigned char actuationFrequency = 0;
    unsigned char P9 = 0;
};

enum class EDS5AffectedTriggers
{
    Both = 1,
    Left,
    Right
};

extern "C" JOY_SHOCK_API int JslConnectDevices();
extern "C" JOY_SHOCK_API int JslGetConnectedDeviceHandles(int* deviceHandleArray, int size);
extern "C" JOY_SHOCK_API void JslDisconnectAndDisposeAll();
extern "C" JOY_SHOCK_API bool JslStillConnected(int deviceId);
extern "C" JOY_SHOCK_API void JslDisconnect(int deviceId);

// get buttons as bits in the following order, using North South East West to name face buttons to avoid ambiguity between Xbox and Nintendo layouts:
// 0x00001: up
// 0x00002: down
// 0x00004: left
// 0x00008: right
// 0x00010: plus
// 0x00020: minus
// 0x00040: left stick click
// 0x00080: right stick click
// 0x00100: L
// 0x00200: R
// ZL and ZR are reported as analogue inputs (GetLeftTrigger, GetRightTrigger), because DS4 and XBox controllers use analogue triggers, but we also have them as raw buttons
// 0x00400: ZL
// 0x00800: ZR
// 0x01000: S
// 0x02000: E
// 0x04000: W
// 0x08000: N
// 0x10000: home / PS
// 0x20000: capture / touchpad-click
// 0x40000: SL
// 0x80000: SR
// These are the best way to get all the buttons/triggers/sticks, gyro/accelerometer (IMU), orientation/acceleration/gravity (Motion), or touchpad
extern "C" JOY_SHOCK_API JOY_SHOCK_STATE JslGetSimpleState(int deviceId);
extern "C" JOY_SHOCK_API IMU_STATE JslGetIMUState(int deviceId);
extern "C" JOY_SHOCK_API MOTION_STATE JslGetMotionState(int deviceId);
extern "C" JOY_SHOCK_API TOUCH_STATE JslGetTouchState(int deviceId, bool previous = false);
extern "C" JOY_SHOCK_API bool JslGetTouchpadDimension(int deviceId, int &sizeX, int &sizeY);

extern "C" JOY_SHOCK_API int JslGetButtons(int deviceId);

// get thumbsticks
extern "C" JOY_SHOCK_API float JslGetLeftX(int deviceId);
extern "C" JOY_SHOCK_API float JslGetLeftY(int deviceId);
extern "C" JOY_SHOCK_API float JslGetRightX(int deviceId);
extern "C" JOY_SHOCK_API float JslGetRightY(int deviceId);

// get triggers. Switch controllers don't have analogue triggers, but will report 0.0 or 1.0 so they can be used in the same way as others
extern "C" JOY_SHOCK_API float JslGetLeftTrigger(int deviceId);
extern "C" JOY_SHOCK_API float JslGetRightTrigger(int deviceId);

// get gyro
extern "C" JOY_SHOCK_API float JslGetGyroX(int deviceId);
extern "C" JOY_SHOCK_API float JslGetGyroY(int deviceId);
extern "C" JOY_SHOCK_API float JslGetGyroZ(int deviceId);

// get accumulated average gyro since this function was last called or last flushed values
extern "C" JOY_SHOCK_API void JslGetAndFlushAccumulatedGyro(int deviceId, float& gyroX, float& gyroY, float& gyroZ);

// set gyro space. JslGetGyro*, JslGetAndFlushAccumulatedGyro, JslGetIMUState, and the IMU_STATEs reported in the callback functions will use one of 3 transformations:
// 0 = local space -> no transformation is done on gyro input
// 1 = world space -> gyro input is transformed based on the calculated gravity direction to account for the player's preferred controller orientation
// 2 = player space -> a simple combination of local and world space that is as adaptive as world space but is as robust as local space
extern "C" JOY_SHOCK_API void JslSetGyroSpace(int deviceId, int gyroSpace);

// get accelerometor
extern "C" JOY_SHOCK_API float JslGetAccelX(int deviceId);
extern "C" JOY_SHOCK_API float JslGetAccelY(int deviceId);
extern "C" JOY_SHOCK_API float JslGetAccelZ(int deviceId);

// get touchpad
extern "C" JOY_SHOCK_API int JslGetTouchId(int deviceId, bool secondTouch = false);
extern "C" JOY_SHOCK_API bool JslGetTouchDown(int deviceId, bool secondTouch = false);

extern "C" JOY_SHOCK_API float JslGetTouchX(int deviceId, bool secondTouch = false);
extern "C" JOY_SHOCK_API float JslGetTouchY(int deviceId, bool secondTouch = false);

// analog parameters have different resolutions depending on device
extern "C" JOY_SHOCK_API float JslGetStickStep(int deviceId);
extern "C" JOY_SHOCK_API float JslGetTriggerStep(int deviceId);
extern "C" JOY_SHOCK_API float JslGetPollRate(int deviceId);
extern "C" JOY_SHOCK_API float JslGetTimeSinceLastUpdate(int deviceId);

// calibration
extern "C" JOY_SHOCK_API void JslResetContinuousCalibration(int deviceId);
extern "C" JOY_SHOCK_API void JslStartContinuousCalibration(int deviceId);
extern "C" JOY_SHOCK_API void JslPauseContinuousCalibration(int deviceId);
extern "C" JOY_SHOCK_API void JslSetAutomaticCalibration(int deviceId, bool enabled);
extern "C" JOY_SHOCK_API void JslGetCalibrationOffset(int deviceId, float& xOffset, float& yOffset, float& zOffset);
extern "C" JOY_SHOCK_API void JslSetCalibrationOffset(int deviceId, float xOffset, float yOffset, float zOffset);
extern "C" JOY_SHOCK_API JSL_AUTO_CALIBRATION JslGetAutoCalibrationStatus(int deviceId);

// this function will get called for each input event from each controller
extern "C" JOY_SHOCK_API void JslSetCallback(void(*callback)(int, JOY_SHOCK_STATE, JOY_SHOCK_STATE, IMU_STATE, IMU_STATE, float));
// this function will get called for each input event, even if touch data didn't update
extern "C" JOY_SHOCK_API void JslSetTouchCallback(void(*callback)(int, TOUCH_STATE, TOUCH_STATE, float));
// this function will get called for each device when it is newly connected
extern "C" JOY_SHOCK_API void JslSetConnectCallback(void(*callback)(int));
// this function will get called for each device when it is disconnected
extern "C" JOY_SHOCK_API void JslSetDisconnectCallback(void(*callback)(int, bool));

// super-getter for reading a whole lot of state at once
extern "C" JOY_SHOCK_API JSL_SETTINGS JslGetControllerInfoAndSettings(int deviceId);
// what kind of controller is this?
extern "C" JOY_SHOCK_API int JslGetControllerType(int deviceId);
// is this a left, right, or full controller?
extern "C" JOY_SHOCK_API int JslGetControllerSplitType(int deviceId);
// what colour is the controller (not all controllers support this; those that don't will report white)
extern "C" JOY_SHOCK_API int JslGetControllerBodyColour(int deviceId);
// what colour is the controller (not all controllers support this; those that don't will report white)
extern "C" JOY_SHOCK_API int JslGetControllerLeftGripColour(int deviceId);
// what colour is the controller (not all controllers support this; those that don't will report white)
extern "C" JOY_SHOCK_API int JslGetControllerRightGripColour(int deviceId);
// what colour is the controller (not all controllers support this; those that don't will report white)
extern "C" JOY_SHOCK_API int JslGetControllerButtonColour(int deviceId);
// set controller light colour (not all controllers have a light whose colour can be set, but that just means nothing will be done when this is called -- no harm)
extern "C" JOY_SHOCK_API void JslSetLightColour(int deviceId, int colour);
// set controller rumble
extern "C" JOY_SHOCK_API void JslSetRumble(int deviceId, int smallRumble, int bigRumble);
/// <summary>
/// Turn the trigger effect off and return the trigger stop to the neutral position.
/// This is an official effect and is expected to be present in future DualSense firmware.
/// </summary>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersOff(int deviceId, EDS5AffectedTriggers affectedTriggers);
/// <summary>
/// Trigger will resist movement beyond the start position.
/// This is an official effect and is expected to be present in future DualSense firmware.
/// </summary>
/// <param name="position">The starting zone of the trigger effect. Must be between 0 and 9 inclusive.</param>
/// <param name="strength">The force of the resistance. Must be between 0 and 8 inclusive.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersFeedback(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char position, unsigned char strength);
/// <summary>
/// Trigger will resist movement beyond the start position until the end position.
/// and 2 after until again before the start position.
/// This is an official effect and is expected to be present in future DualSense firmware.
/// </summary>
/// <param name="startPosition">The starting zone of the trigger effect. Must be between 2 and 7 inclusive.</param>
/// <param name="endPosition">The ending zone of the trigger effect. Must be between <paramref name="startPosition"/>+1 and 8 inclusive.</param>
/// <param name="strength">The force of the resistance. Must be between 0 and 8 inclusive.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersWeapon(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition, unsigned char strength);
/// <summary>
/// Trigger will vibrate with the input amplitude and frequency beyond the start position.
/// This is an official effect and is expected to be present in future DualSense firmware.
/// </summary>
/// <param name="position">The starting zone of the trigger effect. Must be between 0 and 9 inclusive.</param>
/// <param name="amplitude">Strength of the automatic cycling action. Must be between 0 and 8 inclusive.</param>
/// <param name="frequency">Frequency of the automatic cycling action in hertz.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersVibration(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char position, unsigned char amplitude, unsigned char frequency);
/// <summary>
/// Trigger will resist movement at varrying strengths in 10 regions.
/// This is an official effect and is expected to be present in future DualSense firmware.
/// </summary>
/// <seealso cref="JslSetDS5TriggersFeedback(int, EDS5AffectedTriggers, unsigned char, unsigned char)"/>
/// <param name="strength">Array of 10 resistance values for zones 0 through 9. Must be between 0 and 8 inclusive.</param>
/// <returns>The success of the effect write.</returns>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersMultiPosFeedback(int deviceId, EDS5AffectedTriggers affectedTriggers, const std::vector<unsigned char>& strength);
/// <summary>
/// Trigger will resist movement at a linear range of strengths.
/// This is an official effect and is expected to be present in future DualSense firmware.
/// </summary>
/// <seealso cref="JslSetDS5TriggersFeedback(int, EDS5AffectedTriggers, unsigned char, unsigned char)"/>
/// <param name="startPosition">The starting zone of the trigger effect. Must be between 0 and 8 inclusive.</param>
/// <param name="endPosition">The ending zone of the trigger effect. Must be between <paramref name="startPosition"/>+1 and 9 inclusive.</param>
/// <param name="startStrength">The force of the resistance at the start. Must be between 1 and 8 inclusive.</param>
/// <param name="endStrength">The force of the resistance at the end. Must be between 1 and 8 inclusive.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersSlopeFeedback(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition,
    unsigned char startStrength, unsigned char endStrength);
/// <summary>
/// Trigger will vibrate movement at varrying amplitudes and one frequency in 10 regions.
/// This is an official effect and is expected to be present in future DualSense firmware.
/// </summary>
/// <remarks>
/// Note this factory's results may not perform as expected.
/// </remarks>
/// <seealso cref="JslSetDS5TriggersVibration(int, EDS5AffectedTriggers, unsigned char, unsigned char, unsigned char)"/>
/// <param name="frequency">Frequency of the automatic cycling action in hertz.</param>
/// <param name="amplitude">Array of 10 strength values for zones 0 through 9. Must be between 0 and 8 inclusive.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersMultiPosVibration(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char frequency, const std::vector<unsigned char>& amplitude);
/// <summary>
/// The effect resembles the <see cref="JslSetDS5TriggersWeapon(int, EDS5AffectedTriggers, unsigned char, unsigned char, unsigned char)">JslSetDS5TriggersWeapon</see>
/// effect, however there is a snap-back force that attempts to reset the trigger.
/// This is not an official effect and may be removed in a future DualSense firmware.
/// </summary>
/// <param name="startPosition">The starting zone of the trigger effect. Must be between 0 and 8 inclusive.</param>
/// <param name="endPosition">The ending zone of the trigger effect. Must be between <paramref name="startPosition"/>+1 and 8 inclusive.</param>
/// <param name="strength">The force of the resistance. Must be between 0 and 8 inclusive.</param>
/// <param name="snapForce">The force of the snap-back. Must be between 0 and 8 inclusive.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersBow(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition,
    unsigned char strength, unsigned char snapForce);
/// <summary>
/// Trigger will oscillate in a rythmic pattern resembling galloping. Note that the
/// effect is only discernable at low frequency values.
/// This is not an official effect and may be removed in a future DualSense firmware.
/// </summary>
/// <param name="startPosition">The starting zone of the trigger effect. Must be between 0 and 8 inclusive.</param>
/// <param name="endPosition">The ending zone of the trigger effect. Must be between <paramref name="startPosition"/>+1 and 9 inclusive.</param>
/// <param name="firstFoot">Position of first foot in cycle. Must be between 0 and 6 inclusive.</param>
/// <param name="secondFoot">Position of second foot in cycle. Must be between <paramref name="firstFoot"/>+1 and 7 inclusive.</param>
/// <param name="frequency">Frequency of the automatic cycling action in hertz.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersGalloping(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition,
    unsigned char firstFoot, unsigned char secondFoot, unsigned char frequency);
/// <summary>
/// This effect resembles <see cref="JslSetDS5TriggersVibration(int, EDS5AffectedTriggers, unsigned char, unsigned char, unsigned char)">JslSetDS5TriggersVibration</see>
/// but will oscilate between two amplitudes.
/// This is not an official effect and may be removed in a future DualSense firmware.
/// </summary>
/// <param name="startPosition">The starting zone of the trigger effect. Must be between 0 and 8 inclusive.</param>
/// <param name="endPosition">The ending zone of the trigger effect. Must be between <paramref name="startPosition"/> and 9 inclusive.</param>
/// <param name="amplitudeA">Primary strength of cycling action. Must be between 0 and 7 inclusive.</param>
/// <param name="amplitudeB">Secondary strength of cycling action. Must be between 0 and 7 inclusive.</param>
/// <param name="frequency">Frequency of the automatic cycling action in hertz.</param>
/// <param name="period">Period of the oscillation between <paramref name="amplitudeA"/> and <paramref name="amplitudeB"/> in tenths of a second.</param>
extern "C" JOY_SHOCK_API void JslSetDS5TriggersMachine(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition,
    unsigned char amplitudeA, unsigned char amplitudeB, unsigned char frequency, unsigned char period);

extern "C" JOY_SHOCK_API void JslEnableHDRumble(int deviceId);
extern "C" JOY_SHOCK_API void JslDisableHDRumble(int deviceId);
extern "C" JOY_SHOCK_API void JslSetHDRumble(int deviceId, float lowFreq, float lowAmpli, float highFreq, float highAmpli);
extern "C" JOY_SHOCK_API void JslSetHDRumbleLR(int deviceId, float lowFreq_L, float lowAmpli_L, float highFreq_L, float highAmpli_L, float lowFreq_R, float lowAmpli_R, float highFreq_R, float highAmpli_R);
// set controller player number indicator (not all controllers have a number indicator which can be set, but that just means nothing will be done when this is called -- no harm)
extern "C" JOY_SHOCK_API void JslSetPlayerNumber(int deviceId, int number);
