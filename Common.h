#ifndef	_Common_H
#define	_Common_H
#include	<math.h>
#include	<wirish.h>

#define		DebugMode
#undef		DebugMode

typedef	unsigned char	U08;
typedef	signed char		S08;

typedef	unsigned int	U16;
typedef	signed int		S16;

#define WAIT_INPUT     100
#define START_TAPING   200
#define OKURI_START1   300
#define MOTOR_START0   400
#define MOTOR_START1   500
#define MOTOR_START2   600
#define PID_CONTROL0   700
#define PID_CONTROL1   800
#define PID_CONTROL2   900
#define CUTTING_START  1000
#define OKURI_START2   1100
#define LEVER_OPEN	   1200
#define LEVER_CLOSE	   1300

#include	"TapingMotor.h"
#include	"Accessories.h"
#include	"Interface.h"

extern int Running_Status;
extern const int Target_Count[3];
extern const int Delay_Time_Okuri[3];
extern const int Delay_Time_Cutting;
extern volatile int enc_count;
extern int Debug_On;
extern int Debug_Time;
extern int Polling_Time;
extern int MicroSec_To_MilliSec;
extern int MilliSec_To_Sec;

#endif