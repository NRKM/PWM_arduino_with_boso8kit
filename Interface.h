#ifndef	_Interface_H
#define	_Interface_H
#ifdef	__cplusplus
extern	"C"	{
#endif	
	
/////////////////////////I/O_Config/////////////////////////
#define In_PLC_TapingStart1		10
#define In_PLC_TapingStart2		11
#define In_PLC_PreOkuri			12
#define In_PLC_BarOpen			13
#define In_PLC_MotorReset		15
#define In_PLC_ReturnToOrigin	16
//#define In_PLC_Spare1			16
#define In_PLC_Spare2			17

#define Out_PLC_TapingFinish	18
#define Out_PLC_RetToOrigFinish	19
#define Out_PLC_PreOkuriFinish	20
#define Out_PLC_BarOpenFinish	21
#define Out_PLC_MotorAlarm		22
//#define Out_PLC_Spare1		22
#define Out_PLC_Spare2			23

/////////////////////////I/O_Alias_Config/////////////////////////
extern int I_PLC_TapingStart1	;
extern int I_PLC_TapingStart2	;
extern int I_PLC_PreOkuri		;
extern int I_PLC_BarOpen		;
extern int I_PLC_ReturnToOrigin	;
extern int I_PLC_MotorReset		;
//extern int I_PLC_Spare1			;
extern int I_PLC_Spare2			;

extern int O_PLC_TapingFinish	;
extern int O_PLC_RetToOrigFinish;
extern int O_PLC_PreOkuriFinish	;
extern int O_PLC_BarOpenFinish	;
extern int O_PLC_MotorAlarm		;
//extern int O_PLC_Spare1		;
extern int O_PLC_Spare2			;

/////////////////////////I/O_Logic_Config/////////////////////////
extern int L_PLC_TapingStart1	;
extern int L_PLC_TapingStart2	;
extern int L_PLC_PreOkuri		;
extern int L_PLC_BarOpen		;
extern int L_PLC_ReturnToOrigin	;
extern int L_PLC_MotorReset		;
//extern int L_PLC_Spare1		;
extern int L_PLC_Spare2			;

extern void Init_Interface(void);
extern void initI(void);
extern void IOCheck(void);
extern void Serial_Check(void);
extern void SignalsRead(void);

#ifdef	__cplusplus
}
#endif
#endif
