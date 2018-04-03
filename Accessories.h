#ifndef	_Accessories_H
#define	_Accessories_H
#ifdef	__cplusplus
extern	"C"	{
#endif

#define Taping_OkuriMotor        4
#define Taping_OkuriMotor_Middle 5
#define Taping_Cut1              6
#define Taping_Cut2              7
#define Taping_BarOpen           8
#define Sensor_COriginSignal     9

extern int T_OkuriMotor				;
extern int T_OkuriMotor_Middle		;
extern int T_Cut1					;
extern int T_Cut2					;
extern int T_BarOpen				;
extern int S_COriginSignal			;

extern void Init_Accessories(void);
extern void initA(void);
extern void Cutting(int Delay_Time_Cutting);
extern void Okuri_Start(int DelayTime);
extern void LeverOpen(void);
extern void LeverClose(void);

#ifdef	__cplusplus
}
#endif
#endif
