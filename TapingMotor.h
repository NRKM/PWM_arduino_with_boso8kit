#ifndef	_TapingMotor_H
#define	_TapingMotor_H
#ifdef	__cplusplus
extern	"C"	{
#endif

#define Motor_pinA		0
#define Motor_pinB		1
#define Motor_pinPWM	2
#define Motor_pinFW		3
#define Motor_pinBR		24
#define Motor_pinRE		25

extern int M_pinA		;
extern int M_pinB		;
extern int M_pinPWM		;
extern int M_pinFW		;
extern int M_pinBR		;
extern int M_pinRE		;

extern double P			;
extern double I			;
extern double D			;
extern double PWM_MAX	;
extern double PWM_MIN	;
extern int PID_On		;

extern void Init_TapingMotor(void);
extern void enc_changedPinA(void);
extern void enc_changedPinB(void);
extern void Forward(void);
extern void Break(void);
extern void Reverse(void);
extern void PID_Start(void);
extern void PID_Control(int Target_Count, int phase);
extern void PWM_OutPutValue_Check(int Output);
extern void COrigin_Check(int p_deviation,int phase,int output);

#ifdef	__cplusplus
}
#endif
#endif
