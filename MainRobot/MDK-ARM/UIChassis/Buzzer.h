#ifndef __Buzzer_h
#define __Buzzer_h

typedef enum
{
	Buzzer_OFF = 0,
	Buzzer_InterruptionRocker,
	Buzzer_InterruptionArm
	
}Buzzer_State;
extern Buzzer_State BuzzerState;
	
void Buzzer_StartUp(void);

#endif
