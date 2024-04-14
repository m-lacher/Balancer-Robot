//------------------------------------------------------
//File:			Button.h
//Project:	Balancer Robot
//Created:	04.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#ifndef Button
#define Button

extern void Button_InitModule(void);
extern void Button_SetButtonPushedCallback(void(*ButtonPushedCallback)(void));

#endif
