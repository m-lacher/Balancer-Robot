//------------------------------------------------------
//File:			ErrorHandler.h
//Project:	Balancer Robot
//Created:	03.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#ifndef ErrorHandler
#define ErrorHandler

extern void ErrorHandler_InitModule(void);
extern void ErrorHandler_HardFault(void);
extern void ErrorHandler_SetWarning(void);
extern void ErrorHandler_ResetWarning(void);

#endif
