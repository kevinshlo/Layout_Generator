#ifndef _DEBUGGER_H_
#define _DEBUGGER_

#include <string>
#include <iostream>
//#define SHOW
#define DEBUG
#ifdef DEBUG
#   define M_Assert(Expr, Msg)  __M_Assert(#Expr, Expr, __FILE__, __FUNCTION__, __LINE__, Msg)
#else
#   define M_Assert(Expr, Msg) ;
#endif

void __M_Assert(const char* expr_str, bool expr, const char* file, const char* function, int line, const std::string msg);

#endif