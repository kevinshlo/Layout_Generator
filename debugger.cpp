#include "debugger.h"

void __M_Assert(const char* expr_str, bool expr, const char* file, const char* function, int line, const std::string msg)
{
    if (!expr)
    {
        std::cerr << "Assert failed:\t" << msg << "\n"
            << "Expected:\t" << expr_str << "\n"
            << "Source:\t\t" << file <<": "<< function << "()" << ", line " << line << "\n";
        abort();
    }
}
