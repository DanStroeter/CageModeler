#ifndef MACRO_H
#define MACRO_H

#include <iostream>

#define ASSERT(x)                                          \
    do {                                                   \
      if (!(x)) {                                          \
        std::cerr << "# error: assertion failed at\n";     \
        std::cerr << __FILE__ << " " << __LINE__ << "\n";  \
        std::cerr << "# for: " << #x << std::endl;         \
        exit(0);                                           \
      }                                                    \
    } while(0);

#define CALL_SUB_PROG(prog)                             \
  int prog(ptree &pt);                                  \
  if ( pt.get<std::string>("prog.value") == #prog )     \
    return prog(pt);

#endif
