//
// Created by maxime on 11/17/24.
//

#ifndef _H_TIMING_UTILS_
#define _H_TIMING_UTILS_

#ifdef _WIN32
    #include <Windows.h>
#else
    #include <sys/time.h>
    #include <time.h>
#endif

inline double GetMilliseconds() {
#ifdef _WIN32
  return static_cast<double>(GetTickCount());
#else
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (static_cast<double>(tv.tv_sec) * 1000.0) + (static_cast<double>(tv.tv_usec) / 1000.0);
#endif
}

inline double GetSeconds() {
  return GetMilliseconds() / 1000.0;
}

#endif

