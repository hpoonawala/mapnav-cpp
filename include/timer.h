#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <stdio.h>

class Timer{
	public:
		Timer() : start{std::chrono::high_resolution_clock::now()}, end{std::chrono::high_resolution_clock::now()}  {};
		void reset(){
start = std::chrono::high_resolution_clock::now();
		}
		void mark(const char * str){
			end = std::chrono::high_resolution_clock::now();
			duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
			printf("%s",str); printf("%lld",duration_.count());printf(" milliseconds.\n");
		}
	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> start,end;
		std::chrono::milliseconds duration_;
};
#endif
