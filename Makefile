all: test

test: HRTWorkerThread.cpp main.cpp
	clang++ -std=c++11 HRTWorkerThread.cpp main.cpp -o $@ -lpthread
