all: test

test: DriverFramework.cpp DriverMgr.cpp main.cpp
	clang++ -std=c++11 DriverFramework.cpp DriverMgr.cpp main.cpp -o $@ -lpthread
