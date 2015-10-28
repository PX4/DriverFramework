all: test

test: DriverFramework.cpp main.cpp
	clang++ -std=c++11 DriverFramework.cpp main.cpp -o $@ -lpthread
