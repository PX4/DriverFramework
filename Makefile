all: test

test: DriverFramework.cpp DriverMgr.cpp main.cpp
	clang++ -std=c++11 -Wall $^ -o $@ -lpthread

clean:
	rm -f test
