all: build

.PHONY: build
generate:
	mkdir -p build
	cd build && cmake ..

build: generate
	cd build && make

run: build
	cd build && test/testapp
	
clean:
	rm -rf build
