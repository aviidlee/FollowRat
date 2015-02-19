all: main

main:
	g++ main.cpp -o bg_subtract -I/usr/local/include/opencvblobslib \
		/usr/local/lib/libopencvblobslib.a `pkg-config --cflags --libs opencv` \
		--std=c++11 -g

clean:
	rm main 
