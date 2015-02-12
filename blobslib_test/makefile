all: main

main:
	g++ main.cpp -o blobslib_test -I/usr/local/include/opencvblobslib /usr/local/lib/libopencvblobslib.a \
	`pkg-config --cflags --libs opencv`

test:
	g++ another_test.cpp -o another_test -I/usr/local/include/opencvblobslib /usr/local/lib/libopencvblobslib.a \
	`pkg-config --cflags --libs opencv`

clean:
	rm main 
