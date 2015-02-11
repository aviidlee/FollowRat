all: main

main:
	g++ main.cpp -o blobslib_test -I/usr/local/include/opencvblobslib `pkg-config --cflags --libs opencv` /usr/local/lib/libopencvblobslib.a

clean:
	rm main 
