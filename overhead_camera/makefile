all: main image

main:
	g++ main.cpp -o overhead `pkg-config --cflags --libs opencv`

image: imageCollection.cpp
	g++ imageCollection.cpp -o imageCollection `pkg-config --cflags --libs opencv`

clean:
	rm imageCollection overhead
