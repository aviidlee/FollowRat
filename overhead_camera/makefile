all: main image homography

main:
	g++ main.cpp -o overhead `pkg-config --cflags --libs opencv`

image: imageCollection.cpp
	g++ imageCollection.cpp -o imageCollection `pkg-config --cflags --libs opencv`

homography: homography.cpp
	g++ homography.cpp -o homography `pkg-config --cflags --libs opencv`

clean:
	rm imageCollection overhead
