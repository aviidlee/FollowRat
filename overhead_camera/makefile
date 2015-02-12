all: main image homography imagesel

main:
	g++ main.cpp -o overhead `pkg-config --cflags --libs opencv`

image: imageCollection.cpp
	g++ imageCollection.cpp -o imageCollection `pkg-config --cflags --libs opencv`

imagesel: imageSelect.cpp
	g++ -std=c++11 imageSelect.cpp -o imageSelect `pkg-config --cflags --libs opencv`

homography: homography.cpp
	g++ homography.cpp -o homography `pkg-config --cflags --libs opencv`

clean:
	rm imageCollection overhead imageSelect
