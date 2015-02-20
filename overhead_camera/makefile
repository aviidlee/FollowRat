all: main image homography imagesel itrack objdet

main:
	g++ main.cpp -o overhead `pkg-config --cflags --libs opencv`

image: imageCollection.cpp
	g++ imageCollection.cpp -o imageCollection `pkg-config --cflags --libs opencv`

objdet: objectDetection.cpp
	g++ objectDetection.cpp -o objdet `pkg-config --cflags --libs opencv`

itrack: itrack.cpp
	g++ -std=c++11 itrack.cpp -o itrack `pkg-config --cflags --libs opencv`

imagesel: imageSelect.cpp
	g++ -std=c++11 imageSelect.cpp -o imageSelect `pkg-config --cflags --libs opencv`

homography: homography.cpp
	g++ homography.cpp -o homography `pkg-config --cflags --libs opencv`

clean:
	rm imageCollection overhead imageSelect itrack homography objdet
