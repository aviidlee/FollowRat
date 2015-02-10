
all: dumb 

dumb: dumbBlob.cpp
	g++ -std=c++11 dumbBlob.cpp -o dumbBlob `pkg-config --cflags --libs opencv`

clean:
	rm dumbBlob 
