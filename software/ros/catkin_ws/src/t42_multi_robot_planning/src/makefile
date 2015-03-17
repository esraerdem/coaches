all: PRUplus.o PRUprint.o
	g++ -g $^ `pkg-config --libs libxml++-2.6`

%.o : %.cpp
	g++ -c -g -MP -MMD $< `pkg-config --cflags libxml++-2.6`

-include "*.d"

