all: PRUplus.o PRUprint.o MDP.o
	g++ -g $^ `pkg-config --libs libxml++-2.6` -lboost_regex

%.o : %.cpp
	g++ -c -g -MP -MMD $< `pkg-config --cflags libxml++-2.6`

-include $(wildcard *.d)

