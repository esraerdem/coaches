all: PRUplus.o PRUprint.o MDP.o PRU2MDP.o testPRU2MDP.o
	g++ -Wall -g $^ `pkg-config --libs libxml++-2.6` -lboost_regex

%.o : %.cpp
	g++ -Wall -c -g -MP -MMD $< `pkg-config --cflags libxml++-2.6`

-include $(wildcard *.d)

