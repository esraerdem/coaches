all: PRUplus.o PRUprint.o MDP.o PRU2MDP.o PRU2MDPprogress.o testPRU2MDP.o
	g++ -Wextra -g -fstack-check $^ `pkg-config --libs libxml++-2.6` -lboost_regex

%.o : %.cpp
	g++ -Wextra -c -g -MP -MMD -fstack-check $< `pkg-config --cflags libxml++-2.6`

-include $(wildcard *.d)

