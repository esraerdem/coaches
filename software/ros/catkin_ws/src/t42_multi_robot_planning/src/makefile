FLAGS=-Wextra -g -fstack-check -O0
#FLAGS+= -fprofile-arcs -ftest-coverage

all: PRUplus.o PRUprint.o MDP.o PRU2MDP.o PRU2MDPprogress.o PRU2MDPstateStore.o testPRU2MDP.o
	g++ ${FLAGS} $^ `pkg-config --libs libxml++-2.6` -lboost_regex

%.o : %.cpp
	g++ ${FLAGS} -c -MP -MMD $< `pkg-config --cflags libxml++-2.6`

-include $(wildcard *.d)

