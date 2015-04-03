#ifndef __PRU2MDPactionDescriptor__
#define __PRU2MDPactionDescriptor__

#include "PRU2MDP.h"

class PRU2MDPprogress;

class PRU2MDPactionDescriptor{
 public:
  const PRU2MDPprogress *progress;
  const MDPaction *action;
  PRU2MDPactionDescriptor( const PRU2MDPprogress *prog, const MDPaction *act) {
    progress = prog;
    action = act;
  }
};

#endif
