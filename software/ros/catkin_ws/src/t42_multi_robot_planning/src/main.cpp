/**
 * Receives sets of goals
 * Sends navaigation goals and HRI goals
 */

#include "T42.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t42");

  ros::NodeHandle node;

  T42 t42(node);
  t42.run();

  return 0;
}

