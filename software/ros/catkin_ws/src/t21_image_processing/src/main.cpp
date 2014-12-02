/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
 */

#include "T21.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t21");

  ros::NodeHandle node;

  T21 t21(node);
  t21.run();

  return 0;
}

