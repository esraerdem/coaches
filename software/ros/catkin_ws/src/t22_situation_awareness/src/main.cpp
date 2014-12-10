/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
 */

#include "T22.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t22");

  ros::NodeHandle node;

  T22 t22(node);
  t22.run();

  return 0;
}

