/**
 * Receives Laser scans and camera images.
 * Sends features and robot location.
 */

#include "T41.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t41");

  ros::NodeHandle node;

  T41 t41(node);
  t41.run();

  return 0;
}

