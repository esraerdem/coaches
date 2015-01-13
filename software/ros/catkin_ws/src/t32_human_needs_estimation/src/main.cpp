/**
 * Receives features from HRI and from people detection.
 * Sends (human needs) events.
 */

#include "T32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t32");

  ros::NodeHandle node;

  T32 t32(node);
  t32.run();

  return 0;
}

