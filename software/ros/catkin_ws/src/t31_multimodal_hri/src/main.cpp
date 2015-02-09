/**
 * Deals with Human Robot Interaction.
 * Sends features and HRI actuations
 */

#include "T31.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t31");

  ros::NodeHandle node;

  T31 t31(node);
  t31.run();

  return 0;
}

