/**
 * Deals with Human Robot Interaction.
 * Sends features and HRI actuations
 */

#include "T11.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t11");

  ros::NodeHandle node;

  T11 t11(node);
  t11.run();

  return 0;
}

