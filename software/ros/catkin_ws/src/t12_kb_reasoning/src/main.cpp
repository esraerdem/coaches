/**
 * Integrates knowledge and events into the database
 * Receives knowledge from t11, environment events from t22 and human needs events from t32
 * Sends goals
 */

#include "T12.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t12");

  ros::NodeHandle node;

  T12 t12(node);
  t12.run();

  return 0;
}

