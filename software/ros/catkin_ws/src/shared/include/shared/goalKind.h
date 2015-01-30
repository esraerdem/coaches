/* constants for identifying various goal kinds */

const std::string GOAL_PATROL = "patrol";        // Requires the robot to go to _loc_
const std::string GOAL_INTERACT = "interact";    // Requires the robot to go to _loc_ and interact with _param_
const std::string GOAL_ADVERTISE = "advertise";  // Requires the robot to go to _loc_ and display ad _param_
const std::string GOAL_INSPECT = "inspect";      // Requires the robot to go to _loc_ and take a picture
const std::string GOAL_ESCORT = "escort";        // Requires the robot to initiate escort _param_ to _loc_
const std::string GOAL_FOLLOW = "follow";        // Requires the robot to ask _param_ following the robot to _loc_
const std::string GOAL_ESCORT2 = "escort2";      // Requires the robot to start escorting _param_ to _loc_
const std::string GOAL_DONE = "done";            // Requires the robot to acknowledge the end of the escort to _param_

