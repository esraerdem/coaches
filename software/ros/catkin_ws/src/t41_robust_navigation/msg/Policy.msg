# represents the policy of a PRU for going to somewhere and activate a task

string        goal_name    # the name of the goal this policy solves
string        final_state  # the name of the location to end this PRU
StatePolicy[] policy       # a pair (location, action) for each state
	      		   # location "CURRENT" means current position
