# Represents the feedback for executing some policy

string goal_name # the name of current goal
string state     # the name of the MDP state where the feeback occurs
string action    # the name of the action that took place there
string param     # the parameter of the action if any
string feedback  # a decription of what happened.
                 # SUCCESS for a successful action
		 # the reason of failure otherwise
