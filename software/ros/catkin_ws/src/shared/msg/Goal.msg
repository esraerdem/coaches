# a goal the robot could reach
string loc	 # a unique identifier for this goal's location
string kind	 # describes what has to be done in loc to get value
       		 # (advertise, look, help, escort, ...)
string param	 # additional value for this goal (what to advertise, ...)
float32 value 	 # an evaluation of this goal's interest
float32 duration # the expected duration of the task
