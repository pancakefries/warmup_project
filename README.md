# Writeup
## drive-square.py
### Description
My approach to make the turtlebot move in a square involved using timing to alternate between moving forward and turning the bot. I used r.sleep() to wait 5 seconds between each command, alternating between moving forward and turning 90 degrees.

### Code explanation
``__init__()`` initializes ``/cmd_vel`` and sets the variables ``self.publisher``, ``self.rate``, ``self.lin_speed``, and ``self.ang_speed``. They set the publisher, the speed at which the messages are alternated between, the speed at which the bot moves when it moves forward, and the speed at which the bot turns respectively. 
``run()`` creates vectors using the previously defined ``self.lin_speed`` and ``self.ang_speed``, which it then combines into turning and moving messages. After the definitions, it has a while loop (while rospy is still running) which publishes the turning and moving messages, alternating between each iteration, then waits for 5 seconds before publishing the next one.

## Challenges
The biggest challenge that I faced when writing this program was getting the turtlebot to turn 90 degrees. After several attempts of trial and error with more or less random numbers, I realized I should calculate the angular velocity needed instead of guessing. I determined a value of 0.314 rad/s over a period of 5 seconds to get a final turn of pi/2 radians. 

## Future work
If I had more time to improve my code, I would probably look into using ``/odom`` instead of timing, as was mentioned in the instructions. Due to imperfections in real world conditions, as well as rounding the angular velocity, the square traced out using my code will drift over time. This is an obvious area of improvement that would make a big difference in the effectiveness and elegance of my code. 

## Takeaways
- I feel that I now have a better understanding of how to implement OOP techniques into the code I write using rospy. This will make future code that I write much more efficient and easier to read. Additionally, using OOP allows the code to be a bit more modular and easier for others to understand/collaborate on given its superior structure/order in more complex projects.
- From the challenge that I mentioned above, it can be valuable to take a step back from what you're coding and reassess. In my case, I was so consumed with trying to find the best value for ``self.ang_speed`` that I missed the fact that it could be easily calculated. Taking a step back to reassess in this case led to my code being more accurate and my methods being more efficient.
