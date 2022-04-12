# Writeup
## drive-square.py
### Description
My approach to make the turtlebot move in a square involved using timing to alternate between moving forward and turning the bot. I used r.sleep() to wait 5 seconds between each command, alternating between moving forward and turning 90 degrees.

### Code explanation
``__init__()`` initializes ``/cmd_vel`` and sets the variables ``self.publisher``, ``self.rate``, ``self.lin_speed``, and ``self.ang_speed``. They set the publisher, the speed at which the messages are alternated between, the speed at which the bot moves when it moves forward, and the speed at which the bot turns respectively. 
``run()`` creates vectors using the previously defined ``self.lin_speed`` and ``self.ang_speed``, which it then combines into turning and moving messages. After the definitions, it has a while loop (while rospy is still running) which publishes the turning and moving messages, alternating between each iteration, then waits for 5 seconds before publishing the next one.

### GIF
![Square drive GIF](/gifs/square.gif)

## Person Follower
### Description
My approach was to divide the LiDAR sensor into left and right halves, determine which side had a closer object, and turn in that direction. I also scaled the linear speed so that the turtlebot would slowly come to a stop in front of the person/object.

### Code Explanation
`__init__()` initializes the Subscriber and Publisher to `scan` and `cmd_vel` respectively, as well as sets the maximum linear speed, maximum angular speed, and the distance from the person that it should stop at. 
`scan_callback()` splits the sensor data into left and right halves by dividing the length of `ranges` by 2. It then takes each data point and makes a list of the detected objects on each side. Using these lists, it finds the minimum on each side and compares the two values to determine which side is closer to the person. It uses that information to tell the turtlebot which direction to turn and how fast to go, the latter depending on how far away the person is.

### GIF
![Person follower GIF](/gifs/person_follower.gif)

## Wall Follower
### Description
My approach was similar to Person Follower, except that this time the sensor data is split into front and back sections instead of left and right. It determines which half is closer to the wall and uses that info to turn towards or away from the wall while maintaining distance.

### Code Explanation
`__init__()` initializes the Subscriber and Publisher, as well as sets the maximum linear and angular speeds.
``scan_callback()`` splits the LiDAR sensor data into 2 halves, front and back, splitting at 1/4 the length of `ranges` and 3/4 the length of `ranges`. Using this split, it takes each data point and creates lists of the distances on each half. Using the lists, it finds the minimum of each and compares them to determine which half of the turtlebot is closer to the wall. It then turns towards or away from the wall to follow and maintain distance.

### GIF
![Wall follower GIF](/gifs/wall.gif)


## Challenges
The biggest challenge that I faced when writing drive-square was getting the turtlebot to turn 90 degrees. After several attempts of trial and error with more or less random numbers, I realized I should calculate the angular velocity needed instead of guessing. 
When coding the person follower I had a tough time figuring out how to deal with significant jitter I was getting when the turtlebot was travelling what should have been a straight line. By adding a threshold in order for the bot to turn, I was able to resolve the issue. 
The biggest challenge that I faced when writing the wall follower was without a doubt trying to get it to maintain a relatively consistent distance as it followed the wall. I added a threshold like I did with the person follower, which helped but did not resolve it completely. It was only after quite some trial and error that I realized I could add a second conditional to account for the robot getting too close to the wall. This allowed the turtlebot to maintain a relatively consistent distance from the wall.

## Future work
If I had more time to improve drive-square, I would probably look into using ``/odom`` instead of timing, as was mentioned in the instructions. Due to imperfections in real world conditions, as well as rounding the angular velocity, the square traced out using my code will drift over time so this would be a big improvement.
If I had more time to improve the person follower, I would look into scaling the linear speed by a logarithm instead of linearly, so that the robot stays fast unless it's close to the person. I would also probably look into scaling the angular speed so that it turns faster when you stand behind it and slower when you stand in front.
If I had more time to improve the wall follower, I would try to improve the consistency of the distance to the wall that the robot maintains. Right now, it seems to sometimes drift away from the wall, so I would like to add additional checks to ensure that it turns back to where it should be more conssitently

## Takeaways
- I feel that I now have a better understanding of how to implement OOP techniques into the code I write using rospy. This will make future code that I write much more efficient and easier to read. Additionally, using OOP allows the code to be a bit more modular and easier for others to understand/collaborate on given its superior structure/order in more complex projects.
- From one of the challenges that I mentioned above, it can be valuable to take a step back from what you're coding and reassess. In my case, I was so consumed with trying to find the best value for ``self.ang_speed`` that I missed the fact that it could be easily calculated. Taking a step back to reassess in this case led to my code being more accurate and my methods being more efficient.
