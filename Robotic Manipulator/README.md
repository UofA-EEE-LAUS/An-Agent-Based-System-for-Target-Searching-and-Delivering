The two .ttt files given are models for which code needs to be inserted for a specific application.

The 2DOF arm can be attached to a 3DOF rover's side. The code is now written such that the it reaches down and comes back to its original position after a certain period. Please modify the code according to your needs.

For the 6DOF arm design, the arm can be attached to a top of the rover. Currently the settings are provided such that it picks up a dummy target in front of it. To find out detail explanations regarding IK Groups and "fake pickups" please vist the coppeliasim website. The settings for the fake pick up can be removed and the joints can be coded as to user's needs. However, a realistic pickup using a gripper is quite dificult to achieve and show unrealistic behaviour in most cases. It is recommended that the other joints are coded and a fake pickup is used to link the obect to a dummy and simulating a fake pickup.

https://www.youtube.com/watch?v=JUiSZinyH1c Here is a link to an indepth 2-part tutorial explaining inverse kinematics and achieving fake pickups.
