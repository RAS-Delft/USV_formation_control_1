# USV_formation_control_1

Repository regarding the formation control algorythm developed for the AVATAR project
This project requires some other (RAS) modules as well (e.g. velocity controller, speed differentiator, heading controller).
Software specific for this formation control setup is found here

Comments, questions: Bart Boogmans - email: b.boogmans@tudelft.nl

## Main functionality

There is a virtual formation reference that moves alsong a path. Each ship finds a target point depending on it's position within formation frame. Vessels have parralel distance keeping and heading control to yield formation keeping.

The block scheme is as follows. 
![dda drawio (2)](https://github.com/RAS-Delft/USV_formation_control_1/assets/5917472/d4f4ab8a-a080-4fc4-90ac-cef6f217103d)

The image below deplicts a moving reference formation along a path:
<add>

The image below depicts three ships aiming at their respective targetpoints. 
<add>
