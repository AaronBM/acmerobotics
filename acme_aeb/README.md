# acmerobotics
Repo for SAE Robotics Bootcamp

20210622   -  Initial conception due to unrecoverable state of ACME_Robotics
           -  Modification of launch file to include all required components.

**author**-  Aaron 

**author**-  CJ
**author**-  Mark
**author**-  Erika

*Gracious Executive Consultant* - Huzefa



SAE Robotics BootCamp Summer 2021
https://sae.instructure.com/courses/143/assignments/259?module_item_id=821


============Automatic Emergency Braking (AEB) INSTRUCTIONS========================

1) Navigate to your catkin workspace and add remote repository from git
```
$ git clone https://github.com/AaronBM/acmerobotics
```
2) Refresh catkin.  Move up one directory from toplevel src file and run:

```
$ catkin_make
```
3) Run the launchfile
```
$ roslaunch acme_aeb aeb.launch
```

4) Revel in the glory of an uncrashed car.
🎉 🎉 🎉

5) Invest in ACME Robotics.  Bitcoin accepted
🚀

------------Methodology----------------------------------------------
The algorithm searches for the straight ahead angle and returns the average
distance measured in a 4 degree cone.  This distance is then fed into a 
velocity controller with the following logic.
  1) If the error (current distance - threshold value of 3 m) multiplied by the gain is greater than the max velocity, then command max velocity
  2) If the error multiplied by the gain is less than the max velocity, command a velocity equal to the error multiplied by the gain
  3) If the error multiplied by the gain is negative, command zero velocity.

