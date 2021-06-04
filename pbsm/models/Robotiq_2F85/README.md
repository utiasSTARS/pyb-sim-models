# Robotiq 2F-85 Model
Although the gripper is heavily used in academia and industry, nearly all simulation models for this gripper that are available online are wrong or simply not working. This work tries to fill the gap by providing the community with a realistic model of the gripper.

This model was produced with the following steps:
1. Downloading the STEP (volumes) files from Robotiq's website.
1. Using Solidworks to divide the gripper into parts.
1. Specify the right material properties for the parts.
1. Using Solidworks to create a constrained assembly from the parts.
1. Using [Solidworks to URDF exporter](https://github.com/ros/solidworks_urdf_exporter) to generate the URDF.
1. Using the CAD to locate the position of the additional constraints that needs to be added with PyBullet.
1. Implement the additional constraints in PyBullet.

Consequently, this model must be used with PyBullet. The URDF could be used with Gazebo as long as the additional constraints are correctly implemented in the simulator.

## Mechanical properties and constraints
The two-finger gripper is an adaptive end-effector as the configuration of the fingertips adapts to the shape of the object being grasped. This behaviour is made possible by the underactuation of the fingers which is made possible by the mechanical design of the gripper. As a result, a realistic simulation of the gripper is only possible if all the constraints of the gripper are respected and correctly implemented.

The gripper is made of the following parts:

![2F85_Parts](https://user-images.githubusercontent.com/10478385/103466825-9a671c00-4d16-11eb-8221-638f7d2018f3.png)

Each finger is made of 5 parts: driver, coupler, follower, pad and spring link. The driver is the only link that is directly actuated with a motor.

### Closed Kinematic Loop
The URDF file format specifies a model as a tree. Consequently, it does not support closed kinematic loops. Clearly, each finger of the 2F-85 forms a closed kinematic loop. As a result, at least one constraint for each loop needs to be specified outside of the URDF file. The constraint we chose to specify outside the URDF is the joint between the spring link and the follower. In PyBullet, the type of joint is called JOINT_POINT2POINT and corresponds to a revolute URDF joint. The location of the joint axis must be specicfied relative to the center of mass of the link. Using the CAD, we locate the axis as such:

![Additional_Constraint](https://user-images.githubusercontent.com/10478385/103467080-398d1300-4d19-11eb-8151-f45b90a7139a.png)

And the constraint can be implemented as such:
```python
p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=right_follower_index, childBodyUniqueId=self.getUID(), childLinkIndex=right_springlink_index, jointType=p.JOINT_POINT2POINT, jointAxis=[1, 0, 0], parentFramePosition=[6/1000,-6.7/1000,0], childFramePosition=[0,28.9/1000,0])
```

### Spring Links
When the fingertips are not in contact with any object, the pads are supposed to be parallel. To allow the gripper to exhibit this behavior, springs located at the joint between the base and the spring link must be simulated. Some simulators, like Gazebo,  allow the user to specify a spring stiffness constant for a joint but PyBullet does not currently have this capability. Consequently, the joint is motorized with a very weak force such that the spring links are constantly trying to close toward the palm. This simulated spring can be implemented with the following code:
```python
p.setJointMotorControl2(targetPosition=self.MAX_CLOSING, bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_spring_link_joint'), controlMode=p.POSITION_CONTROL, force=1)
``` 

### Limit Pins
Another requirement to allow the fingertips to stay parallel is to enforce the constraint imposed by the pin attached to the driver link. This pin limits the movement of the coupler in one direction. Consequently, the fingertips can close toward the palm but cannot open outward. One way of enforcing this constraint would be to enable the collision checking between different parts of the same robot but that would be computationally very expensive and might produce unstable simulations. Another way is to specify a maximum angle that the driver-coupler joint can reach. Finally, a third way could disable the movement of the driver-coupler joint altogether. This method would force the fingertips to stay parallel at all times and would loose the adaptive capability of the gripper.

Since, for some simulations, it might be acceptable to loose the underactuation of the gripper and keep the pads parallel at all times, we allow the user to set an XACRO parameter that would disable the underactiation. The ```enable_underactuation``` parameter can be set to ```false``` in the XACRO files to disable the driver-coupler joint and force the fingertips to stay parallel:

```xml
<xacro:Robotiq_2F85 parent="world" name="gripper" enable_underactuation="true" soft_pads="false" softness="10" >
<origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:Robotiq_2F85>
```
Otherwise, the constraint of the pin is modeled using a joint limit. This limit can be specified in the URDF but for a reason we still ignore, the limit needs to be also specified in PyBullet for it to work:

```python
p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_coupler_joint'), jointLowerLimit=0.13 , jointUpperLimit=0.79)
```

### Gears
The real gripper always closes symmetrically and a blocking mechanism make it so that no holding torque is required once a position is reached. PyBullet supports the JOINT_GEAR joint type which can actuate one joint when another joint moves. This works well for some situations but problems arise when an external force is applied on one fingertip. To achieve a more realistic result, this type of constraint is not used and each finger is actuated separately with two different simulated motors. A parameter can be set by the user such that only one actuator is used and that the JOINT_GEAR constraint is added:
```python
if self.USE_SINGLE_ACTUATOR == True:
            right_driver_joint_index   = self.getJointIndexFromName('gripper_right_driver_joint')
            left_driver_joint_index    = self.getJointIndexFromName('gripper_left_driver_joint')
            #A JOINT_GEAR is unidirectional so two constraints needs to be added to have the real gear behaviour
            c = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=right_driver_joint_index, 
                childBodyUniqueId=self.getUID(), childLinkIndex=left_driver_joint_index,  jointType=p.JOINT_GEAR, 
                jointAxis=[0,0,1], parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
            p.changeConstraint(c, gearRatio=-1, maxForce=self.MAX_FORCE)
            c = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=left_driver_joint_index, 
                childBodyUniqueId=self.getUID(), childLinkIndex=right_driver_joint_index,  jointType=p.JOINT_GEAR, 
                jointAxis=[0,0,1], parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
            p.changeConstraint(c, gearRatio=-1, maxForce=self.MAX_FORCE)
```
The blocking mechanism is not yet implemented but could be done by changing the maximum force exerted by the actuators when the gripper is not moving to something very large.

### Materials, Masses and Inertias
Most of the parts of the gripper are made out of anodized aluminum 6061 T6 which has a mass density of 0.0027 Kg/cm^3. The spring links are made out of stainless steel 17-4 ph which has a mass density of 0.0028 Kg/cm^3. Finally, the pads are made out of silicon 60A which has a mass density of 0.0023 Kg/cm^3. Combining these details with the shapes of the parts allow us to accurately compute the masses and inertias of each component. Doing so is of the highest importance as incorrect inertias will lead to unrealistic simulations and could even lead to unstability issues.

Depending on the type of pad that is used, the pad might be somewhat compliant and slightly deform when grasping an object but PyBullet does not seems to officially support soft bodies. To allow for some compliance an XACRO parameter ```soft_pads``` can be set to ```true```. Doing so creates a linear joint that allow the pad to slightly penetrate in the link that supports it. To model the stiffness of the pad, a second parameter ```softness``` can be used to set the force of the linear joint.
```xml
<xacro:Robotiq_2F85 parent="world" name="gripper" enable_underactuation="true" soft_pads="false" softness="10" >
<origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:Robotiq_2F85>
```