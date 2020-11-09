# StickyFingers Plugin #


[![Build Status](https://travis-ci.org/cwru-robotics/cwru_stickyfingers.svg?branch=master)](https://travis-ci.org/cwru-robotics/cwru_stickyfingers)

### What Is StickyFingers? ###

StickyFingers is a ROS plugin meant to assist in grasping objects or representing a magnetic, suction, or adhesive gripping element. It makes one link in a Gazebo model “sticky”- when enabled by a ROS message, if that link comes into contact with an object that object will become “stuck” to the link and follow its position until the disable message is sent.
	
### Employing StickyFingers ###

To add a StickyFingers plugin to a link in your robot, add the following material to the *model* definition containing the link you want to make sticky:<pre><code>&lt;plugin name="<strong>[NAME 1]</strong>" filename="libsticky_fingers.so">
&nbsp;&nbsp;&nbsp;&nbsp;&lt;capacity><strong>[##]</strong>&lt;/capacity>
&nbsp;&nbsp;&nbsp;&nbsp;&lt;link><strong>[LINK_NAME]</strong>&lt;/link>
&lt;/plugin></code></pre>
**[NAME 1]** can be anything.
**[##]** is the maximum mass the gripper should be able to lift- anything above this value, and any static objects, will not attach to the gripper.
**[LINK NAME]** is the _fully qualified Gazebo name_ of the link you want to make sticky. This can be found through the "models" subsection of the "world" tab in the Gazebo interface- select the link in question, and look at the "name" field.

An example of this structure can be found in `world/blocks_on_table.world`.

Note that in a URDF file, like any plugin definition you will need to include the stickyfingers plugin in *unnamed* Gazebo tags, in the main robot definition (and inside the macro, if one is used) outside of all of the links:<pre><code>&lt;robot name="some_name" xmlns:xacro="<http://www.ros.org/wiki/xacro>">
&nbsp;&nbsp;&nbsp;&nbsp;&lt;xacro:macro name="something" params="something_else">
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;link name="<strong>[LINK_NAME]</strong>">
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;... 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;/link>
		
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;gazebo>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;plugin name="<strong>[NAME 1]</strong>" filename="libsticky_fingers.so">
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;capacity><strong>[##]</strong>&lt;/capacity>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;link><strong>parent::other_parent::[LINK_NAME]</strong>&lt;/link>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;/plugin>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;/gazebo>
&nbsp;&nbsp;&nbsp;&nbsp;&lt;/xacro:macro>
&lt;/robot></pre></code>
### Controlling StickyFingers ###

On startup, every StickyFingers link offers a <a href="http://wiki.ros.org/Services">ROS service</a> that communicates with <a href="http://docs.ros.org/jade/api/std_srvs/html/srv/SetBool.html">std_srvs/SetBool</a> services.

The messages are on topics named <code>sticky_finger/<strong>[NAME 1]</strong></code>- for instance, if we had a finger named `ftip_sticky`, we would call the service `sticky_finger/ftip_sticky`. The console will display the messages used by each sticky finger in the simulation whenever Gazebo starts up.

A `True` value will enable the finger, a `False` value will disable a finger and cause it to drop whatever it is holding.
A stand-alone executable to produce these messages is included as `finger_control_dummy_node`.
