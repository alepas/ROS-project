<h1> ROS-project</h1>
This repo has been created for the project associated to the Robotics course (2018/2019) at Politecnico di Milano.

<h2> Project Request </h2>
<ul>
  <li><em>Compute two different odometry and publish both as tf and odom topic</em>:<br>
    <ul>
      <li>using Differential Drive Kinematics</li>
      <li>using Ackerman model</li>
    </ul>
  <li> Use dynamic reconfigure to switch between different published odometry.</li>
  <li>Use dynamic reconfigure to reset the odometry to (0,0) or to set to a
  specific starting point (x,y).</li>
  <li>Publish a custom message with odometry value and type of source.</li>
</ul>  

<h2>Commands</h2>
In order to run this project it must be insert into a ROS envirnoment and the following commands must me applied into 4 different prompts:
<ul>
  <li> in the first prompt start ROS server by typing <code> roscore </code></li>
  <li> in the second one do the following commands:
    <ul>
      <li><code>cd ~/catkin_ws/</code> which is the ROS environment folder</li>
      <li><code>catkin_make</code> to build up everything</li>
      <li><code>rosrun project reader</code> to run the package and the reader node</li>
    </ul>
  </li>  
  <li> in the third one open the graphical tool required to make dynamic_reconfigure work properly with <br>
  <code> rosrun rqt_reconfigure rqt_reconfigure</code></li> 
  <li> the last one plays the bag via <code>rosbag play bag_1.bag</code></li>
</ul>
