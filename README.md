# ZumoBot
GitHub Repo for the ZumoBot Project with Embedded Software, Deep Learning Models, ....
<br>

About the Project: 
This Project aims to develop a fully autonomous, self driving vehicle which is able to knock out an enemy vehice inside a "sumo ring" by pushing the enemy vehicle out
of the marked area. The vehicle features a Intel RealSense Depth Cam, which will detect the enemy in 3D using Deep Learning (3D-CNN) and attack weakpoints of the vehicle in order to push out the enemy (Anomaly detection - Autoencoder). 
<br>
Modules: 
<ul>
  <li>Base: Raspberry Pi 4</li>
  <li>Camera: Intel RealSense L515 or D435</li>
  <li>Edge/Ring avoidance: IR-Sensor</li>
  <li>Distance and Collision avoidance from the side: Ultrasonic Sensors x 7</li>
  <li>Motors: 2x DC-Motors + Driver</li>
</ul>
<br>
All modules will be placed onto a Motherboard, which will feature all Pins and Plugs for the modules above
