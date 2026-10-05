<p align="center">
  <img src="assets/canard-view.avif" width="260">
  <img src="assets/skyview.avif" width="260">
  <img src="assets/artillery-impact.avif" width="260">
</p>



<p align="center">
  <strong>155-mm-artillery-shell</strong><br>
  Project repo for <em>Smart India Hackathon 2026</em><br>
  Problem Statement:<br>
  <em>Development of a Low-Cost Precision Guidance and Smart Electronic Fuze System for a 155 mm Artillery Shell</em><br>
  PS No: SIH26098
</p>

<table align="center">
<tr>
<td width="50%">
<img src="assets/canard-demo.avif" width="100%">
</td>
<td width="50%" valign="middle">
  
## Canard Control
The canard actuation system uses four servo motors positioned at 90° intervals to control the shell’s aerodynamic surfaces.
An IMU provides orientation data, allowing the control system to detect deviation from the intended trajectory and adjust the canards accordingly.

</td>
</tr>
</table>



<table align="center">
<tr>
<td width="50%">
<img src="assets/fuze-system.avif" width="100%">
</td>
<td width="50%" valign="middle">
  
## Fuze System
The multi-modal electronic fuze uses proximity and impact sensors to detect the selected terminal condition.
A microcontroller processes these inputs and switches between proximity, impact, and timed demonstration modes.
</td>
</tr>
</table>


<table align="center">
<tr>
<td width="50%">
<img src="assets/digital-demo.avif" width="100%">
</td>
<td width="50%" valign="middle">
  
## HTML Demo
Real-time sensor data from the guidance system is transmitted to an interactive HTML dashboard, where the artillery shell's orientation and canard movement are visualized.
The dashboard provides a live representation of the system's sensor readings and actuator state during operation.
</td>
</tr>
</table>


<img align="left" width="260" height="200" src="assets/cattgit.avif">

<div align="center">

### Core Components

<table>
  <tr>
    <td align="center">
      <a href="6DOF.py"><strong>6DOF.py</strong></a><br>
      <sub>Simulates the actual and ideal flight trajectories of the artillery shell.</sub>
    </td>
  </tr>

  <tr>
    <td align="center">
      <a href="artillery_viewer.html"><strong>artillery_viewer.html</strong></a><br>
      <sub>Interactive dashboard that mirrors the orientation of the shell during flight.</sub>
    </td>
  </tr>

  <tr>
    <td align="center">
      <a href="final.cpp"><strong>final.cpp</strong></a><br>
      <sub>Microcontroller firmware for the canard actuation and electronic fuze systems.</sub>
    </td>
  </tr>
</table>

</div>

<br clear="all">




