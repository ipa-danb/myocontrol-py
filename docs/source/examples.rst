Examples
==========

pyGUI
------

Simple GUI to work with the displacmeent controller.

 .. figure:: pyGUI_Orthosis.jpg
    :scale: 50%

    Screenshot of pyGUI


To start it run
::

  $ rosrun myo_py pyGUI_dispControl.py

On the wheel you can set the reference value based on maximum and minimum
allowed values.

On the left you can see the how the reference value compares to the min-max values

On the right you can decouple the clutch and log all states for a short time.
(careful, as log size increases fast!)
