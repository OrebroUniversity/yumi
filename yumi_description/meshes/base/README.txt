PROGRAMMER: Frederick Wachter
DATE CREATED: 2016-05-19
LAST MODIFIED: 2016-05-19
PURPOSE: Explains YuMi stand STL file errors and how to compensate for them

Issue: 
 - RViz has issues with STL files exported from SolidWorks due to the convention solidworks uses for their first line in STL files. More specifically, SolidWorks puts the word "solid" as the first characters in the STL file, which RViz assumes the file is an ASCII STL file instead of a binary STL file. This causes warnings to pop up when starting RViz and eventually states that there must be some error in the definition of the STL file. 

Work Around: 
 - Take the original file from SolidWorks and save it as a STEP file.
 - Import the STEP file into a free CAD software called FreeCAD
     NOTE: FreeCAD can only export in the units of milimeters, which is why the file needs be scaled as stated below
 - The file needs to be scaled by 0.001 on each axis, this can be done using the walkthrough in this repo
     NOTE: The walkthrough can be found at misc/FreeCAD/scaleSTL.py
           Scaling will mess up the origin, follow the step below to fix this issue
 - Export the file as mentioned in the walkthrough, and this will remove the [WARN] issue with RViz
 - Since the origin is generally changed through this process, load this file back into SolidWorks
     NOTE: To load into SolidWorks properly, do the following:
            - Click open and find the STL file
            - On the right hand size, choose the file type as ".stl"
            - Click the options button, and in the "Input As" seciton choosed "Solid Body"
            - Click ok, then click open
 - Get the distance from the desired origin position to the current origin
 - Write down the <dx,dy,dz> values and use those as the <origin_offset> variable below
 

/* Variable Names to Change in URDF Below*/

<object> - string - the name of the object
<object>_link - string - can use the same name for object as in the previous variable
<file_location> - string - folder location that contains the STL file
<file_name> - string - name of the STL file
<origin_offset> - "dx dy dz" - offset retrieved from SolidWorks, replace dx, dy, and dx appropriately


/* Example Code for URDF */

<link name="<object>"/>
<link name="<object>_link">
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://<file_location>/<file_name>.stl"/>
    </geometry>
    <material name="Orange"/>
  </visual>
  <collision>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://<file_location>/<file_name>.stl"/>
    </geometry>
  </collision>
</link>
<joint name="<object>_link_to_<object>"" type="fixed">
  <parent link="<object>"/>
  <child link="<object>_link"/>
  <origin rpy="0 0 0" xyz="<origin_offset>"/>
</joint>
<joint name="<object>_to_world" type="fixed">
  <parent link="world"/>
  <child link="<object>"/>
  <origin rpy="0 0 0" xyz="0 0 0"/>
</joint>


