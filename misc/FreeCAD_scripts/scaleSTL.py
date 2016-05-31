# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-19
# LAST MODIFIED: 2016-05-19
# PURPOSE: Script used in FreeCAD to scale STL files
# REFERENCE: http://forum.freecadweb.org/viewtopic.php?t=4774

# DIRECTIONS: (Will work Mac, not sure about other operating systems)
#  - Open FreeCAD software
#      NOTE: Close and reopen if already opened to ensure the active document is the document that you would like to scale
#  - Open STL file that you would like to scale
#  - Open Python Console (View > Views > Python Console)
#  - Type into the console the code below
#  - Click the newly created mesh that was brought into the application to highlight it
#      NOTE: should be called "Mesh"
#  - Click Export (File > Export)
#  - Save the file as <name>.stl in the desired directory
#      NOTE: <name> is whatever name you want to save it as

# ---------------------------------------------------------------------
# IMPORTANT - NOTE: Scaling the file will most likely affect the origin location on your model. It is difficult to avoid this. To work around this, look at the walkthrough at "yumi_description/meshes/base/README.txt"
# ---------------------------------------------------------------------

# Asumming desired scaling of <0.001,0.001,0.001> meters
import Mesh, BuildRegularGeoms # import necessary libraries

scaleMatrix = FreeCAD.Matrix() # create matrix to transform an object
scaleMatrix.scale(0.001,0.001,0.001) # set the scale factor for the desired STL filed

stlMesh = App.ActiveDocument.ActiveObject.Mesh.copy() # get mesh of active object
stlMesh.transform(scaleMatrix) # scale the mesh of the active object

Mesh.show(stlMesh) # bring the mesh into the application





