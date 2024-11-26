# CageModeler

## Getting started

CageModeler utilizes CMake for project file generation, vcpkg for managing external dependencies, and CMake presets 
to streamline configurations across major platforms - Windows, Linux, and macOS. For Linux, ensure the presence of
`zip` for vcpkg functionality and `libomp-dev` as an external dependency. On macOS, `libomp` can be installed via 
Homebrew. 

Set the environment variable `CUDA_HOME` to point to the CUDA installation directory (e.g. `/usr/local/cuda`).
Ensure that `LD_LIBRARY_PATH` contains `$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64` and `PATH` includes `$CUDA_HOME/bin`
for the CMake scripts to locate the CUDA installation.

## Cloning

Cloning the repository necessitates a recursive fetch of all submodules:

```
git clone git@github.com:DanStroeter/CageModeler.git
git submodule update --init --recursive
```

## Available presets

The following presets are defined and can be utilized either on a build server or a local machine corresponding to the operating system:

- `windows-msvc-debug` - <strong>Windows</strong> using <strong>MSVC</strong> in debug mode.
- `windows-msvc-release` - <strong>Windows</strong> using <strong>MSVC</strong> in release mode.
- `unix-clang-debug` - <strong>Linux</strong> using <strong>clang</strong> in debug mode.
- `unix-clang-release` - <strong>Linux</strong> using <strong>clang</strong> in release mode.
- `unix-gcc-debug` - <strong>Linux</strong> using <strong>gcc</strong> in debug mode.
- `unix-gcc-release` - <strong>Linux</strong> using <strong>gcc</strong> in release mode.
- `macos-debug` - <strong>macOS</strong> in debug mode using <strong>clang</strong> which is bundled with Xcode and the command line tools.
- `macos-release` - <strong>macOS</strong> in release mode using <strong>clang</strong> which is bundled with Xcode and the command line tools.

## Generating the project files

Generating project files is straightforward:
```
mkdir build
cd build
cmake .. --preset=<Your Preset Name>
```

## Main Functions of Editor.cpp
RecordUI
Design main work space if the app using ImGui, including 
* mainMenuBar
* statusBar
*  toolBar
* projectOperationPanel
* projectSettingPanel
* newProjectPanel
Also will initialize rectangle selection if there is one

Update
Is in the main thread
1. Get the current camera viewInfo and turn it into internal matrix and apply it to every mesh (based on the changes made by gizmo)
2. Change the status of gizmo object(highlight when select and click a direction)
3. Process the current mouse action (if the mouse is selecting vertex, edge or polygon, and make reaction: highlight)
4. Mark the dirty

UpdateMeshSelection
Process the current mouse action :click (if the mouse is selecting vertex, edge or polygon, and make reaction: highlight) ,other actions like dragging are not processed

SetUpUIElement
1. Get the projectModelData
2. Set up the statusBar(bottom), ToolBar(left top), projectOptionsPanel(right)

OnNewProjectCancelled
Seems has bug

OnProjectSettingsApplied
Change the project setting in the project options panel and click the “Apply” button, the new model data will be set by calling OnNewProjectCreated

OnNewProjectCreated
React to the action of clicking “create” in the “new project”panel.
1. Get a thread from thread pool as the main thread, assign this operation as a task of the main thread
2.  Get the data from the panel input by user and call “Create Project”,save the creation result into projectResult
3. Check if the creation succeed
4. Compute the weight(out of main thread), and check the error
5. Update the weight data from the weighting result
6. Compute the deformed mesh from projectResult and update the deformationData
7. Remove the old mesh in the scene
8. Get the value of projectResult as projectData
9. Calculate the translation and scale matrix from the projectData, and combine them into newModelMatrix
10. Add new mesh and cage to the scene whose data is from projectData, and make them follow the shape of the model by applying newModelMatrix
11. Update the vertex color of the mesh
12. Update the setting panel, ToolBar and Status Bar
13. update vertex data
14. Rebuild the BVH
15. Get ready to dismiss the new project panel and project setting panel

UpdateGizmoSelection
Check if a gizmo is selected, if so, set it highlight

OnClicked
Handle the click event 
* Keydown
     OnMouseClickPressed(): analysis if the user is clicking a gizmo or the mesh element(vertex/edge/polygon)or making a rectangle selection
* KeyPressed
     OnMouseMoved(): analysis if trigger the rectangle selection/gizmo transformation according to the displacement of mouse
* KeyUp
     OnMouseClickReleased():
       If the event is from rectangle selection: Cache the projected vertices into screen space, and get the selection type (vertex/edge/polygon)from StatusBar
       If the event is from Gizmo Transformation, recompute the deformed mesh and update the render proxy
       If the event os from click selection: check the selection type(vertex/edge/polygon) then make the element highlight
      Reset the event status

OnToolSelectionChanged
  Set the visibility of the gizmos based on the selected tool from the tool bar

OnSelectionTypeChanged
  If the selection type change,  set the wireframe render mode to the deformedMesh according to the new type

OnSequencerFrameIndexChanged
  Update the deformed mesh according to the new data of frameIndex

OnSequencerNumFramesChanged
  Do the deformation operation when the maximum number of frames in the sequencer changes

UpdateDeformedMeshPositionsFromDeformationData
  Be called to update the deformed mesh according to the deformation data extracted from frameIndex

ExportCurrentDeformedMesh
  Export deformed mesh data of current frame to the filename

ExportDeformedMeshes
  Export all deformed mesh data to the filename

ResetGizmoPositionFromSelection
  Get the current vertex selection from different selection type (vertex/edge/polygon) and calculate its average position, then apply the average position to the gizmo to update the position of gizmo
