
# Object Spawner Manager

## 1. Description
Blalba

## 2. Concept of the Object Spawner Manger

## 3. Installation 
To be able to execute the objtect_spawner_manager, you need to follow the process for installation described on the top level of this repository. 

## 4. Messages
Messages and Services are sourced from the spawn_object_interfaces package.
```
ObjectMsg
```
* `obj_name`: tbd
* `parent_frame`: tbd
* `pose` (Pose): tbd
* `cad_data` (str): tbd
```
RefFrameMsg
```
* `frame_name`: tbd
* `parent_frame`: tbd
* `pose` (Pose): tbd
## 5. Services
Messages and Services are sourced from the spawn_object_interfaces package.
The object_spawner_manager offers the following services:
```
SpawnObject
```
* `obj_name`: tbd
* `parent_frame`: tbd
* `translation`: tbd
* `rotation`: tbd
* `cad_data`: tbd
------------------------
* `success`: tbd

```
DestroyObject
```
* `obj_name`: tbd
------------------------
* `success`: tbd

```
CreateRefFrame
```
* `frame_name`: tbd
* `parent_frame`: tbd
* `pose`: tbd
------------------------
* `success`: tbd
```
DeleteRefFrame
```
* `frame_name`: tbd
------------------------
* `success`: tbd
```
ChangeParentFrame
```
* `obj_name`: tbd
* `parent_frame`: tbd
------------------------
* `success`: tbd
```
SpawnFromDict
```
* `dict`: tbd
------------------------
* `success`: tbd
```
ModifyPose
```
* `frame_name`: tbd
* `rel_pose`: tbd
------------------------
* `success`: tbd
```
GetInfo
```
* None
------------------------
* `obj_names (string[])`: tbd
* `ref_frame_names (string[])`: tbd