### Dependencies

- Docker
- Nvidia Container toolkit

### Installing
Because we will use CoppeliaSim as our simulation and some other third app for this project, we need to download the zip file from [here](https://drive.google.com/drive/folders/1QUWJlT4B2yIQaNmF-G1xE8XvtskaKTv8?usp=sharing). Then, place them in (__do not unzip them__)
```
[directory where you have the scene_modeling]/scene_modeling/docker_image/download
```
Build the docker image, and this step will take a long time.
```
cd [directory where you have the scene_modeling]/scene_modeling/docker_image
sh build.sh
```

Once the image is built, you can run the image as a container by (__xhost is required once you use docker run here__)
```
xhost +
sh run.sh
```

At this point, you will enter the container, but the workspace is not compiled yet. In the docker container, you need to run following code for preparing the workspace.

<a id="workspace_prepare"></a>
```
cd $HOME
./prepare_workspace.sh
source .bashrc
```

If you want to run multiple terminals in the container after running above commands, you can run the following commands in a new terminal(__not in the container__)
```
cd [directory where you have the scene_modeling]/scene_modeling/docker_image && sh enter_lastest_container.sh
```
For runing this command properly, you can have only one active container.