# DipG-Seg
***DipG-Seg*** is a fast and accurate ground segmentation algorithm based on double images including $z$-image and $d$-image. This method is pixel-wise, which could be regarded as the counterpart of point-wise on the 3D point cloud. Despite this, our method is very efficient, meanwhile, accurate.
## 1. Features of DipG-Seg
* A **complete framework of ground segmentation totally based on images**. Thus, it is very easy to further accelerate by adjusting the resolutions of images.
* **Accurate and Super Fast**. DipG-Seg can run at more than 120Hz on an Intel NUC (i7 1165G7) with a resolution of $64\times 870$, achieving a high accuracy of over 94% on the SemanticKITTI dataset.
* **Robust** to LIDAR models and scnenarios. The given parameters allow DipG-Seg to work well on 64, 32, and 16-beam LiDARs and in scenarios in nuScenes and SemanticKITTI.

<!-- <p align="center"><img src=./pictures/seq00-gif.gif alt="animated" /></p> -->
<table><tr>
<td><img src= pictures/seq04-gif.gif border=0></td>
<td><img src= pictures/seq08-gif.gif border=0></td>
</tr></table>

## 2. About this repo
### 2.1 Hope this repo can help you

If you find our repo helpful for your research, please cite our paper. Thank you!

Author: [Hao Wen](https://scholar.google.com/citations?user=823HzfIAAAAJ&hl=zh-CN) and [Chunhua Liu](https://scholar.google.com/citations?user=7WEZSaIAAAAJ&hl=zh-CN) from EEPT Lab at CityU.

Paper: [DipG-Seg: Fast and Accurate Double Image-Based Pixel-Wise Ground Segmentation](), Hao Wen, Senyi Liu, Yuxin Liu, and Chunhua Liu, T-ITS, Regular Paper
```
@article
```
Explore more demos in [Video](video_link)

### 2.2 What in this repo
* An example of a ros node for validation on your own platform.
* Visualization demo and evaluation program based on the KITTI dataset.

## 3. How to Run
### 3.1 Environment
Ubuntu18.04 + ROS Melodic
> ROS can be installed by following this [tutorial](https://wiki.ros.org/melodic/Installation/Ubuntu).

### 3.2 Prerequisite Packages
#### C++ packages
1. OpenCV
2. PCL 

> **OpenCV** can be installed from this [link](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html). <br>
> **PCL** can be installed by runing this in the terminal:
```bash
sudo apt install libpcl-dev
```

#### Python packages
```bash
#install python lib -- numpy, pandas -- for python script of evaluation.
pip2 install numpy pandas
```
### 3.3 Build
```bash
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone https://github.com/EEPT-LAB/DipG-Seg.git
cd .. && catkin_make
# remember to source devel/setup.bash before you run the nodes in the dipgseg
```
Also, you can build with catkin tools:
```bash
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws
catkin init
cd ~/catkin_ws/src/
git clone https://github.com/EEPT-LAB/DipG-Seg.git
cd .. && catkin build dipgseg
# remember to source devel/setup.bash before you run the nodes in the dipgseg
```

### 3.4. Dataset
> 1. If you want to validate DipG-Seg on [SemanticKITTI](http://www.semantic-kitti.org/). Please download it and place it on your `~/your_dataset_path/`.
> 2. If you want to validate DipG-Seg on [nuScenes](https://www.nuscenes.org/). Please download it and place it on your `~/your_dataset_path/`. **NOTE THAT** we do not provide the complete program for the evaluation on nuScenes, BUT we provide the projection parameter for nuScenes LIDAR. Remember to modify the [projection_parameter](./src/include/projection_param.h) when you validate on nuScenes.
> 3. You can also validate on your own mobile platform, and please ensure your LiDAR can publish `sensor_msgs::PointCloud2` to the topic `/pointcloud `.  Otherwise, you can remap the topic `/pointcloud` to `your_point_cloud_topic_name` in the [launch file](./launch/demo.launch). Last but not least, you should modify the [projection_parameter](./src/include/projection_param.h) for your LiDAR. Some tool scripts are provided in the [scripts](./scripts/) folder. 

### 3.5 Let's run
#### 3.5.1 Run on the SemanticKITTI dataset.
+ Modify the parameters dataset_path and the seq_num in the [launch file](./launch/visualization_offline_kitti.launch) as:
```xml
<rosparam param="dataset_path">"/path_to_your_downloaded_dataset/sequences/"</rosparam>
<node pkg="dipgseg" type="offline_kitti_node" name="dipgseg_node" output="screen" args="seq_num">
```

+ Then,  run the following command in the terminal:
```bash
roslaunch dipgseg visualization_offline_kitti.launch 
```

#### 3.5.2 Evaluation on SemanticKITTI dataset.
+ Modify the parameters dataset_path in the [launch file](./launch/eval_offline_kitti.launch) as:
```xml
<rosparam param="dataset_path">"/path_to_your_downloaded_dataset/sequences/"</rosparam>
```
+ Then,  run this python [script](./scripts/eval_on_kitti.py) in the terminal:
```bash
cd ~/catkin_ws/src/DipG-Seg/scripts/
python2 eval_on_kitti.py
```
When the evaluation is finished, you can find the evaluation results in the [result](./result/) folder.

#### 3.5.3. Run on your own mobile robots or record ros bag file.
+ According to the parameters of your LiDAR, modify the [projection_param](./src/include/projection_param.h) file. A simple tool script for generating the projection parameters of LIDARs that have even vertical angle resolution will be released. Now, only the parameters for LIDARs of SemanticKITTI and nuScenes are provided. 

> NOTE: If you want to generate the projection parameters of LIDARs except for the above two datasets, instructions are explained in detail in our [paper](paper_link).

+ Remap the topic `/pointcloud` to `your_point_cloud_topic_name` in the [launch file](./launch/demo.launch) if necessary.
```xml
<remap from="/pointcloud" to="your_point_cloud_topic_name" />
```

+ Start your LiDAR sensor node at first, then run the following command in the terminal:
```bash
roslaunch dipgseg demo.launch
```
### 3.6 Tool scripts
Finished

-[x] [eval_on_kitti.py](./scripts/eval_on_kitti.py): Evaluation on the SemanticKITTI dataset.

-[x] [kitti_ros_publisher.py](./scripts/kitti_ros_publisher.py): Publish the SemanticKITTI dataset as ros bag file.

To be finished

-[ ] [projection_param_generator.py](./scripts/): Generate the projection parameters of LIDARs that have even vertical angle resolution.

## 4. Contact
Maintainer: Hao Wen

Email: hao.wen@my.cityu.edu.hk

## Acknowledgement
To achieve a nice code style and good performance, the following works give us a lot of help and inspiration. Thanks to the authors for their great works! 
1. [depth_clustering](https://github.com/PRBonn/depth_clustering): File loader and image-based segmentation.

2. [Patchwork](https://github.com/LimHyungTae/patchwork) and [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) and [Ground-Segmentation-Benchmark](https://github.com/url-kaist/Ground-Segmentation-Benchmark): Ground segmentation evaluation.
