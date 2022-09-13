# deremove
detect & remove
本项目基于ROS，目的在于基于视觉语义滤除激光点云中的动态点。
包含三个ROS节点，Seg node执行语义分割，Remove node接收原始点云和语义，完成点云到图像的投影，剔除动态点云。得到的静态点云发给Aloam节点建图。
TODO：语义分割效果不好，考虑更好的分割模型，并结合激光点云的几何聚类提高分割精度。
使用方法：放入ros工作空间编译。在KITTI数据集上测试。
![deremove](https://user-images.githubusercontent.com/42105276/189855013-44142777-d37d-41f8-b9e1-be1d35981d7a.png)

![图片3](https://user-images.githubusercontent.com/42105276/189471492-9e1b5675-ea8e-4e4b-9252-b13b59cfb2c2.png)
