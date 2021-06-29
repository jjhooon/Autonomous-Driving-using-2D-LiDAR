## 2021-1학기 캡스톤 프로젝트

RPLiDAR a1 모델을 이용하여 rc카 자율주행을 실시하였습니다.

이번 캡스톤 프로젝트에서의 목표는 2D LiDAR 센서를 이용하여 직선 주행, 코너 주행을 성공하는 것입니다.

![image](https://user-images.githubusercontent.com/81551992/123759467-e4624080-d8fa-11eb-8a40-2f7eacec64a1.png)

그림과 같이 2D LiDAR 센서와 Jetson Nano, Arduino, ESC, Battery, Motor로 하드웨어를 구축하였습니다.

![image](https://user-images.githubusercontent.com/81551992/123759778-2f7c5380-d8fb-11eb-9ccc-bf8b3f8d4f83.png)

Ubuntu 환경에서 ROS를 이용하여 소프트웨어 환경을 구축하였고, 위의 그림과 같이 RPLiDAR a1 모델로부터 얻은 scan data를 이용하여 직접 개발한 warm_capstone package에서 후처리 및 직선/코너 주행 알고리즘을 구현하였습니다. 최종적으로 상황에 맞는 속도와 조향각을 arduino로 넘겨 arduino에서 조향 및 속도를 제어하였습니다.

LiDAR scan data 후처리 과정으로는 PCL변환, Voxelization, Pass through, Clustering을 진행하였습니다. 이후, ransac 알고리즘을 이용하여 양쪽 벽의 직선의 방정식을 구하고, 양쪽 벽 기울기의 평균값을 이용하여 곡선 및 직선 주행을 판단하였습니다.
