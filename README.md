# 基於gazebo的全向輪機器人的模擬

~~從入門到放棄~~

## 環境

### win10

- Solidworks2019_SP0
- solidworks to URDF Exporter 1.5.1 [github](https://github.com/ros/solidworks_urdf_exporter/releases)

### ubuntu20.04

- ros-noetic
- gazebo-11.0.0

## 步驟

### solidworks

1. 繪製全向輪，[參考資料](md_data/solidworks繪製全向輪.pdf)、[網站](https://kknews.cc/zh-tw/news/mngj8r6.html)
2. 繪製車體，配合全向輪進行繪製，三或四輪看個人
3. 將車體與全向輪進行結合，並為需要進行運動的**所有**關節建立座標系統與基準軸
4. 使用 Solidworks > 工具 > File > Export as URDF
5. 建立機器人URDF檔案，[參考資料](md_data/SolidWorks模型輸出成URDF.pdf)、[網站](https://weishih.wordpress.com/2017/01/06/%E5%B0%87solidworks%E6%A8%A1%E5%9E%8B%E8%BC%B8%E5%87%BA%E6%88%90urdf/?fbclid=IwAR3gcZBXRp4yhfm-1zP98zb8EmopKIXv-wqflF5GA7eK4-uB9-Gvpz0pcFA)
6. Solidworks 完成

- URDF轉檔資料夾中使用到的：
  - urdf
  - meshes
  - package.xml
  - CMakeLists.txt

### ubuntu

1. 建立一個工作環境

    ```bash
    mkdir -p catkin_ws/src && cd catkin_ws/src
    ```

2. 複製URDF資料夾至src，**此教學範例名稱為robot**

3. 進行catkin_make，測試URDF是否正常

    ```bash
    cd ~/catkin_ws && catkin_make
    source devel/setup.bush
    roslaunch robot gazebo.launch
    ```

4. 修改URDF副檔名，並修改內容使其相符

    ```bash
    cd ~/catkin_ws/src/robot/urdf
    cp robot.urdf robot.xacro
    nano robot.xacro
    ```

    使用ctrl+w收尋``robot``，約為第五、六行
    將 `` <robot name="robot"> `` 更改為 `` <robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro"> ``

5. 新增機器人敘述文件

    於robot下一行新增``<xacro:include filename="$(find robot)/urdf/description.gazebo" />``，**提醒robot為此教學範例名稱**

    ```bash
    touch description.gazebo && nano description.gazebo
    ```

    description.gazebo

    ```xml
    <?xml version="1.0"?>
    <robot>
      <gazebo reference="link_name">
        <material>color</material>
        <!--  Friction coefficients  -->
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <!-- If true, the link can collide with other links in the model. -->
        <selfCollide>false</selfCollide>
      </gazebo>
    </robot>
    ```

    依照上方格式新增各個link與修改``link_name``與``color``，[敘述參考資料](md_data/gazebo_description.pdf)、[網站](http://gazebosim.org/tutorials?tut=ros_urdf)；[顏色參考資料](md_data/gazebo_color.pdf)、[網站](https://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials)

6. 新增.yaml配置文件

    ```bash
    cd ~/catkin_ws/src/robot && mkdir config && cd config && touch robot.yaml && nano robot.yaml
    ```

    robot.yaml

    ```yaml
    robot:
      # Publish all joint states -----------------------------------
      joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 50

      # joint Velocity Controllers ---------------------------------------
      left_joint_velocity_controller:
        type: effort_controllers/JointVelocityController
        joint: joint_name
        pid: {p: 0.001, i: 0.0, d: 0.0}

      back_joint_velocity_controller:
        type: effort_controllers/JointVelocityController
        joint: joint_name
        pid: {p: 0.001, i: 0.0, d: 0.0}

      right_joint_velocity_controller:
        type: effort_controllers/JointVelocityController
        joint: joint_name
        pid: {p: 0.001, i: 0.0, d: 0.0}
    ```

    將``joint_name``修改輪轂的joint名稱，控制器類型參考 [參考資料](md_data/ros_control.pdf)、[網站](http://wiki.ros.org/ros_control) 2.Controllers類型

7. 新增URDF傳動系統

    **注意gazebo_ros_control只能呼叫一次**

    - 方法一：新增.gazebo，再於xacro中include(推薦)

    ```bash
    cd ~/catkin_ws/src/robot/urdf && touch control.gazebo && nano control.gazebo
    ```

    control.gazebo

    ```xml
    <?xml version="1.0"?>
    <robot>
      <!-- parses the transmission tags and loads the appropriate hardware interfaces and controller manager -->
      <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
          <robotNamespace>/robot</robotNamespace>
          <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
        </plugin>
      </gazebo>

      <!-- all joint transmission -->
      <transmission name="left_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint_name">
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="left_motor">
          <mechanicalReduction>1</mechanicalReduction>
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </actuator>
      </transmission>

      <transmission name="right_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint_name">
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="right_motor">
          <mechanicalReduction>1</mechanicalReduction>
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </actuator>
      </transmission>

      <transmission name="back_transmission">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint_name">
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="back_motor">
          <mechanicalReduction>1</mechanicalReduction>
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </actuator>
      </transmission>

    </robot>

    ```

    將``joint_name``修改輪轂的joint名稱，transmission與actuator名稱可自定義

    ```bash
    nano robot.xacro
    ```

    使用ctrl+w收尋``robot``，約為第五、六行於robot下一行新增``<xacro:include filename="$(find robot)/urdf/control.gazebo" />``，[gazebo_ros_control參考資料](md_data/gazebo_ros_control.pdf)；[網站](http://gazebosim.org/tutorials/?tut=ros_control)[傳動系統參考資料](md_data/URDF_Transmissions.pdf)、[傳動系統網站](https://wiki.ros.org/urdf/XML/Transmission)

    - 方法二：

    ```bash
    cd ~/catkin_ws/src/robot/urdf && nano robot.xacro
    ```

    使用ctrl+w收尋``</robot>``，於/robot上一行新增

    ```xml
    <!-- parses the transmission tags and loads the appropriate hardware interfaces and controller manager -->
    <gazebo>
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/robot</robotNamespace>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
      </plugin>
    </gazebo>

    <!-- all joint transmission -->
    <transmission name="left_transmission">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="joint_name">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="left_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </actuator>
    </transmission>

    <transmission name="right_transmission">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="joint_name">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="right_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </actuator>
    </transmission>

    <transmission name="back_transmission">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="joint_name">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="back_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </actuator>
    </transmission>
    ```

    將``joint_name``修改輪轂的joint名稱，transmission與actuator名稱可自定義，[gazebo_ros_control參考資料](md_data/gazebo_ros_control.pdf)；[網站](http://gazebosim.org/tutorials/?tut=ros_control)[傳動系統參考資料](md_data/URDF_Transmissions.pdf)、[傳動系統網站](https://wiki.ros.org/urdf/XML/Transmission)

8. 新增launch文件

    ```bash
    cd ~/catkin_ws/src/robot && mkdir launch && cd launch && touch gazebo.launch && nano gazebo.launch
    ```

    gazebo.launch

    ```xml
    <launch>
      <!-- world param -->
      <arg name="paused" default="true"/>
      <arg name="use_sim_time" default="true"/>
      <arg name="gui" default="true"/>
      <arg name="headless" default="false"/>
      <arg name="debug" default="false"/>

      <!-- Robot pose (Optional) -->
      <arg name="x" default="0"/>
      <arg name="y" default="0"/>
      <arg name="z" default="0"/>
      <arg name="roll" default="0"/>
      <arg name="pitch" default="0"/>
      <arg name="yaw" default="0"/>

      <!-- Load joint controller configurations from YAML file to parameter server -->
      <rosparam file="$(find model)/config/robot.yaml" command="load"/>
      <!-- create empty_world -->
      <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="headless" value="$(arg headless)"/>
        <arg name="debug" value="$(arg debug)"/>
      </include>

      <!-- Load the URDF into the ROS Parameter Server -->
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find model)/urdf/robot.xacro'" />

      <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
      <node
        name="urdf_spawner"
        pkg="gazebo_ros"
        type="spawn_model"
        respawn="false"
        output="screen"
        args="-urdf -model robot -param robot_description -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)" />

      <!-- load the controllers -->
      <node
        name="controller_spawner"
        pkg="controller_manager"
        type="spawner"
        respawn="false"
        output="screen"
        ns="/robot"
        args="joint_state_controller
              left_joint_velocity_controller
              back_joint_velocity_controller
              right_joint_velocity_controller"/>

      <!-- convert joint states to TF transforms for rviz, etc -->
      <node
        name="robot_state_publisher"
        pkg="robot_state_publisher"
        type="robot_state_publisher"
        respawn="false"
        output="screen">
        <remap from="/robot" to="/robot/test"/>
      </node>

    </launch>
    ```

    以ctrl+w收尋``文字``
    - 修改項目：
      - ``rosparam file``，command的yaml位置、名稱
      - ``name="robot_description"``，command的xarco位置、名稱
      - ``name="controller_spawner"``，ns可自訂，args對應robot.yaml
      - ``remap``，from對應controller_spawner的ns，to可自訂

---

## Error

Controller Spawner couldn't find the expected controller_manager ROS interface.

```bash
sudo apt-get install ros-<distro>-gazebo-ros-control
```

Could not stop controller 'joint_state_controller' since it is not running

```bash
sudo apt-get install ros-<distro>-joint-state-controller
sudo apt-get install ros-<distro>-position-controllers
sudo apt-get install ros-<distro>-effort-controllers
```
