# rma-projeto-1
Projeto de Implementação 1 da Disciplina de Robôs Móveis Autônomos

Alunos:
* Antonio Carlos Daud Filho
* Leandro Silva de Assis
* Leonardo Simião de Luna

Sistema operacional:	Ubuntu 20.04
ROS	Noetic
Gazebo	Gazebo multi-robot simulator - version 11.13.0

# Como utilizar
Após a instalação (descrita na seção abaixo), deve-se utilizar o o seguinte launch para inicializar o modelo do robô no gazebo:
```bash
roslaunch mobile_rob_dev_sim dcrobot_gazebo.launch
```
Com o modelo no gazebo, é possível visualizar o mapa com o comando:
```bash
rosrun rviz rviz -d rma.rviz
```
Caso a visualização não funcione, na janela do rviz deve-se ir em File > Open Config e selecionar o arquivo rma.rviz deste repositório.

Para realizar a movimentação do robô, deve-se utilizar os comandos:
```bash
cd ~/workspace/src/dcrobot/mobile_rob_dev_sim/scripts/
python3 run_trajectory.py
```

# Instalando as dependências
$ sudo apt-get udo apt-get install ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins python3-wstool python3-catkin-tools ros-noetic-ompl ros-noetic-move-base ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-costmap-2d ros-noetic-teb-local-planner ros-noetic-robot-self-filter ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy
$ pip install numpy matplotlib scipy
$sudo apt-get install ros-noetic-openslam-gmapping
$ cd ~/workspace/src/
$ git clone https://github.com/ethz-asl/eigen_catkin.git
$ git clone https://github.com/catkin/catkin_simple.git
$ git clone https://github.com/anybotics/grid_map.git
$ git clone https://github.com/ros-perception/slam_gmapping.git
$ git clone https://github.com/ros-planning/navigation.git

### Navegar até o repositório SRC e Clonar o repositório
$ cd/home/$USER/dcrobot_ws/src
$ git clone https://github.com/leonardosimiao/rma-projeto-1

### Montando o projeto
$cd /home/$USER/dcrobot_ws/
catkin build

### Obter Workspace do catkin
$ source /home/$USER/dcrobot_ws/devel/setup.bash

### inicializar o ROS GAZEBO
$ roslaunch gazebo.launch

### Time in ROS
o algoritmo calcula o tempo de percorer a trajetória 

# Move robot
Publica os comandos para movimentar o robor a partir da função twist
Transforma posição quaternaria para vetor e angulo, retornando posição X, Y, Z
Recebe a coordenada global do ponto e define a primeiro como sendo X e a segunda como sendo Y
Define os parametros para navegação: ponto de partida, ponto de chegada, numero de interações angular, incremento angular, incremento linear, velocidade angular, velocidade linear, posição de partida linear do odometro, posição de partida angular, variação agular e linear total, define um ponto para reiniciar o planejamento
Recebe distancia e angulo local através da função callback e compara com angulo e distância global, para definir o planejamento da proxima posição. 
