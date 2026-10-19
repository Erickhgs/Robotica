# Robótica

Workspace ROS 2 Humble (Ubuntu 22.04) das práticas.

## Pacotes

- `robot_description`: modelo do robô (URDF/SDF), Gazebo e ros2_control
- `robot_control`: controle de pose, missão e trajetória em 8 (prática 03)
- `robot_navigation`: navegação
- `robot_state_estimation`: localização

## Rodando

```bash
./run.sh deps                 # dependências, só na primeira vez
./run.sh sim                  # Gazebo com o robô e os controladores
./run.sh sdf                  # modelo SDF da prática 01
./run.sh teleop               # teclado, em outro terminal com o sim aberto
./run.sh pratica03 task:=eight mode:=feedback kff:=1.0
./run.sh test
```

Os CSVs da prática 03 ficam em `resultados/`.
