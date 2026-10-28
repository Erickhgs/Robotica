# Práticas de Robótica — ROS 2 Humble

Os comandos abaixo partem de `~/ros2_ws/Robotica`, que contém esta pasta `src`.
Execute somente uma simulação por vez e encerre com **Ctrl+C** antes de iniciar
outro ensaio. Não rode teleop junto com um controlador de trajetória.

## Preparação

Para instalar as dependências declaradas nos pacotes, use `./run.sh deps` na raiz
do projeto. As dependências acrescentadas/usadas pela prática 5 podem ser
instaladas diretamente com:

```bash
sudo apt-get update
sudo apt-get install ros-humble-robot-localization ros-humble-plotjuggler-ros
```

Compile depois das alterações:

```bash
cd ~/ros2_ws/Robotica
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Em **cada novo terminal**, carregue o ambiente:

```bash
cd ~/ros2_ws/Robotica
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Prática 5 — exercício 1.1: estimar a pose com Kalman

```bash
ros2 launch robot_control pratica05.launch.py task:=sensors noise:=low
```

Esse comando abre Gazebo, sensores, controladores, RViz e o `ekf_node` do pacote
`robot_localization`. O robô começa parado. Em outro terminal:

```bash
ros2 topic echo /odometry/filtered --once
ros2 topic hz /odometry/filtered
```

A saída é `nav_msgs/msg/Odometry`: `pose.pose.position.x/y` contém a posição em
metros e `pose.pose.orientation` contém a orientação em quaternion. A frequência
configurada é 50 Hz de tempo simulado; a taxa observada depende da velocidade da
simulação. Para testar em movimento, use em outro terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
```

Mantenha o foco no terminal do teleop. Use `i` para frente, `,` para trás,
`j`/`l` para girar e `k` para parar. Repita a tecla para manter o movimento se
o timeout dos comandos parar o robô. Para perceber o ruído, avance, pare com
`k`, gire e pare novamente.

Abra dois outros terminais com o ambiente carregado e observe X/Y:

```bash
# Terminal A: GPS + IMU sem Kalman.
ros2 topic echo /localization/odom --field pose.pose.position --flow-style
```

```bash
# Terminal B: estimativa do EKF.
ros2 topic echo /odometry/filtered --field pose.pose.position --flow-style
```

Acrescente `--once` para imprimir somente uma amostra. Para incluir a orientação,
troque o campo por `--field pose.pose`. Ela é um quaternion: `orientation.z` não
é o ângulo yaw. No CSV comparativo, `yaw` já está convertido para radianos.
Compare X/Y; a altura Z é zerada pelo modo planar do EKF.

`/odom` é a odometria das rodas e `/ground_truth/odom` é o estado real do Gazebo.
Para avaliar a filtragem dos sensores use `/localization/odom` versus
`/odometry/filtered`, que compartilham os mesmos eixos neste mundo. A odometria
das rodas pode ter outra orientação inicial, especialmente no ensaio em 8.
Dois `echo` não sincronizam amostras; use os gráficos ou o CSV para uma comparação
por tempo. `task:=sensors` não grava os CSVs do experimento em 8.

O fluxo de dados é:

```text
/gps/fix -> sensor_pose -> /localization/odom (pose bruta, GPS + yaw da IMU)
/imu/raw -> sensor_pose -> /imu/data (IMU com ruído de orientação)
                            |
          X/Y bruto + yaw/velocidade angular IMU -> ekf_node
                                                  |
                                           /odometry/filtered
```

O arquivo `robot_state_estimation/config/ekf.yaml` habilita o modo planar
`two_d_mode` e seleciona X/Y do GPS e yaw/velocidade angular Z da IMU. O yaw da
odometria bruta não entra novamente no filtro: ele já vem dessa mesma IMU.
O ground truth e a odometria das rodas não entram no EKF.

Neste mundo local, `map` e `odom` têm a mesma origem ENU fixa e são ligados por
uma transformação identidade. O EKF publica a pose em `odom` e a TF
`odom -> chassis`; a TF da pose bruta fica desativada. O controlador das rodas
também mantém sua TF desativada. Assim há uma única TF para a pose do robô.
Essa escolha é para o mundo plano fornecido; um sistema de navegação com
correções globais descontínuas exige rever a árvore map/odom.

## Prática 5 — exercício 2.1: trajetória em 8 com estado estimado

Primeiro encerre o experimento anterior e o teleop. Rode com ruído baixo:

```bash
ros2 launch robot_control pratica05.launch.py task:=eight noise:=low seed:=42
```

Espere a mensagem **Trajetória concluída** e encerre com Ctrl+C. Depois rode com
ruído alto, mantendo a mesma referência e semente:

```bash
ros2 launch robot_control pratica05.launch.py task:=eight noise:=high seed:=42
```

O controlador existente usa `/odometry/filtered` como realimentação, com a mesma
lei de controle da prática 4. A referência é
`x = sin(omega*t)`, `y = 2*sin(2*omega*t)`, com `omega=0.1 rad/s` por padrão.
Uma volta dura aproximadamente **62,83 segundos de simulação**, além da
inicialização. O robô nasce orientado na tangente inicial da curva.

| Grandeza | `noise:=low` | `noise:=high` |
|---|---:|---:|
| Desvio padrão GPS X/Y | 0,02 m | 0,16 m |
| Desvio padrão yaw IMU | 0,01 rad | 0,08 rad |
| Desvio padrão giroscópio | 0,002 rad/s | 0,016 rad/s |

O ruído alto multiplica os desvios dos sensores por 8 e as variâncias por 64.
As covariâncias fornecidas ao filtro acompanham o ruído. `noise:=zero` existe
para diagnóstico; use `low` e `high` na comparação solicitada.

Opções úteis:

```bash
# Sem interfaces gráficas; os dados continuam sendo registrados.
ros2 launch robot_control pratica05.launch.py noise:=high gui:=false rviz:=false

# Duas voltas ou outra velocidade de referência.
ros2 launch robot_control pratica05.launch.py cycles:=2 omega:=0.1
```

## Resultados e PlotJuggler

Cada ensaio em 8 cria uma subpasta própria em `~/pratica05_resultados` contendo:

- `run_metadata.json`: ruído, semente, frequência e tópico usado no controle;
- `eight_*.csv`: estado estimado usado pelo controle, referência e comandos;
- `ground_truth_*.csv`: estado real do Gazebo, independente do estimador;
- `raw_pose.csv`: pose bruta GPS + IMU, sem Kalman.

Após encerrar os ensaios, gere as comparações:

```bash
ros2 run robot_control analyze_sensors ~/pratica05_resultados \
  --output ~/pratica05_resultados/resumo.csv
ros2 run plotjuggler plotjuggler
```

O analisador cria `comparison.csv` dentro de **cada ensaio**. No PlotJuggler:

1. Use **Load Data** e abra o `comparison.csv` do ensaio de ruído baixo.
2. Selecione **ros_time** como coluna de tempo.
3. Crie três gráficos e arraste as quatro séries indicadas abaixo para cada um.
4. Repita para o ensaio de ruído alto e compare na mesma escala.

| Gráfico | Real (Gazebo) | Bruto (sensores) | Estimado (Kalman) | Desejado |
|---|---|---|---|---|
| X(t), metros | `x_true` | `x_raw` | `x` | `x_ref` |
| Y(t), metros | `y_true` | `y_raw` | `y` | `y_ref` |
| θ(t), radianos | `yaw_true` | `yaw_raw` | `yaw` | `yaw_ref` |

Para visualizar o desenho do 8, use um gráfico **XY** com os pares
`(x_true,y_true)`, `(x_raw,y_raw)`, `(x,y)` e `(x_ref,y_ref)`.
Crie uma área de gráfico vazia, selecione as duas séries de um par segurando
**Ctrl** e arraste-as para a área com o **botão direito** do mouse. Na janela
de criação, confirme qual série será X e qual será Y. Repita os quatro pares
na mesma área. Uma área que já contém gráficos de tempo precisa ter suas curvas
removidas antes de ser convertida em XY.
Salve o layout do PlotJuggler para reutilizar e capture os gráficos para o relatório.
Os ângulos estão em `[-pi, pi]`, então atravessar esse limite produz um salto
visual; os erros angulares são calculados com normalização.

Também é possível acompanhar ao vivo pelo plugin **ROS 2 Topic Subscriber**:
`/ground_truth/odom`, `/localization/odom`, `/odometry/filtered` e
`/tracking/reference` fornecem as quatro posições. No CSV, o yaw já está
convertido para radianos, dispensando a conversão dos quaternions na interface.

Com a simulação aberta, inicie o PlotJuggler em um terminal com o ambiente ROS
carregado. Na seção **Streaming**, selecione **ROS2 Topic Subscriber** e clique
em **Start**. Selecione os quatro tópicos acima (no modo `task:=sensors`, a
referência desejada não existe). Marque **If present, use the timestamp in the
field [header.stamp]** para usar o tempo das mensagens do Gazebo. Confirme a
seleção e arraste os campos de posição para as áreas dos gráficos:

| Estado | Campo X | Campo Y |
|---|---|---|
| Real | `/ground_truth/odom/pose/pose/position/x` | `/ground_truth/odom/pose/pose/position/y` |
| Bruto | `/localization/odom/pose/pose/position/x` | `/localization/odom/pose/pose/position/y` |
| Estimado | `/odometry/filtered/pose/pose/position/x` | `/odometry/filtered/pose/pose/position/y` |
| Desejado | `/tracking/reference/pose/position/x` | `/tracking/reference/pose/position/y` |

A referência é `PoseStamped`, por isso seu caminho tem somente um `pose`.
Use um gráfico para X e outro para Y, com quatro curvas em cada. A captura ao
vivo começa quando você se inscreve nos tópicos; para incluir o início do ensaio
e obter também o yaw convertido, abra o `comparison.csv` após a execução.

O analisador interpola o ground truth no tempo de cada amostra do controle. A
curva bruta usa a última leitura disponível com idade máxima de 0,1 s, sem
interpolar/suavizar o ruído. Se houver poses brutas com o mesmo timestamp, usa
a última registrada naquele instante, sem fazer média. Recuos do relógio
continuam sendo rejeitados. O erro bruto é calculado contra o ground truth no
timestamp original dessa leitura. São descartadas amostras sem correspondência
recente. Confira `coverage_percent` e `complete` no resumo.

Compare `rmse_raw_m` com `rmse_estimation_m` para avaliar a posição bruta e a
estimada; `rmse_raw_yaw_rad` e `rmse_filtered_yaw_rad` avaliam a orientação.
`rmse_tracking_true_m` mede o seguimento **real** da referência pelo robô.
O RMSE publicado em `/tracking/rmse_position` mede referência versus estimativa
usada pelo controle; ele sozinho não mede o erro real do robô. O filtro pode
reduzir ruído e introduzir atraso; verifique o resultado medido em cada ensaio.

## Como rodar as práticas anteriores disponíveis no projeto

Os roteiros anteriores não estão nesta pasta; estes comandos correspondem às
funcionalidades já implementadas nos pacotes.

### Modelo SDF e simulação URDF

```bash
ros2 launch robot_description gazebo_sdf.launch.py
```

Para o robô URDF com os controladores das rodas:

```bash
ros2 launch robot_description bringup.launch.py
```

Em outro terminal, use `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
para teleoperar a simulação URDF. Para trocar de simulação, encerre a anterior.

### Prática 3: pose, missão e trajetória em 8

Execute um comando por vez:

```bash
# Controle até uma pose; aguarda objetivo em /goal_pose.
ros2 launch robot_control pratica03.launch.py gazebo:=true task:=pose method:=continuous

# Alternativa de controle por três manobras.
ros2 launch robot_control pratica03.launch.py gazebo:=true task:=pose method:=three_maneuvers

# Missão com os waypoints de robot_control/config/mission.yaml.
ros2 launch robot_control pratica03.launch.py gazebo:=true task:=mission

# Trajetória em 8 com realimentação ou em malha aberta.
ros2 launch robot_control pratica03.launch.py gazebo:=true task:=eight mode:=feedback kff:=1.0
ros2 launch robot_control pratica03.launch.py gazebo:=true task:=eight mode:=open_loop
```

No modo `task:=pose`, envie em outro terminal um objetivo (1 m, 1 m, yaw zero):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}'
```

Para testar o controle sem Gazebo, substitua `gazebo:=true` por `demo:=true`.
Os CSVs ficam em `~/pratica03_resultados` e podem ser avaliados com:

```bash
ros2 run robot_control analyze_results ~/pratica03_resultados/*.csv
```

### Prática 4: sensores e controle com pose bruta

```bash
# Inspeção de GPS, IMU, câmera e lidar; permite teleop em outro terminal.
ros2 launch robot_control pratica04.launch.py task:=sensors noise:=low

# Execute separadamente, encerrando o anterior: controle direto com GPS+IMU.
ros2 launch robot_control pratica04.launch.py task:=eight noise:=low seed:=42
ros2 launch robot_control pratica04.launch.py task:=eight noise:=high seed:=42

# Depois de encerrar as simulações:
ros2 run robot_control analyze_sensors ~/pratica04_resultados
```

Na prática 4 a entrada continua sendo `/localization/odom`, sem Kalman. Na prática
5 a entrada passa para `/odometry/filtered`. Os launches compartilham o mesmo
experimento para manter modelo, sensores e referência iguais.

## Verificação

```bash
colcon test --packages-select robot_control robot_state_estimation --python-testing pytest
colcon test-result --verbose
```

Se aparecer `Package 'robot_localization' not found`, instale a dependência da
preparação. Se os pacotes locais não forem encontrados, refaça o build e o
`source install/setup.bash`. A mensagem de espera por odometria na inicialização
é normal; se persistir, confira `/clock`, `/gps/fix`, `/imu/data`,
`/localization/odom`, `/odometry/filtered` e os avisos em `/diagnostics`.

Se `/localization/odom` aparece mas `/odometry/filtered` não, confira se o
`ekf_node` encerrou com erro no terminal do launch. Um erro
`libdiagnostic_updater.so: cannot open shared object file`, com saída 127,
indica que o `robot_localization` instalado requer uma versão mais recente de
`diagnostic_updater`. Neste ambiente, a versão 4.0.6 não fornece essa biblioteca;
a versão 4.0.7 fornece. Atualize a dependência pelo gerenciador de pacotes:

```bash
sudo apt-get update
sudo apt-get install --only-upgrade ros-humble-diagnostic-updater
```

Encerre o launch anterior com Ctrl+C e inicie novamente:

```bash
ros2 launch robot_control pratica05.launch.py task:=sensors noise:=low
```

Em outro terminal com o ambiente carregado, confirme o publicador e uma amostra:

```bash
ros2 topic info /odometry/filtered
ros2 topic echo /odometry/filtered --once
```

Deve haver um publicador do EKF e uma mensagem `nav_msgs/msg/Odometry`. O launch
agora encerra se o processo do EKF falhar, em vez de continuar sem estimativa.

Referências: [configuração oficial do EKF para Humble](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)
e [PlotJuggler](https://github.com/PlotJuggler/PlotJuggler).
