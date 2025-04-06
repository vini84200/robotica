# phi_aria

## Como compilar: ##
```colcon build```

## Como executar: ##

*OBS: antes de rodar o programa é preciso abrir o simulador MobileSIM (ou ligar o robô real, se for o caso).*

Para rodar em simulação:

```ros2 run phi_aria phi_p3dx --ros-args -p publish_aria_lasers:=true -p port:=localhost```

Para rodar com o robô real:

```ros2 run phi_aria phi_p3dx --ros-args -p publish_aria_lasers:=true -p port:=192.168.1.11:10002```
