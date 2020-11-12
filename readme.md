# Joy To Vesc

## Facultad de Ingenieria del Ejercito

Paquete ROS que maneja el motor de direccion y los finales de carrera que limitan el movimiento del mismo. El motor es controlado mediante el bus i2c de un Jetson TX2 y los finales de carrera estan conectados al GPIO del Jetson TX2.

### Requisitos

* ROS Melodic
* Instalar los paquetes de requirments.txt

### Instalacion

```bash
cd ~/catkin_ws/src
git clone https://github.com/carlosmgc2003/direction_ros.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="direction_ros"

```

### Utilizacion

```bash
roscd
python3 ../src/direction/scripts/direction_handler.py
```

Este paquete esta preparado para funcionar con un nodo joy_to_vesc y un nodo joy inicializados.

### Pendiente

* Control de la direccion con un angulo.
* Lazo de control.