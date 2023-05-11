# turtlebot3-camarero

![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/jbg73/turtlebot3-camarero)

Estos paquetes de ros permiten usar el robot turtlebot3 en el entorno gazebo simulando la tarea de un camarero. Tiene unas posiciones predefinidas que simulan las mesas, se le envia como orden que acuda a una de ellas, tras llegar debe leer el codigo QR de la mesa y confirmar que ha llegado y finalmente volver a su posicion home


-------------------PUESTA EN MARCHA-----------------------------
En primer lugar hay que añadir el mundo creado a gazebo, este esta definido en mi_mapa_ok.world. Hay que incluirlo en la carpeta world de turtlebot3_gazebo (suele en encontrarse en /opt/ros/noetic/share/turtlebot3_gazebo/worlds). Luego hay que configurar el fichero .launch que lanza este mundo, en este caso habria que sustituir el fichero turtlebot3_world.launch original por el que se proporciona en este repositorio con  el mismo nombre (se suele encontrar en /opt/ros/noetic/share/turtlebot3_gazebo/launch).

Para poder usar los QR creados en gazebo, los cuales estan en la carpeta gazebo_models, es necesario añadir la ruta de dicha carpeta en gazebo y posteriormente añadirlos a mano en las distintas mesas (cubos en gazebo).

Para poder usar el entorno RViz para la navegacion, es necesario descargar el fichero res_map_final_ok.yaml que es el resultado del mapeado realizado

A continuacion, descargar los paquetes practica3(es el que hace la tarea de vision) y move_turtlebot(navegacion autonoma) e incluirlos en el workspace


-----------------EJECUCION-------------------------------------

Finalmente, para su ejecucion, hay que seguir los siguientes pasos: 
(En cada terminal que se use turtlebot3 escribir previamente export TURTLEBOT3_MODEL=waffle)

1. roslaunch turtlebot3_gazebo turtlebot3_world.launch
2. roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/$HOME/user/map_name.yaml
3. roslaunch move_turtlebot mi_launch.launch dest:=0/1/2
