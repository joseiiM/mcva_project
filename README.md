# Robot Lazarillo
*Autor: Jose Manuel Pérez Molerón*

Este proyecto se trata del proyecto final de la asignatura Métodos Computacionales en vida artificial del master en Lógica, Computación e Inteligencia Artificial de la Universidad de Sevilla. 

 ## Introducción
 
 Este proyecto presenta una simulación de un robot lazarillo
el cuál sigue a un agente concreto, el cual sería su dueño, a
cierta distancia. Para conseguir este objetivo el robot buscará a
lo largo del mapa a si objetivo hasta que, tras encontrarlo, este
llevará a cabo una planificación de la trayectoria a seguir cada
cierto tiempo utilizando el algoritmo Theta *, ya que este
algoritmo nos da como resultado caminos mas cortos y realistas
que otros algoritmos de planificación como Dijkstra.

A lo largo del mapa el robot encontrará distintos obstáculos y otros
agentes en movimiento, lo cuál le supondrá un desafío para poder 
seguir su objetivo a la vez que esquiva estos
obstáculos. Para conseguir esto se utiliza un SFM (Social
Force Model). Este modelo nos permite plantear el modelo del
movimiento del robot como una suma de vectores de fuerzas
teniendo en cuenta una fuerza atractiva hacia el objetivo y unas
fuerzas repulsivas a los obstáculos y otras personas en el entorno.

Este proyecto está basado en ROS (Robotic Operating System), por
lo que será necesario tener instalado ROS en tu dispositivo.

## Instalación y dependencias

Para la utilizar este paquete de ROS será necesario tener instalado en
tu dispositivo ROS en su versión Noetic. Para la instalación de ROS Noetic 
debes seguir los pasos que se indican en el siguiente enlace.

- ***Instalación de ROS Noetic***: http://wiki.ros.org/noetic/Installation/Ubuntu

Una vez instalado ROS Noetic es necesario crear un workspace donde vamos a trabajar. 
En el siguiente enlace se indican los pasos a seguir para crear un workspace.

- ***Crear un workspace***:  http://wiki.ros.org/ROS/Tutorials/CreatingPackage

Para finalizar con la instalación debes clonar este repositorio en la carpeta /src de 
tu workspace.

Es necesario utilizar una versión de python 3.8 o superior para poder ejecutar este proyecto.
Además, este proyecto utiliza el simulador TARS, el cual es un simulador multi-agente basado en 
el modelo de fuerzas sociales SFM (Social Force Model) y que nos permitirá manejar nuestro robot
en un entorno simulado. Para utilizar TARS será necesario clonar el repositorio en la carpeta /src de
tu workspace.

- ***TARS***: https://github.com/Ignacio-Perez/TARS

# Modo de uso

Para ejecutar el proyecto debes ejecutar el siguiente comando:
```
roslaunch mcva_project planner.launch
```
Tras ejecutar el comando se abrirá una ventana con el visualizador rviz de ROS
donde podremos observar el mapa con los obstáculos remarcados en negro, una serie
de agentes en movimiento y nuestro robot.

El robot comenzará buscando a su objetivo recorriendo el mapa de manera aleatoria 
hasta que lo encuentre. Una vez encontrado el objetivo, el robot comenzará a planificar 
la ruta más corta encontrada a través del algoritmo Theta * hacia la posición del objetivo
cada cierto tiempo, de manera que vaya siguiendo a su objetivo. En caso de perderlo de 
durante un largo periodo de tiempo, este volverá de nuevo a buscar a su objetivo de
manera aleatoria por el mapa hasta encontrarlo de nuevo.
