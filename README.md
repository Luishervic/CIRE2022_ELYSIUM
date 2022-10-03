# CIRE2022.
## Entregables de las distintas etapas del Concurso Iberoamericano de Robótica Espacial 2022.
- Equipo: Elysium.
- Institución: TECNM Campus Uruapan.
## _ETAPAS_
### 01
Capacitación de iniciación en ROS.
### 02 
Usamos el enfoque de maquina de estados para realizar nuestra programacion de evasión de obstaculos, añadiendo la busqueda del punto meta. Lo anterior considerando que, en determinado momento éste puede cambiar. Al llegar al punto el robot se mantiene a la espera, evaluando si cambia la meta para volver a ejecutar la rutina programada de busqueda.
### 03
Se modificó la máquina de estados para realizar la detección de los objetos, este modo en cada uno de los estados se publica el punto meta alrededor de los puntos de referencia que nos otorgaron, para dar paso a la detección por medio de la cámara, ya que utilizamos la detección de color para aproximar la ubicación de los objetos. 
Fue necesario realizar distintas operaciones morfológicas a las imágenes para que los contornos coincidieran con la figura y así calcular su centro de masa. La rutina finaliza cuando el robot regresa a la ubicación (0,0) con referencia al mapa. :)
