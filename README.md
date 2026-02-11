[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)

Primero que nada se necesita preparar el entorno de trabajo para controlar el Dron Bebop Parrot 2 para controlarlo con ROS 1 Noetic desde la versión de Ubuntu 20.04, esto se puede hacer siguiendo los pasos descritos en este [repositorio](https://github.com/ricardoRamoM/bebop_ws). Y se creara un package con el nombre bebop_tmr.

Se puede tenr un boton para activar el despegue y el aterrizaje, sin embargo, la ejecución de los movimientos son de forma autónoma. 

Se pueden usar un máximo 3 drones a la vez. Cada dron debe de pesar máximo 300 gramos, si alguno pesa más solo se permite usar 1.

Se puede usar más de un dron pero se usa solo uno a la vez durante cada misión.

Un piloto de seguridad deberá actuar en caso de emergencia y activar el aterrizaje de emergencia con la tecla "  ".

Las baterías no utilizadas deberán almacenarse en bolsas de seguridad adecuadas y
cargarse únicamente en la estación designada.

Las misiones se ejecutarán una a la vez. 

Se tendrá un aproximado entre 15 y 20 minutos para completar tantas misiones como sea posible.

No habrá humo de la mision 4.

PENDIENTES

- Revisar la duración de las baterías.
- Probar la distancia máxima a la que el dron pierde la conexion con la Laptop. Y en dado caso de requerirlo intenta usar un modem externo y volver a hacer la prueba de distancia.
- Entrenar las redes neuronales para las ventanas azules, las verdes y las rojas.
- Revisar cómo cambiar el aterrizaje para que apague los motores más abajo de lo normal (aprox 1 metro)
- Juntar la red entrenada por Sam e integrarla para usarla con el aterrizaje.
- Entrenar avanzar y evitar los tubos naranjas (puede ser con una red entrenada).
- Mecanismo para liberación del kit
- Mecanismo para pintar en el pizarron.
- DEFINIR EL ORDEN DE PRIORIDAD DE LAS MISIONES QUE VAMOS A REALIZAR. (NO INTENTAR HACER TODO A LA VEZ, ES POCO A POCO)
- Para la mision del pizarron, ver como identifiar arucos.
- Si bien si movemos el dron con odometria, tenemos que hacer que avance exactamente solo la distancia que queremos o sea muy cercano. Se me ocurre que se puede hacer con control PID en estos instantes.
- Codigo para hacer que avance y mantenga la altura para pasar las ventanas
- Recubrir las ventanas con las cintas del color actual para hacer las pruebas.

---

<a id="indice"></a>

## 📋 Índice
- [📦 Estructura del Proyecto](#estructura-del-proyecto)
- [✅ Requisitos](#requisitos)
- [🔧 Instalación desde Cero](#instalacion-desde-cero)
- [▶️ Uso del Drone Parrot Bebop 2](#uso-del-drone-parrot-bebop-2)
  - [1️⃣ Conexión con el dron](#conexion-con-el-dron)
  - [2️⃣ Iniciar ROS](#iniciar-ros)
  - [3️⃣ Lanzar el Nodo Principal](#lanzar-el-nodo-principal)
  - [4️⃣ Comandos Básicos](#comandos-basicos)
  - [5️⃣ Verificar Tópicos Disponibles](#verificar-topicos-disponibles)
  - [6️⃣ Ver la Cámara](#ver-la-camara)
  - [7️⃣ Visualizar Nodos y Tópicos (rqt_graph)](#visualizar-nodos-y-topicos-rqt-graph)
  - [8️⃣ Ejemplo Python - Vuelo Simple](#ejemplo-python-vuelo-simple)
  - [9️⃣ Diagrama Básico del Flujo de Vuelo](#diagrama-basico-del-flujo-de-vuelo)



 
---

<a id="estructura-del-proyecto"></a>

## 📦 Estructura del Proyecto

```
bebop_ws/
 ├── build/
 ├── devel/
 └── src/
      ├── parrot_arsdk       # Wrapper SDK Parrot
      └── bebop_autonomy     # Driver principal ROS
```

[🔙 Volver al Índice](#indice)

---

<a id="requisitos"></a>

## ✅ Requisitos