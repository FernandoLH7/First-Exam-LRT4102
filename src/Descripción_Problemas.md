# Descripción de los Problemas

Este documento explica cómo resolví los problemas planteados en el examen. Aquí detallo paso a paso lo que hice en los programas `problema1.py` y `problema2.py`, explicando las decisiones que tomé y cómo llegué a la solución. Espero que quede claro y fácil de entender.

---

## Problema 1: Manejo de Tortugas en Turtlesim

### **¿Qué hace este programa?**
El programa `problema1.py` interactúa con el simulador `turtlesim` para hacer dos cosas principales:
1. Borrar la tortuga que aparece por defecto en el simulador (`turtle1`).
2. Crear varias tortugas en posiciones aleatorias dentro del área del simulador.

### **¿Cómo lo resolví?**
1. **Primero, inicialicé el nodo de ROS**:
   - Usé la librería `rospy` para crear un nodo llamado `nodo_tortugas`. Esto es obligatorio para que el programa pueda comunicarse con ROS.
   - Básicamente, el nodo es como el "cerebro" del programa que controla todo.

2. **Después, eliminé la tortuga inicial**:
   - Usé el servicio `/kill` que viene con `turtlesim`. Este servicio permite eliminar tortugas por su nombre.
   - Creé un método llamado `borrar_tortuga_inicial` que espera a que el servicio esté disponible y luego lo llama para borrar la tortuga llamada `turtle1`.

3. **Luego, generé tortugas en posiciones aleatorias**:
   - Usé el servicio `/spawn` para crear nuevas tortugas.
   - En el método `crear_tortugas_random`, generé coordenadas aleatorias para las posiciones de las tortugas usando la librería `random`.
   - Les di nombres únicos como `tortuga1`, `tortuga2`, etc., para que no hubiera conflictos.

4. **Finalmente, lo junté todo en el bloque principal**:
   - Creé una instancia de la clase `Tortugas`.
   - Llamé al método para borrar la tortuga inicial y luego al método para crear 5 tortugas nuevas.

### **¿Por qué lo hice así?**
- Usé una clase (`Tortugas`) para organizar mejor el código y que fuera más fácil de entender.
- Agregué mensajes de log (`rospy.loginfo`, `rospy.logwarn`, etc.) para que el programa me dijera qué estaba pasando en cada momento.

---

## Problema 2: Control de Movimiento de Tortugas

### **¿Qué hace este programa?**
El programa `problema2.py` controla el movimiento de las tortugas creadas en el simulador `turtlesim`. La idea es que las tortugas se muevan de acuerdo a ciertas reglas que definí.

### **¿Cómo lo resolví?**
1. **Primero, inicialicé el nodo de ROS**:
   - Igual que en el problema 1, creé un nodo llamado `nodo_movimiento` para manejar todo lo relacionado con el movimiento de las tortugas.

2. **Después, me suscribí a los tópicos de posición**:
   - Usé el tópico `/turtleX/pose` (donde `X` es el número de la tortuga) para obtener la posición actual de cada tortuga.
   - Esto me permitió saber dónde estaba cada tortuga en todo momento.

3. **Luego, publiqué comandos de velocidad**:
   - Usé el tópico `/turtleX/cmd_vel` para enviar comandos de velocidad lineal y angular a las tortugas.
   - Básicamente, esto es lo que hace que las tortugas se muevan.

4. **Implementé la lógica de movimiento**:
   - Creé un bucle que calcula hacia dónde deben moverse las tortugas y con qué velocidad.
   - También agregué condiciones para evitar que las tortugas se salgan del área del simulador.

### **¿Por qué lo hice así?**
- Usé suscriptores y publicadores porque es la forma estándar de interactuar con los tópicos en ROS.
- Agregué mensajes de log para saber qué estaba haciendo cada tortuga en cada momento.

---

## ¿Cómo ejecutar los programas?

### **Requisitos previos**
1. Tener instalado ROS y el paquete `turtlesim`.
2. Asegurarte de que los scripts `problema1.py` y `problema2.py` sean ejecutables. Para eso, usa este comando en la terminal:
   ```bash
   chmod +x problema1.py problema2.py