#!/usr/bin/env python3

# Aca importé las librerías necesarias
import rospy  # Esta es la librería de Python para trabajar con ROS
from turtlesim.srv import Spawn, Kill  # Importo los servicios para crear y eliminar tortugas
import random  # Libreria random para generar números aleatorios

# Clase para manejar las tortugas en el simulador
class Tortugas:
    def __init__(self):
        # Inicializo el nodo de ROS. Esto es obligatorio para que el script funcione con ROS.
        rospy.init_node('nodo_tortugas')  # Le pongo un nombre al nodo, en este caso 'nodo_tortugas'

    def borrar_tortuga_inicial(self):
        """
        Este método elimina la tortuga que aparece por defecto en el simulador (turtle1).
        Es importante porque queremos empezar con un espacio limpio.
        """
        rospy.wait_for_service('/kill')  # Espero a que el servicio '/kill' esté disponible
        try:
            # Creo un proxy para conectarme al servicio '/kill' y poder usarlo
            borrar = rospy.ServiceProxy('/kill', Kill)
            borrar('turtle1')  # Llamo al servicio para eliminar la tortuga llamada 'turtle1'
            rospy.loginfo("Se borró la tortuga inicial.")  # Mensaje para confirmar que se eliminó
        except rospy.ServiceException as errorcito:
            # Si algo falla, imprimo un mensaje de advertencia con el error
            rospy.logwarn(f"No se pudo borrar la tortuga inicial: {errorcito}")

    def crear_tortugas_random(self, cantidad):
        """
        Este método crea varias tortugas en posiciones aleatorias dentro del área del simulador.
        La cantidad de tortugas a crear se pasa como parámetro.
        """
        rospy.wait_for_service('/spawn')  # Espero a que el servicio '/spawn' esté disponible
        try:
            # Creo un proxy para conectarme al servicio '/spawn' y poder usarlo
            crear = rospy.ServiceProxy('/spawn', Spawn)
            for i in range(cantidad):  # Uso un bucle para crear la cantidad de tortugas que se pidió
                # Genero coordenadas aleatorias para la posición de la tortuga
                pos_x = random.uniform(0.5, 10.5)  # Coordenada x aleatoria (dentro del rango del simulador)
                pos_y = random.uniform(0.5, 10.5)  # Coordenada y aleatoria (dentro del rango del simulador)
                nombre_tortuga = f'tortuga{i+1}'  # Le doy un nombre único a cada tortuga (tortuga1, tortuga2, etc.)
                crear(pos_x, pos_y, 0.0, nombre_tortuga)  # Llamo al servicio para crear la tortuga
                # Mensaje para confirmar que la tortuga se creó y mostrar su posición
                rospy.loginfo(f"Se creó la tortuga '{nombre_tortuga}' en ({pos_x:.2f}, {pos_y:.2f})")
        except rospy.ServiceException as errorcito:
            # Si algo falla, imprimo un mensaje de error con el detalle
            rospy.logerr(f"No se pudieron crear las tortugas: {errorcito}")

# Bloque principal del programa
if __name__ == '__main__':
    try:
        # Creo una instancia de la clase Tortugas para manejar todo
        manejador_tortugas = Tortugas()
        manejador_tortugas.borrar_tortuga_inicial()  # Llamo al método para borrar la tortuga inicial
        manejador_tortugas.crear_tortugas_random(5)  # Llamo al método para crear 5 tortugas en posiciones random
    except rospy.ROSInterruptException:
        # Si el programa se interrumpe (por ejemplo, con Ctrl+C), simplemente salgo sin errores
        pass
