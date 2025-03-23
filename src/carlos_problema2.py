#!/usr/bin/env python3

# Importamos las librerías necesarias para trabajar con ROS y turtlesim
import rospy
from geometry_msgs.msg import Twist  # Para controlar la velocidad de la tortuga
from turtlesim.msg import Pose  # Para obtener la posición de la tortuga
from turtlesim.srv import Spawn, Kill  # Para crear y eliminar tortugas
from std_srvs.srv import Empty  # Para limpiar el área de trabajo
import time  # Para manejar pausas en la ejecución

class TurtleController:
    def __init__(self):
        # Inicializamos el nodo de ROS
        rospy.init_node('turtle_control')

        # Variables iniciales para el control de la tortuga
        self.pub = None  # Publicador para enviar comandos de velocidad
        self.x_actual = 0  # Posición actual en X de la tortuga
        self.y_actual = 0  # Posición actual en Y de la tortuga
        self.rate = rospy.Rate(10)  # Frecuencia de actualización (10 Hz)

        # Dimensiones del área de trabajo de turtlesim
        self.ancho_area = 11
        self.alto_area = 11

        # Eliminamos la tortuga por defecto para empezar con un área limpia
        self.eliminar_tortuga('turtle1')

    def actualizar_posicion(self, pose):
        """Callback para actualizar la posición actual de la tortuga."""
        # Aquí actualizamos las variables de posición cada vez que recibimos datos del tópico de posición
        self.x_actual = pose.x
        self.y_actual = pose.y

    def suscribirse_posicion(self, nombre_tortuga):
        """Suscribirse al tópico de posición de la tortuga."""
        # Nos suscribimos al tópico de posición de la tortuga para recibir actualizaciones en tiempo real
        rospy.Subscriber(f'/{nombre_tortuga}/pose', Pose, self.actualizar_posicion)

    def configurar_publicador(self, nombre_tortuga):
        """Configurar el publicador de velocidad para la tortuga."""
        # Configuramos el publicador para enviar comandos de velocidad a la tortuga
        self.pub = rospy.Publisher(f'/{nombre_tortuga}/cmd_vel', Twist, queue_size=10)

    def crear_tortuga(self, x, y, nombre):
        """Crear una tortuga en las coordenadas dadas."""
        # Usamos el servicio de ROS para crear una nueva tortuga en las coordenadas especificadas
        rospy.wait_for_service('/spawn')
        try:
            spawn = rospy.ServiceProxy('/spawn', Spawn)
            spawn(x, y, 0.0, nombre)
            rospy.loginfo(f"Tortuga '{nombre}' creada en ({x}, {y}).")
        except rospy.ServiceException as e:
            rospy.logerr(f"No se pudo crear la tortuga '{nombre}': {e}")

    def eliminar_tortuga(self, nombre):
        """Eliminar una tortuga."""
        # Usamos el servicio de ROS para eliminar una tortuga existente
        rospy.wait_for_service('/kill')
        try:
            kill = rospy.ServiceProxy('/kill', Kill)
            kill(nombre)
            rospy.loginfo(f"Tortuga '{nombre}' eliminada.")
        except rospy.ServiceException as e:
            rospy.logerr(f"No se pudo eliminar la tortuga '{nombre}': {e}")

    def mover_a_punto(self, x_destino, y_destino):
        """Mover la tortuga a un punto usando control proporcional."""
        # Aquí implementé un control proporcional para mover la tortuga a un punto específico
        Kp = 1.0  # Ganancia proporcional. Elegí este valor porque es suficiente para un control suave

        while not rospy.is_shutdown():
            # Calculamos el error entre la posición actual y la posición deseada
            error_x = x_destino - self.x_actual
            error_y = y_destino - self.y_actual

            # Condición de parada: si el error es pequeño, consideramos que llegamos al destino
            if abs(error_x) < 0.05 and abs(error_y) < 0.05:
                rospy.loginfo(f"Llegó al punto: ({x_destino}, {y_destino})")
                break

            # Calculamos las velocidades proporcionales
            vel_x = Kp * error_x
            vel_y = Kp * error_y

            # Limité la velocidad máxima para evitar movimientos bruscos que puedan desestabilizar el control
            vel_x = max(min(vel_x, 2.0), -2.0)
            vel_y = max(min(vel_y, 2.0), -2.0)

            # Creamos el mensaje de velocidad y lo publicamos
            msg = Twist()
            msg.linear.x = vel_x
            msg.linear.y = vel_y

            self.pub.publish(msg)
            rospy.loginfo(f"Posición actual: ({self.x_actual}, {self.y_actual}), Error: ({error_x}, {error_y})")
            self.rate.sleep()

        # Detenemos la tortuga explícitamente después de llegar al destino
        msg = Twist()
        self.pub.publish(msg)
        rospy.loginfo("Tortuga detenida.")

    def validar_espacio(self, puntos):
        """Verificar si los puntos caben en el área de trabajo."""
        # Aquí verificamos si todos los puntos de la figura están dentro del área de trabajo
        for x, y in puntos:
            if x < 0 or x > self.ancho_area or y < 0 or y > self.alto_area:
                return False
        return True

    def limpiar_area(self):
        """Borrar todo lo trazado en el área de trabajo."""
        # Usamos el servicio de ROS para limpiar el área de trabajo
        rospy.wait_for_service('/clear')
        try:
            clear = rospy.ServiceProxy('/clear', Empty)
            clear()
            rospy.loginfo("Área de trabajo limpiada.")
        except rospy.ServiceException as e:
            rospy.logerr(f"No se pudo limpiar el área de trabajo: {e}")

    def mostrar_menu_post_ejecucion(self):
        """Mostrar un menú después de completar una figura."""
        # Este menú permite al usuario elegir entre regresar al menú principal o salir del programa
        while True:
            print("\n¿Qué deseas hacer ahora?")
            print("1 -> Elegir otra opción")
            print("2 -> Salir")
            opcion = input("Selecciona una opción: ")

            if opcion == '1':
                return True  # Regresar al menú principal
            elif opcion == '2':
                print("Saliendo...")
                return False  # Salir del programa
            else:
                print("Opción no válida. Intenta de nuevo.")

    def dibujar_triangulo(self, x_inicio, y_inicio):
        """Dibujar un triangulo."""
        rospy.loginfo("Dibujando un triángulo...")

        # Calculamos las esquinas del triángulo
        puntos = [
            (x_inicio + 3, y_inicio),
            (x_inicio+1.5, y_inicio + 3),
            (x_inicio, y_inicio)
        ]

        # Mostramos las coordenadas de las esquinas al usuario
        print("Coordenadas del triangulo a dibujar:")
        for i, (x, y) in enumerate(puntos):
            print(f"Esquina {i + 1}: ({x}, {y})")

        # Validamos si el triangulo cabe en el área de trabajo
        if not self.validar_espacio(puntos):
            print("El triangulo no cabe en el área de trabajo. Intenta con otras coordenadas.")
            return False

        # Movemos la tortuga a cada esquina del triangulo
        for x, y in puntos:
            self.mover_a_punto(x, y)
            time.sleep(1)

        print("¡triangulo terminado!")
        return True

    def dibujar_trapecio_isosceles(self, x_inicio, y_inicio):
        """Dibujar un trapecio isósceles."""
        rospy.loginfo("Dibujando un trapecio isósceles...")

        # Calculamos las esquinas del trapecio isósceles
        puntos = [
            (x_inicio + 4, y_inicio),
            (x_inicio + 3, y_inicio + 2),
            (x_inicio + 1, y_inicio + 2),
            (x_inicio, y_inicio)
        ]

        # Mostramos las coordenadas de las esquinas al usuario
        print("Coordenadas del trapecio isósceles a dibujar:")
        for i, (x, y) in enumerate(puntos):
            print(f"Esquina {i + 1}: ({x}, {y})")

        # Validamos si el trapecio isósceles cabe en el área de trabajo
        if not self.validar_espacio(puntos):
            print("El trapecio isósceles no cabe en el área de trabajo. Intenta con otras coordenadas.")
            return False

        # Movemos la tortuga a cada esquina del trapecio isósceles
        for x, y in puntos:
            self.mover_a_punto(x, y)
            time.sleep(1)

        print("¡trapecio isósceles terminado!")
        return True

    def menu(self):
        """Menú principal."""
        while not rospy.is_shutdown():
            print("\nControl de Tortugas")
            print("1 -> Dibujar triangulo")
            print("2 -> Dibujar trapecio isósceles")
            print("3 -> Limpiar área de trabajo")
            print("4 -> Salir")

            opcion = input("Selecciona una opción: ")

            if opcion == '4':
                print("Saliendo...")
                break

            elif opcion == '3':
                print("Opción seleccionada: Limpiar área de trabajo")
                self.limpiar_area()

            elif opcion == '1':
                print("Opción seleccionada: Dibujar triángulo")

                while True:
                    print("\nCoordenadas para el triangulo")
                    x_triangulo = float(input("Coordenada x: "))
                    y_triangulo = float(input("Coordenada y: "))

                    # Mostramos las coordenadas de las esquinas antes de validar
                    puntos = [
                        (x_triangulo + 3, y_triangulo),
                        (x_triangulo+1.5, y_triangulo + 3),
                        (x_triangulo, y_triangulo)
                    ]
                    print("Coordenadas del triangulo a dibujar:")
                    for i, (x, y) in enumerate(puntos):
                        print(f"Esquina {i + 1}: ({x}, {y})")

                    # Validamos si el triangulo cabe en el área de trabajo
                    if not self.validar_espacio(puntos):
                        print("El triangulo no cabe en el área. Intenta con otras coordenadas.")
                        continue

                    self.crear_tortuga(x_triangulo, y_triangulo, 'turtle1')
                    self.suscribirse_posicion('turtle1')
                    self.configurar_publicador('turtle1')
                    if self.dibujar_triangulo(x_triangulo, y_triangulo):
                        self.eliminar_tortuga('turtle1')
                        break
                    self.eliminar_tortuga('turtle1')

                # Mostramos el menú post ejecución
                if not self.mostrar_menu_post_ejecucion():
                    break

            elif opcion == '2':
                print("Opción seleccionada: Dibujar trapecio isósceles")

                while True:
                    print("\nCoordenadas para el trapecio isósceles")
                    x_trapecio_isosceles = float(input("Coordenada x: "))
                    y_trapecio_isosceles = float(input("Coordenada y: "))

                    # Mostramos las coordenadas de las esquinas antes de validar
                    puntos = [
                        (x_trapecio_isosceles + 4, y_trapecio_isosceles),
                        (x_trapecio_isosceles + 3, y_trapecio_isosceles + 2),
                        (x_trapecio_isosceles + 1, y_trapecio_isosceles + 2),
                        (x_trapecio_isosceles, y_trapecio_isosceles)
                    ]
                    print("Coordenadas del trapecio isósceles a dibujar:")
                    for i, (x, y) in enumerate(puntos):
                        print(f"Esquina {i + 1}: ({x}, {y})")

                    # Validamos si el trapecio isósceles cabe en el área de trabajo
                    if not self.validar_espacio(puntos):
                        print("El trapecio isósceles no cabe en el área. Intenta con otras coordenadas.")
                        continue

                    self.crear_tortuga(x_trapecio_isosceles, y_trapecio_isosceles, 'turtle2')
                    self.suscribirse_posicion('turtle2')
                    self.configurar_publicador('turtle2')
                    if self.dibujar_trapecio_isosceles(x_trapecio_isosceles, y_trapecio_isosceles):
                        self.eliminar_tortuga('turtle2')
                        break
                    self.eliminar_tortuga('turtle2')

                # Mostramos el menú post ejecución
                if not self.mostrar_menu_post_ejecucion():
                    break

if __name__ == '__main__':
    try:
        control = TurtleController()
        control.menu()
    except rospy.ROSInterruptException:
        pass