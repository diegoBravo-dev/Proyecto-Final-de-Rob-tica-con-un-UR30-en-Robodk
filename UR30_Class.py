from numpy import cos, sin , array, eye, pi, deg2rad

class UR30:

    """
    Constructor:
        -> Este Contructor contiene los atributos básicos de un UR30 para poder obtener su
            Cinemática Directa y aplicar Cinemática Inversa o Control Cinemático
    """

    def __init__(self, θ):
        self.θ = deg2rad(θ)
        self.__a = [0, -0.6370, -0.5037, 0, 0, 0]
        self.__d = [0.2363, 0, 0, 0.2010, 0.1593, 0.1543]
        self.__α = [pi/2, 0, 0, pi/2, -pi/2, 0]
        self.__tool = array([[0.866025, 0, 0.5,      0.050],
                           [0,        1, 0,        0],
                           [-0.5,     0, 0.866025, 0.450],
                           [0,        0, 0,        1]])
        
        self.T = self.CinematicaDirecta()

        self.Pos = self.Posicion()  
    
    """
    Función que obtiene la cinemática directa del UR30

    """

    def CinematicaDirecta(self):
         T = eye(4)

         for i in range(6):
            Aj = array([[cos(self.θ[i]), -sin(self.θ[i])*cos(self.__α[i]), sin(self.θ[i])*sin(self.__α[i]), self.__a[i]*cos(self.θ[i])],
                        [sin(self.θ[i]), cos(self.θ[i])*cos(self.__α[i]), -cos(self.θ[i])*sin(self.__α[i]), self.__a[i]*sin(self.θ[i])],
                        [0,              sin(self.__α[i]),                 cos(self.__α[i]),                self.__d[i]],
                        [0,              0,                                0,                               1]])
            T @= Aj

         T @= self.__tool 

         return T 

    """
    Función que obtiene la posición en x, y y z del Efector Final del robot

    """ 

    def Posicion(self):
        return self.T[:3, 3] * 1000
    