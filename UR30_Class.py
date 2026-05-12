from numpy import cos, sin , array, eye, pi, deg2rad, acos, matrix, rad2deg, dot, arange, zeros, sqrt, transpose


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

        self.Ori = self.Orientacion()

    """
    Ecuación diferencial que resuelve el control cinemático 
    """

    def f(self, t, θs):
        θ1 = θs[0]
        θ2 = θs[1]
        θ3 = θs[2]
        θ4 = θs[3]
        θ5 = θs[4]
        θ6 = θs[5]

        # Definir el control cinemático
        x = 0.05*sin(θ1)*sin(θ5)*cos(θ6) + 0.6043*sin(θ1)*cos(θ5) + 0.201*sin(θ1) - 0.6043*sin(θ5)*cos(θ1)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ6)*sin(θ2 + θ3 + θ4)*cos(θ1) + 0.1593*sin(θ2 + θ3 + θ4)*cos(θ1) - 0.637*cos(θ1)*cos(θ2) + 0.05*cos(θ1)*cos(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4) - 0.5037*cos(θ1)*cos(θ2 + θ3)
        y = -0.6043*sin(θ1)*sin(θ5)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ1)*sin(θ6)*sin(θ2 + θ3 + θ4) + 0.1593*sin(θ1)*sin(θ2 + θ3 + θ4) - 0.637*sin(θ1)*cos(θ2) + 0.05*sin(θ1)*cos(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4) - 0.5037*sin(θ1)*cos(θ2 + θ3) - 0.05*sin(θ5)*cos(θ1)*cos(θ6) - 0.6043*cos(θ1)*cos(θ5) - 0.201*cos(θ1)
        z = -0.637*sin(θ2) - 0.6043*sin(θ5)*sin(θ2 + θ3 + θ4) + 0.05*sin(θ6)*cos(θ2 + θ3 + θ4) - 0.5037*sin(θ2 + θ3) + 0.05*sin(θ2 + θ3 + θ4)*cos(θ5)*cos(θ6) - 0.1593*cos(θ2 + θ3 + θ4) + 0.2363 

        X = array([[x], [y], [z]])

        # Jacobiano
        j11 = 0.6043*sin(θ1)*sin(θ5)*cos(θ2 + θ3 + θ4) + 0.05*sin(θ1)*sin(θ6)*sin(θ2 + θ3 + θ4) - 0.1593*sin(θ1)*sin(θ2 + θ3 + θ4) + 0.637*sin(θ1)*cos(θ2) - 0.05*sin(θ1)*cos(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4) + 0.5037*sin(θ1)*cos(θ2 + θ3) + 0.05*sin(θ5)*cos(θ1)*cos(θ6) + 0.6043*cos(θ1)*cos(θ5) + 0.201*cos(θ1)
        j12 = 0.637*sin(θ2)*cos(θ1) + 0.6043*sin(θ5)*sin(θ2 + θ3 + θ4)*cos(θ1) - 0.05*sin(θ6)*cos(θ1)*cos(θ2 + θ3 + θ4) + 0.5037*sin(θ2 + θ3)*cos(θ1) - 0.05*sin(θ2 + θ3 + θ4)*cos(θ1)*cos(θ5)*cos(θ6) + 0.1593*cos(θ1)*cos(θ2 + θ3 + θ4)
        j13 = 0.6043*sin(θ5)*sin(θ2 + θ3 + θ4)*cos(θ1) - 0.05*sin(θ6)*cos(θ1)*cos(θ2 + θ3 + θ4) + 0.5037*sin(θ2 + θ3)*cos(θ1) - 0.05*sin(θ2 + θ3 + θ4)*cos(θ1)*cos(θ5)*cos(θ6) + 0.1593*cos(θ1)*cos(θ2 + θ3 + θ4)
        j14 = 0.6043*sin(θ5)*sin(θ2 + θ3 + θ4)*cos(θ1) - 0.05*sin(θ6)*cos(θ1)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ2 + θ3 + θ4)*cos(θ1)*cos(θ5)*cos(θ6) + 0.1593*cos(θ1)*cos(θ2 + θ3 + θ4)
        j15 = -0.6043*sin(θ1)*sin(θ5) + 0.05*sin(θ1)*cos(θ5)*cos(θ6) - 0.05*sin(θ5)*cos(θ1)*cos(θ6)*cos(θ2 + θ3 + θ4) - 0.6043*cos(θ1)*cos(θ5)*cos(θ2 + θ3 + θ4)
        j16 = -0.05*sin(θ1)*sin(θ5)*sin(θ6) - 0.05*sin(θ6)*cos(θ1)*cos(θ5)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ2 + θ3 + θ4)*cos(θ1)*cos(θ6)

        j21 = 0.05*sin(θ1)*sin(θ5)*cos(θ6) + 0.6043*sin(θ1)*cos(θ5) + 0.201*sin(θ1) - 0.6043*sin(θ5)*cos(θ1)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ6)*sin(θ2 + θ3 + θ4)*cos(θ1) + 0.1593*sin(θ2 + θ3 + θ4)*cos(θ1) - 0.637*cos(θ1)*cos(θ2) + 0.05*cos(θ1)*cos(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4) - 0.5037*cos(θ1)*cos(θ2 + θ3)
        j22 = 0.637*sin(θ1)*sin(θ2) + 0.6043*sin(θ1)*sin(θ5)*sin(θ2 + θ3 + θ4) - 0.05*sin(θ1)*sin(θ6)*cos(θ2 + θ3 + θ4) + 0.5037*sin(θ1)*sin(θ2 + θ3) - 0.05*sin(θ1)*sin(θ2 + θ3 + θ4)*cos(θ5)*cos(θ6) + 0.1593*sin(θ1)*cos(θ2 + θ3 + θ4)
        j23 = 0.6043*sin(θ1)*sin(θ5)*sin(θ2 + θ3 + θ4) - 0.05*sin(θ1)*sin(θ6)*cos(θ2 + θ3 + θ4) + 0.5037*sin(θ1)*sin(θ2 + θ3) - 0.05*sin(θ1)*sin(θ2 + θ3 + θ4)*cos(θ5)*cos(θ6) + 0.1593*sin(θ1)*cos(θ2 + θ3 + θ4)
        j24 = 0.6043*sin(θ1)*sin(θ5)*sin(θ2 + θ3 + θ4) - 0.05*sin(θ1)*sin(θ6)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ1)*sin(θ2 + θ3 + θ4)*cos(θ5)*cos(θ6) + 0.1593*sin(θ1)*cos(θ2 + θ3 + θ4)
        j25 = -0.05*sin(θ1)*sin(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4) - 0.6043*sin(θ1)*cos(θ5)*cos(θ2 + θ3 + θ4) + 0.6043*sin(θ5)*cos(θ1) - 0.05*cos(θ1)*cos(θ5)*cos(θ6)
        j26 = -0.05*sin(θ1)*sin(θ6)*cos(θ5)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ1)*sin(θ2 + θ3 + θ4)*cos(θ6) + 0.05*sin(θ5)*sin(θ6)*cos(θ1)

        j31 = 0
        j32 = -0.6043*sin(θ5)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ6)*sin(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4) - 0.637*cos(θ2) + 0.05*cos(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4) - 0.5037*cos(θ2 + θ3)
        j33 = -0.6043*sin(θ5)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ6)*sin(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4) + 0.05*cos(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4) - 0.5037*cos(θ2 + θ3)
        j34 = -0.6043*sin(θ5)*cos(θ2 + θ3 + θ4) - 0.05*sin(θ6)*sin(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4) + 0.05*cos(θ5)*cos(θ6)*cos(θ2 + θ3 + θ4)
        j35 = -0.05*sin(θ5)*sin(θ2 + θ3 + θ4)*cos(θ6) - 0.6043*sin(θ2 + θ3 + θ4)*cos(θ5)
        j36 = -0.05*sin(θ6)*sin(θ2 + θ3 + θ4)*cos(θ5) + 0.05*cos(θ6)*cos(θ2 + θ3 + θ4)

                

        J = matrix([[j11, j12, j13, j14, j15, j16],
                    [j21, j22, j23, j24, j25, j26],
                    [j31, j32, j33, j34, j35, j36]])

        # Inversa / pseudoinversa (dependiendo del robot (n) y el problema/tarea (m))
        Jinv = J.getI()

        # Valores deseados de posición
        xd = -0.026461
        yd = -1.485464
        zd = 0.441526

        Xd = array([[xd], [yd], [zd]])

        # Valores deseados de velocidad
        dxd = 0
        dyd = 0
        dzd = 0
        
        dXd = array([[dxd], [dyd], [dzd]])

        #Ganancias de control
        kx = 1
        ky = 2
        kz = 3

        K = matrix([[kx, 0, 0],
                    [0, ky, 0],
                    [0, 0, kz]])
        
        dq = Jinv @ (dXd - K@(X - Xd))
        
        return dq  
    
    """
    Función que resuelve por Euler mejorado las ecuaciones diferenciales
    """
    def euler(self, ts, θs, h):
        q = zeros((len(ts), 6))
    
        q[0, :] = θs

        for i in range(len(ts) - 1):

            dq = self.f(ts[i], q[i, :])
            
            #Flatten para evitar trasponer la matriz
            dq = array(dq).flatten()

            q_ast = q[i, :] + h * dq

            dq_final = self.f(ts[i], q_ast)

            dq_final = array(dq_final).flatten()

            q[i+1, :] = q[i, :] + (h/2) * (dq + dq_final)

        t = array(ts)

        return t, q
    
    """
    Función que obtiene la cinemática directa del UR30 mediante el método de DH y transformación de matrices

    """

    def CinematicaDirecta(self):
         T = eye(4)

         #Aplica la matriz de transformación obtenida con el método de DH
         for i in range(6):
            Aj = array([[cos(self.θ[i]), -sin(self.θ[i])*cos(self.__α[i]), sin(self.θ[i])*sin(self.__α[i]), self.__a[i]*cos(self.θ[i])],
                        [sin(self.θ[i]), cos(self.θ[i])*cos(self.__α[i]), -cos(self.θ[i])*sin(self.__α[i]), self.__a[i]*sin(self.θ[i])],
                        [0,              sin(self.__α[i]),                 cos(self.__α[i]),                self.__d[i]],
                        [0,              0,                                0,                               1]])
            T @= Aj
         
         #Se aplica la transformación de matrices
         T @= self.__tool 

         return T 

    """
    Función que obtiene la posición en x, y y z del Efector Final del robot

    """ 

    def Posicion(self):
        return self.T[:3, 3] * 1000
    

    """
    Función que obtiene la orientación en eje-ángulo (estandar de Universal Robotics) del robot.
    
    """
    
    def Orientacion(self):
        #Se obtiene la matriz de rotación del efector final con respecto a la base
        mat_rot = self.T[:3, :3]

        θ = acos((matrix.trace(mat_rot) - 1) / 2 )

        #u = (1 / (2 sin theta)) * [ R32 - R23; R13 - R31; R21 - R12 ]
        A = 1/(2*sin(θ))
        B = [mat_rot[2, 1] - mat_rot[1, 2],
             mat_rot[0, 2] - mat_rot[2, 0],
             mat_rot[1, 0] - mat_rot[0, 1]]
        
        u = dot(A, B)

        r_ur = u*θ
        r_ur_deg = rad2deg(r_ur)

        return r_ur_deg

    """
    Función que lleva al robot UR30 a la posición deseada para comenzar con el proceso de pintado.
    Esta función utiliza control cinemático.
    """
    def posicionarRobot(self, θs):
        ti = 0 
        h = 0.01
        tf = 30
        ts = arange(ti, tf+h, h)

        _,q = self.euler(ts, θs, h)

        # Desempaqueto solución
        θ1 = q[:,0] 
        θ2 = q[:,1]
        θ3 = q[:,2] 
        θ4 = q[:,3]
        θ5 = q[:,4] 
        θ6 = q[:,5]

        θ1_deg = rad2deg(θ1[-1])
        θ2_deg = rad2deg(θ2[-1])
        θ3_deg = rad2deg(θ3[-1])
        θ4_deg = rad2deg(θ4[-1])
        θ5_deg = rad2deg(θ5[-1])
        θ6_deg = rad2deg(θ6[-1])

        if θ1_deg > 360:
            θ1_deg -= 360
        elif θ1_deg < -360:
            θ1_deg += 360
        else:
            print("Forma parte del rango")
        
        if θ2_deg > 360:
            θ2_deg -= 360
        elif θ2_deg < -360:
            θ2_deg += 360
        else:
            print("Forma parte del rango")
        
        if θ3_deg > 360:
            θ3_deg -= 360
        elif θ3_deg < -360:
            θ3_deg += 360
        else:
            print("Forma parte del rango")

        if θ4_deg > 360:
            θ4_deg -= 360
        elif θ4_deg < -360:
            θ4_deg += 360
        else:
            print("Forma parte del rango")

        if θ5_deg > 360:
            θ5_deg -= 360
        elif θ5_deg < -360:
            θ5_deg += 360
        else:
            print("Forma parte del rango")

        if θ6_deg > 360:
            θ6_deg -= 360
        elif θ6_deg < -360:
            θ6_deg += 360
        else:
            print("Forma parte del rango")

        return [θ1_deg, θ2_deg, θ3_deg, θ4_deg, θ5_deg, θ6_deg]