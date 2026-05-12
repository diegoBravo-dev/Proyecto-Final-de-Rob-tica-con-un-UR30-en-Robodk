from sympy import acos, sin, eye, Matrix, cos, symbols, simplify, pi, diff

def DH_simbolica(θs, a, d, αs):
    T06 = eye(4)
    
    tool = Matrix([[0.866025, 0, 0.5,      0.050],
                   [0,        1, 0,        0],
                   [-0.5,     0, 0.866025, 0.450],
                   [0,        0, 0,        1]])
    
    for i in range(6):
        T = Matrix([[cos(θs[i]), -sin(θs[i])*cos(αs[i]), sin(θs[i])*sin(αs[i]), a[i]*cos(θs[i])],
                    [sin(θs[i]), cos(θs[i])*cos(αs[i]), -cos(θs[i])*sin(αs[i]), a[i]*sin(θs[i])],
                    [0,          sin(αs[i]),             cos(αs[i]),            d[i]],
                    [0,          0,                         0,                        1]])
        
        T06 = T06.multiply(T)
    
    T0_tool = T06.multiply(tool)
    
    return T0_tool

θ_1, θ_2, θ_3, θ_4, θ_5, θ_6 = symbols('θ1, θ2, θ3, θ4, θ5, θ6')
π = pi

θs = [θ_1, θ_2, θ_3, θ_4, θ_5, θ_6]
a = [0, -0.6370, -0.5037, 0, 0, 0]
d = [0.2363, 0, 0, 0.2010, 0.1593, 0.1543]
αs = [π/2, 0, 0, π/2, -π/2, 0]

T0_tool = DH_simbolica(θs, a, d, αs)

X = T0_tool.col(3)

px = simplify(X[0])
py = simplify(X[1])
pz = simplify(X[2])

#print(prz)