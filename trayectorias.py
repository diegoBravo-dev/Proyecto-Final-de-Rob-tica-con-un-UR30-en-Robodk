from numpy import array, zeros, arange
from numpy.linalg import solve

def trazarTrayectoria(θ0, θf):
    ω0 = zeros(6)
    α0 = zeros(6)
    ωf = zeros(6)
    αf = zeros(6)

    t0 = 0
    tf = 10
    h = 0.5

    A = array([[t0**5   , t0**4   , t0**3  , t0**2, t0, 1],
               [5*t0**4 , 4*t0**3 , 3*t0**2, 2*t0 , 1 , 0],
               [20*t0**3, 12*t0**2, 6*t0   , 2    , 0 , 0],
               [tf**5   , tf**4   , tf**3  , tf**2, tf, 1],
               [5*tf**4 , 4*tf**3 , 3*tf**2, 2*tf , 1 , 0],
               [20*tf**3, 12*tf**2, 6*tf   , 2    , 0 , 0]])
    
    t = arange(t0, tf+h, h)

    θ_all = []
    ω_all = []
    α_all = []

    for i in range(6):
        B = array([
            θ0[i], ω0[i], α0[i],
            θf[i], ωf[i], αf[i],
        ])

        X = solve(A, B)
        a5, a4, a3, a2, a1, a0_ = X

        q = a5*t**5 + a4*t**4 + a3*t**3 + a2*t**2 + a1*t + a0_
        w = 5*a5*t**4 + 4*a4*t**3 + 3*a3*t**2 + 2*a2*t + a1
        acc = 20*a5*t**3 + 12*a4*t**2 + 6*a3*t + 2*a2

        θ_all.append(q)
        ω_all.append(w)
        α_all.append(acc)

    return t, array(θ_all), array(ω_all), array(α_all)