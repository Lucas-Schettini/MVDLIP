using JuMP, HiGHS
using JSON3

data = JSON3.read("teste.json")

D = data.sets.D
V = data.sets.V
Vr = data.sets.VR
K = data.sets.K
Er = [(e.u, e.v) for e in data.required_edges]
de = Dict((e.u, e.v) => e.de for e in data.required_edges) #fazer d(uv) = d(vu)?
Ap = [(garc.i, garc.j) for garc in data.Ap]
d = Dict((garc.i, garc.j) => garc.dij for garc in data.Ap) #fazer dij = dji?
T = [1,2,3]

model = Model(HiGHS.Optimizer)

#DEFINIR ENR 

@variable(model, w[(i,j) in Ap, t in T], Bin)
@variable(model, z[d in D, t in T], Bin)
@variable(model, g[(i,j) in Ap, t in T] >= 0, Int)
@variable(model, x[e in E, t in T, d in D, k in K], Bin)
@variable(model, y[e in Enr, t in T, d in D, k in K], Bin)
@variable(model, m >= 0)

@objective(model, Min, m)

#Restrição de Min Max
vs = 4
vc = 14
vd = 6 # Maior do que a velocidade de serviço

# ts = [de[e]/vs for e in Er]
# tvoo = [de[e]/vd for e in Enr]
# t_terra = [dij[i][j]/vc ]

# @constraint(model, con2[t in T],
#     sum(t_terra[i,j] * w[i,j,t] for (i,j) in Ap) +
#     sum(kappa[d] * z[d,t] for d in D) +
#     sum(
#         sum(t_s[e] * x[e,t,d,k] for e in Er) +
#         sum(t_voo[e] * (x[e,t,d,k] + y[e,t,d,k]) for e in Enr)
#         for d in D, k in K
#     ) <= m
# ) # modelo, nome, expressão 

# #Roteamento terrestre
# @constraint(model, con3[t in T], 
#     sum(w[0,d,0] for d in D) >= 1)

# D0 = union([0], D)

# @constraint(model, con4[d in D, t in T],
#     sum(w[d,j,t] for j in D0 if j != d) == z[d, t]) 

# @constraint(model, con5[d in D, t in T],
#     sum(w[j,d,t] for j in D0 if j != d) == z[d, t]) 

# @constraint(model, con6[d in D], 
#     sum(z[d,t] for t in T) <= 1)

# @constraint(model, con7[t in T],
#     sum(g[0,d,t] for d in D) ==
#     sum(z[d,t] for d in D))

# @constraint(model, con8[d in D, t in T],
#     sum(g[j,d,t] for j in D0) - 
#     sum(g[d,j,t] for j in D0) == z[d,t])

# @constraint(model, con9[(i,j) in Ap, t in T],
#     g[i,j,t] <= length(D)*w[i,j,t])

# #Roteamento aéreo

# # @constraint(model, con10)
# # @constraint(model, con11)
# # Retrições para resolver o subtour

# @constraint(model, con12[e in Er], 
#     sum(x[e,t,d,k] for e in Er for t in T for d in D for k in K >= 1))

# @constraint(model, con13[e in Enr, d in D, k in K, t in T], 
#     x[e,t,d,k] >= y[e,t,d,k])

# @constraint(model, con14[d in D, k in K, t in T],
#     sum(ts[e]*x[e,t,d,k] for e in Er) + 
#     sum(tvoo[e]*(x[e,t,d,k] + y[e,t,d,k]) for e in Enr) <= 
#     L*z[d])

# @constraint(model, con15[e in E, d in D, k in K, t in T],
#     x[e,t,d,k] <= z[d])

# @constraint(model, con16[t in T, d in D, k in K])

# write_to_file(model, "modelo.lp")