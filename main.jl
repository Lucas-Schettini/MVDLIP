using JuMP, CPLEX
using JSON3
using Combinatorics

data = JSON3.read("dummy_K2.json")

D = Int.(data.sets.D)
V = Int.(data.sets.V)
Vr = Int.(data.sets.VR)

VrD = union(Vr, D) #vértices requeridos + os pontos de parada

all_pairs = Set(Tuple(p) for p in combinations(VrD, 2))
Er_set = Set(Tuple(sort([e.u, e.v])) for e in data.required_edges)
Enr = collect(setdiff(all_pairs, Er_set))

coords = Dict(parse(Int, string(k)) => (v.x, v.y) for (k, v) in pairs(data.coordinates))
euclid(u, v) = sqrt((coords[u][1] - coords[v][1])^2 + (coords[u][2] - coords[v][2])^2)
dnr = Dict(e => euclid(e[1], e[2]) for e in Enr)

Er = [(e.u, e.v) for e in data.required_edges]
dr = Dict((e.u, e.v) => e.de for e in data.required_edges) #fazer d(uv) = d(vu)?

E = union(Er, Enr)

Ap = [(garc.i, garc.j) for garc in data.Ap]
dc = Dict((garc.i, garc.j) => garc.dij for garc in data.Ap) #fazer dij = dji?

T = [1,2]
K = [1,2]
kappa = [10 for d in D]

model = Model(CPLEX.Optimizer)

AE = vcat(E, [(j,i) for (i,j) in E]) # E no sentido (i,j) e (j,i)
AEnr = vcat(Enr, [(j,i) for (i,j) in Enr]) # Enr nos dois sentidos
AEr  = vcat(Er, [(j,i) for (i,j) in Er]) # Er nos dois sentidos

@variable(model, w[(i,j) in Ap, t in T], Bin)
@variable(model, z[d in D, t in T], Bin)
@variable(model, g[(i,j) in Ap, t in T] >= 0, Int)
@variable(model, x[e in AE, t in T, d in D, k in K], Bin)
@variable(model, y[e in AEnr, t in T, d in D, k in K], Bin)
#@variable(model, a[(i,j) in AE, t in T, d in D, k in K], Bin)
#@variable(model, b[(i,j) in AEnr, t in T, d in D, k in K], Bin)
#@variable(model, p[v in V, t in T, d in D, k in K], Int)
#@variable(model, u[v in V, t in T, d in D, k in K], Bin) VERIFICAR SE É DE FATO NECESSÁRIO
@variable(model, f[(i,j) in AE, t in T, d in D, k in K] >= 0)
@variable(model, m >= 0)

@objective(model, Min, m)

#Restrição de Min Max
vs = 4
vc = 14
vd = 6 # Maior do que a velocidade de serviço

ts = Dict()

for a in AEr
    if haskey(dr, a)
        valor = dr[a]
    else
        valor = dr[(a[2], a[1])]
    end

    ts[a] = valor / vs
end
tvoo = Dict(a => euclid(a[1], a[2]) / vd for a in AEnr)
t_terra = Dict((i,j) => dc[(i,j)]/vc for (i,j) in Ap)

@constraint(model, con2[t in T],
    sum(t_terra[(i,j)] * w[(i,j),t] for (i,j) in Ap) +
    # sum(kappa[d] * z[d,t] for d in D) #+
    sum(
        sum(ts[e] * x[e,t,d,k] for e in AEr) +
        sum(tvoo[e] * (x[e,t,d,k] + y[e,t,d,k]) for e in AEnr)
        for d in D, k in K
    ) 
    <= m
) # modelo, nome, expressão 

# Roteamento terrestre
@constraint(model, con3[t in T], 
      sum(w[(0,d),t] for d in D) <= 1)

D0 = union([0], D)

@constraint(model, con4[d in D, t in T],
    sum(w[(d,j),t] for j in D0 if j != d) == z[d, t]) 

@constraint(model, con5[d in D, t in T],
    sum(w[(j,d),t] for j in D0 if j != d) == z[d, t]) 

@constraint(model, con6[d in D], 
    sum(z[d,t] for t in T) <= 1)

@constraint(model, con7[t in T],
    sum(g[(0,d),t] for d in D) ==
    sum(z[d,t] for d in D))

@constraint(model, con8[d in D, t in T],
    sum(g[(j,d),t] for j in D0 if j != d) - 
    sum(g[(d,j),t] for j in D0 if j != d) == z[d,t])

@constraint(model, con9[(i,j) in Ap, t in T],
    g[(i,j),t] <= length(D)*w[(i,j),t])

# #Roteamento aéreo

delta_plus     = Dict(v => [(i,j) for (i,j) in AE   if i == v] for v in VrD)  
delta_minus    = Dict(v => [(i,j) for (i,j) in AE   if j == v] for v in VrD)  
delta_plus_nr  = Dict(v => [(i,j) for (i,j) in AEnr if i == v] for v in VrD)
delta_minus_nr = Dict(v => [(i,j) for (i,j) in AEnr if j == v] for v in VrD)

@constraint(model, con10[v in VrD, t in T, d in D, k in K],
    sum(x[a,t,d,k] for a in delta_plus[v])  + sum(y[a,t,d,k] for a in delta_plus_nr[v]) ==
    sum(x[a,t,d,k] for a in delta_minus[v]) + sum(y[a,t,d,k] for a in delta_minus_nr[v]))

@constraint(model, con11[(v,j) in AE, t in T, d in D, k in K; v != d],
    sum(f[a,t,d,k] for a in delta_plus[v]) - sum(f[a,t,d,k] for a in delta_minus[v]) >= x[(v,j),t,d,k])

@constraint(model, con12x[(i,j) in AEr, t in T, d in D, k in K],
    f[(i,j),t,d,k] <= (length(VrD) - 1) * x[(i,j),t,d,k])
@constraint(model, con12y[(i,j) in AEnr, t in T, d in D, k in K],
    f[(i,j),t,d,k] <= (length(VrD) - 1) * (x[(i,j),t,d,k] + y[(i,j),t,d,k]))

@constraint(model, con13[(i,j) in AEr], 
    sum((x[(i,j),t,d,k] + x[(j,i),t,d,k]) for t in T for d in D for k in K) >= 1)

@constraint(model, con14[(i,j) in AE, t in T, d in D, k in K],
    x[(i,j),t,d,k] + x[(j,i),t,d,k] <= 1)

@constraint(model, con15[(i,j) in AEnr, t in T, d in D, k in K],
    y[(i,j),t,d,k] + y[(j,i),t,d,k] <= 1)    

@constraint(model, con16[(i,j) in AEnr, d in D, k in K, t in T], 
    x[(i,j),t,d,k] + x[(j,i),t,d,k] >= y[(i,j),t,d,k] + y[(j,i),t,d,k])

L = 4

@constraint(model, con17[d in D, k in K, t in T],
    sum(ts[e]*x[e,t,d,k] for e in AEr) + 
    sum(tvoo[e]*(x[e,t,d,k] + y[e,t,d,k]) for e in AEnr) <= 
    L*z[d,t])

@constraint(model, con18[e in AE, d in D, k in K, t in T],
    x[e,t,d,k] <= z[d,t])

@constraint(model, con19[t in T, d in D, k in K; k < K[end]],
    sum(x[a,t,d,k]   for a in AE) >=
    sum(x[a,t,d,k+1] for a in AE))

#write_to_file(model, "modelo.lp")

optimize!(model)

println(termination_status(model))
println(primal_status(model))

for v in all_variables(model)
    val = value(v)
    if abs(val) > 1e-6
        println(v, " = ", val)
    end
end

thr = 0.5 

truck_arcs = [
    Dict("t" => t, "i" => i, "j" => j)
    for (i,j) in Ap, t in T if value(w[(i,j),t]) > thr
]

flight_arcs = []
for t in T, d in D, k in K
    for (i,j) in AE
        if value(x[(i,j),t,d,k]) > thr
            push!(flight_arcs, Dict("t"=>t,"d"=>d,"k"=>k,"i"=>i,"j"=>j,
                "kind" => ((i,j) in Er ? "service" : "deadhead"),
                "pass" => 1))
        end
    end
    for (i,j) in AEnr
        if value(y[(i,j),t,d,k]) > thr
            push!(flight_arcs, Dict("t"=>t,"d"=>d,"k"=>k,"i"=>i,"j"=>j,
                "kind" => "deadhead", "pass" => 2))
        end
    end
end

open("solucao.json", "w") do io
    JSON3.write(io, Dict("m" => value(m),
                         "truck_arcs" => truck_arcs,
                         "flight_arcs" => flight_arcs))
end