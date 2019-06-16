module Plot_SlidingFriction

#=




returns: v_small     , if absv=0
         0.01*v_small, if absv=v_small
         0           , if absv=infinity
=#

# regularize(absv,v_small) = v_small*exp(log(0.01)*(absv/v_small))

regularize(absv,v_small) = absv >= v_small ? absv : absv*(absv/v_small)*(1.0 - (absv/v_small)/3.0) + v_small/3.0


function resultantCor(cor1,cor2,abs_v_rel_n,v_small)
    @assert(cor1 >= 0.0 && cor1 <= 1.0)
    @assert(cor2 >= 0.0 && cor2 <= 1.0)
    @assert(abs_v_rel_n >= 0.0)
    @assert(v_small > 0)
 
    cor_min  = 0.001
    cor_mean = max(cor_min, (cor1 + cor2)/2.0)
    cor_res  = cor_mean + (cor_min - cor_mean)*exp(log(0.01)*(abs_v_rel_n/v_small))
    return cor_res
end

function resultantDampingCoefficient(cor1, cor2, abs_v_rel_n,v_small)
    @assert(cor1 >= 0.0 && cor1 <= 1.0)
    @assert(cor2 >= 0.0 && cor2 <= 1.0)
    @assert(abs_v_rel_n >= 0.0)
    @assert(v_small > 0)

    cof   = resultantCor(cor1,cor2,abs_v_rel_n,v_small)
    d_res = 8.0*(1.0 - cof)/(5*cof*regularize(abs_v_rel_n,v_small))
    return d_res
end

using PyCall
using PyPlot
pyplot_rc = PyCall.PyDict(PyPlot.matplotlib."rcParams")
pyplot_rc["font.size"] = 10.0

vsmall = 0.01
cof0 = 1.0
cor1 = 0.7
cor2 = 0.3
cof3 = 0.1
cof4 = 0.01

vrela = collect(range(0.5*vsmall,4*vsmall,length=100))
vrelb = collect(range(0         ,  vsmall,length=500))
vrelc = collect(range(0         ,4*vsmall,length=100))
vreld = collect(range(0         ,2*vsmall,length=100))

d_res0a = zeros( length(vrela) )
d_res1a = zeros( length(vrela) )
d_res2a = zeros( length(vrela) )
d_res3a = zeros( length(vrela) )
d_res4a = zeros( length(vrela) )

d_res0b = zeros( length(vrelb) )
d_res1b = zeros( length(vrelb) )
d_res2b = zeros( length(vrelb) )
d_res3b = zeros( length(vrelb) )
d_res4b = zeros( length(vrelb) )

cor_res0 = zeros( length(vrelc) )
cor_res1 = zeros( length(vrelc) )
cor_res2 = zeros( length(vrelc) )
cor_res3 = zeros( length(vrelc) )
cor_res4 = zeros( length(vrelc) )

reg = zeros( length(vreld) )

for i in 1:length(vrela)
   d_res0a[i] = resultantDampingCoefficient(cof0,cof0,vrela[i],vsmall)
   d_res1a[i] = resultantDampingCoefficient(cor1,cor1,vrela[i],vsmall)
   d_res2a[i] = resultantDampingCoefficient(cor2,cor2,vrela[i],vsmall)
   d_res3a[i] = resultantDampingCoefficient(cof3,cof3,vrela[i],vsmall)
   d_res4a[i] = resultantDampingCoefficient(cof4,cof4,vrela[i],vsmall)
end

for i in 1:length(vrelb)
   d_res0b[i] = resultantDampingCoefficient(cof0,cof0,vrelb[i],vsmall)
   d_res1b[i] = resultantDampingCoefficient(cor1,cor1,vrelb[i],vsmall)
   d_res2b[i] = resultantDampingCoefficient(cor2,cor2,vrelb[i],vsmall)
   d_res3b[i] = resultantDampingCoefficient(cof3,cof3,vrelb[i],vsmall)
   d_res4b[i] = resultantDampingCoefficient(cof4,cof4,vrelb[i],vsmall)
end

for i in 1:length(vrelc)
   cor_res0[i] = resultantCor(cof0,cof0,vrelc[i],vsmall)
   cor_res1[i] = resultantCor(cor1,cor1,vrelc[i],vsmall)
   cor_res2[i] = resultantCor(cor2,cor2,vrelc[i],vsmall)
   cor_res3[i] = resultantCor(cof3,cof3,vrelc[i],vsmall)
   cor_res4[i] = resultantCor(cof4,cof4,vrelc[i],vsmall)

end

for i in 1:length(vreld)
   reg[i] = regularize(vreld[i],vsmall)
end

figure(1)
clf()
plot(vrela, d_res0a, vrela, d_res1a, vrela, d_res2a, vrela, d_res3a)
grid(true)
xlabel("\$|\\delta^-|\$")
ylabel("\$d_{res}\$")
legend(["\$cor_1 = cor_2 = 1.0, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.7, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.3, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.1, v_{small}=0.01\$"])

figure(2)
clf()
plot(vrelb, d_res0b, vrelb, d_res1b, vrelb, d_res2b, vrelb, d_res3b, vrelb, d_res4b)
grid(true)
xlabel("\$|\\delta^-|\$")
ylabel("\$d_{res}\$")
legend(["\$cor_1 = cor_2 = 1.0, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.7, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.3, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.1, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.01, v_{small}=0.01\$"])

figure(3)
clf()
plot(vrelc, cor_res0, vrelc, cor_res1, vrelc, cor_res2, vrelc, cor_res3, vrelc, cor_res4)
grid(true)
xlabel("\$|\\delta^-|\$")
ylabel("\$cor_{res}\$")
legend(["\$cor_1 = cor_2 = 1.0, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.7, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.3, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.1, v_{small}=0.01\$",
        "\$cor_1 = cor_2 = 0.0, v_{small}=0.01\$"],loc="upper right")


figure(4)
clf()
plot(vreld, reg)
grid(true)
xlabel("\$v_{abs}\$")
legend(["\$reg(v_{abs},0.1)\$"])
end