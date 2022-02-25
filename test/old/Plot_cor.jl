module Plot_cor

import Modia3D
import ModiaPlot_PyPlot: PyCall, PyPlot

pyplot_rc = PyCall.PyDict(PyPlot.matplotlib."rcParams")
pyplot_rc["font.size"] = 10.0

vsmall = 0.01
cor0 = 1.0
cor1 = 0.7
cor2 = 0.3
cor3 = 0.1
cor4 = 0.01

vrela = collect(range(0.5*vsmall, 4*vsmall, length=1000))
vrelb = collect(range(0         ,   vsmall, length=5000))
vrelc = collect(range(0         , 4*vsmall, length=1000))
vreld = collect(range(0         , 2*vsmall, length=1000))

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
w   = 0.0
for i in 1:length(vrela)
    d_res0a[i] = Modia3D.resultantDampingCoefficient(cor0,vrela[i],vsmall,2000)
    d_res1a[i] = Modia3D.resultantDampingCoefficient(cor1,vrela[i],vsmall,2000)
    d_res2a[i] = Modia3D.resultantDampingCoefficient(cor2,vrela[i],vsmall,2000)
    d_res3a[i] = Modia3D.resultantDampingCoefficient(cor3,vrela[i],vsmall,2000)
    d_res4a[i] = Modia3D.resultantDampingCoefficient(cor4,vrela[i],vsmall,2000)
end

for i in 1:length(vrelb)
    d_res0b[i] = Modia3D.resultantDampingCoefficient(cor0,vrelb[i],vsmall,2000)
    d_res1b[i] = Modia3D.resultantDampingCoefficient(cor1,vrelb[i],vsmall,2000)
    d_res2b[i] = Modia3D.resultantDampingCoefficient(cor2,vrelb[i],vsmall,2000)
    d_res3b[i] = Modia3D.resultantDampingCoefficient(cor3,vrelb[i],vsmall,2000)
    d_res4b[i] = Modia3D.resultantDampingCoefficient(cor4,vrelb[i],vsmall,2000)
end

resultantCoefficientOfRestitution(cor, abs_vreln, vsmall) = abs_vreln > vsmall ? cor : 0.0

for i in 1:length(vrelc)
    cor_res0[i] = resultantCoefficientOfRestitution(cor0,vrelc[i],vsmall)
    cor_res1[i] = resultantCoefficientOfRestitution(cor1,vrelc[i],vsmall)
    cor_res2[i] = resultantCoefficientOfRestitution(cor2,vrelc[i],vsmall)
    cor_res3[i] = resultantCoefficientOfRestitution(cor3,vrelc[i],vsmall)
    cor_res4[i] = resultantCoefficientOfRestitution(cor4,vrelc[i],vsmall)
end

for i in 1:length(vreld)
    reg[i] = Modia3D.regularize(vreld[i],vsmall)
end

PyPlot.figure(1)
PyPlot.clf()
PyPlot.plot(vrelc, cor_res0, vrelc, cor_res1, vrelc, cor_res2, vrelc, cor_res3, vrelc, cor_res4)
PyPlot.grid(true)
PyPlot.xlabel("\$\\dot{\\delta}^- \\; [m/s]\$")
PyPlot.ylabel("\$cor_{reg}\$")
PyPlot.legend(["\$cor = 1.0, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.7, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.3, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.1, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.0, v_{small}=0.01 \\; m/s\$"],loc="upper right")

PyPlot.figure(2)
PyPlot.clf()
PyPlot.plot(vrela, d_res0a, vrela, d_res1a, vrela, d_res2a, vrela, d_res3a, vrela, d_res4a)
PyPlot.grid(true)
PyPlot.xlabel("\$\\dot{\\delta}^- \\; [m/s]\$")
PyPlot.ylabel("\$d \\; [Ns/m]\$")
PyPlot.legend(["\$cor = 1.0, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.7, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.3, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.1, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.0, v_{small}=0.01 \\; m/s\$"],loc="upper right")

PyPlot.figure(3)
PyPlot.clf()
PyPlot.plot(vrelb, d_res0b, vrelb, d_res1b, vrelb, d_res2b, vrelb, d_res3b, vrelb, d_res4b)
PyPlot.grid(true)
PyPlot.xlabel("\$\\dot{\\delta}^- \\; [m/s]\$")
PyPlot.ylabel("\$d \\; [Ns/m]\$")
PyPlot.legend(["\$cor = 1.0, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.7, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.3, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.1, v_{small}=0.01 \\; m/s\$",
               "\$cor = 0.0, v_{small}=0.01 \\; m/s\$"],loc="upper right")

PyPlot.figure(4)
PyPlot.clf()
PyPlot.plot(vreld, reg)
PyPlot.grid(true)
PyPlot.xlabel("\$v_{abs}\$")
PyPlot.legend(["\$reg(v_{abs},0.1)\$"])

println("... test/collision/Plot_cor.jl completed.")

end
