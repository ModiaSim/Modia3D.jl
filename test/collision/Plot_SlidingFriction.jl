module Plot_SlidingFriction

using PyCall
using PyPlot
pyplot_rc = PyCall.PyDict(PyPlot.matplotlib."rcParams")
pyplot_rc["font.size"] = 10.0

vsmall = 0.01
k = -log(0.01)/vsmall
vrel = collect(range(0,2.5*vsmall,length=100))
fr1 = zeros( length(vrel) )
fr2 = zeros( length(vrel) )

for i in 1:length(vrel)
   fr1[i] = vrel[i]/ (vrel[i] + vsmall*exp(-k*vrel[i]))
end

figure(1)
clf()
plot(vrel, fr1)
grid(true)
xlabel("\$|\\vec{v}_{rel,t}|\$")
legend(["\$ |\\vec{v}_{rel,t}| / \\left( |\\vec{v}_{rel,t}| + reg(|\\vec{v}_{rel,t}|, 0.01) \\right) \$"])

end