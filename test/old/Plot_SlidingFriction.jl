module Plot_SlidingFriction

using PyCall
using PyPlot
pyplot_rc = PyCall.PyDict(PyPlot.matplotlib."rcParams")
pyplot_rc["font.size"] = 10.0

regularize(absv,vsmall) = absv >= vsmall ? absv : absv*(absv/vsmall)*(1.0 - (absv/vsmall)/3.0) + vsmall/3.0
vsmall = 0.01
vrel = collect(range(0,2*vsmall,length=100))
fr1 = zeros( length(vrel) )

for i in 1:length(vrel)
    fr1[i] = vrel[i] / regularize(vrel[i], vsmall)
end

figure(1)
clf()
plot(vrel, fr1)
grid(true)
xlabel("\$|\\vec{v}_{rel,t}|\$")
legend(["\$ |\\vec{v}_{rel,t}| / reg( |\\vec{v}_{rel,t}|, 0.01) \$"])

println("... test/collision/Plot_SlidingFriction.jl completed.")

end
