module Plot_SlidingFriction

using PyCall
using PyPlot
pyplot_rc = PyCall.PyDict(PyPlot.matplotlib."rcParams")
pyplot_rc["font.size"] = 10.0

vt   = 2.0
vmin = 0.4
# a = atanh( (0.99 - 0.5)/0.5 ) / (vmin/2)
# println("a = $a")
# k = -log(2/(1+0.99)-1)/vmin
# 0.01*vmin = vmin*exp(-k*vmin) -> log(0.01) = -k*vmin -> k = -log(0.01)/vmin
k = -log(0.05)/vmin
vrel = collect(range(0,1,length=100))
fr1 = zeros( length(vrel) )
fr2 = zeros( length(vrel) )

for i in 1:length(vrel)
   fr1[i] = vrel[i]/ (vrel[i] + vmin*exp(-k*vrel[i]))
   # fr1[i] = vrel[i]/ (vrel[i] < vmin ? vmin/2*(1 + (vrel[i]/vmin)^2) : vrel[i])
   # fr1[i] = 2/(1+exp(-k*vrel[i])) - 1
   # fr2[i] = vrel[i]/(max(vrel[i],vmin))*fr1[i]
   # fr[i] = vt/(max(vt,vmin))*(2/(1+exp(-k*vrel[i])) - 1)
   # fr[i] = 0.5 + 0.5*tanh(a*(vrel[i] - vmin/2))
end

clf()
plot(vrel, fr1)
grid(true)
xlabel("\$|\\vec{v}_t| \\quad (v_{min} = 0.4; \\; k=-log(0.01)/v_{min})\$")
#legend(["\$\\frac{|\\vec{v}_t|}{\\max(|\\vec{v}_t|, v_{min}=0.2)} \\left( \\frac{2}{1+e^{-k |\\vec{v}_t|}} - 1 \\right) \$"])
# legend(["\$2/(1+e^{-k |\\vec{v}_t|}) - 1 \$"])
legend(["\$|\\vec{v}_t| / \\left( |\\vec{v}_t| + v_{min} e^{-k|\\vec{v}_t|} \\right)\$"])

end