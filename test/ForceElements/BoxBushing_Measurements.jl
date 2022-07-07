module BoxBushing_Measurements

using Modia3D
using Modia3D.Measurements

const largeAngles = true
if largeAngles
    startAngles = [0.8, 0.4, 0.2]
else
    startAngles = [0.12, 0.06, 0.03]
end
fc(p) = 50.0 * p
fd(v) = 2.0 * v
mc(a) = 20.0 * a
md(w) = 0.2 * w

BoxBushing = Model3D(
    Length = 0.1,
    Mass = 1.0 ± 0.5,
    IMoment = 0.1 ± 0.05,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=:Length,
                                   enableContactDetection=false)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    box = Object3D(parent=:world, fixedToParent=false,
                   translation=[0.2, 0.1, 0.05],
                   rotation=startAngles,
                   feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    force = Bushing(obj1=:world, obj2=:box,
                    springForceLaw=[fc, 100.0, 200.0], damperForceLaw=[1.0, fd, 4.0],
                    rotSpringForceLaw=[5.0, 10.0, mc], rotDamperForceLaw=[0.1, md, 0.4], largeAngles=largeAngles)
)

boxBushing = @instantiateModel(BoxBushing, unitless=true, FloatType=Measurements.Measurement{Float64})

stopTime = 5.0
dtmax = 0.1
if largeAngles
    requiredFinalStates = [-0.013215277601760956 ± 0.10238262257381348, 0.0005526103273748118 ± 0.0034290441668571774, -0.04904665489506527 ± 0.024454889113053886, 0.07579851089880853 ± 0.6989650278803374, 0.0033419359041493876 ± 0.08002469080591593, -4.9715423211640314e-5 ± 0.00044925302816920573, -0.0567101057097755 ± 0.5086539247889504, 0.0009601261505964376 ± 0.00017381365017344816, 4.346670031167073e-5 ± 0.000775684386519148, 0.38094447328538217 ± 2.899746834182109, 0.0010797172375852052 ± 0.1194083233689792, -0.0006171237993113059 ± 9.719831767196763e-5]
else
    requiredFinalStates = [-0.013215277601552635 ± 0.10238262252917202, 0.0005526103197354207 ± 0.0034290442242695743, -0.049046654893074346 ± 0.02445488915038268, 0.0757985108632449 ± 0.6989650278303001, 0.003341935959151564 ± 0.08002468988356412, -4.971539341264976e-5 ± 0.0004492536593590073, -0.008050262232562722 ± 0.061052026485471915, 0.000349394588935699 ± 0.0018088785905316534, 1.46263089343343e-6 ± 1.0759148476334787e-5, 0.04511131507783584 ± 0.42767241422730234, 0.0018238149177386674 ± 0.04960105784995856, -2.073769187790423e-5 ± 0.0004842532652131016]
end

simulate!(boxBushing, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(boxBushing, ["box.translation", "box.velocity", "box.rotation", "box.angularVelocity"], figure=1)

end
