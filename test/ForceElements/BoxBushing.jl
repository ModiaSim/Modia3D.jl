module BoxBushingSimulation

using Modia3D

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

BoxBushing = Model(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]), nominalLength=:Length)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    box = Object3D(feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    joint = FreeMotion(obj1=:world, obj2=:box, r=Var(init=[0.2, 0.1, 0.05]), rot=Var(init=startAngles)),
    force = Bushing(obj1=:world, obj2=:box,
                    springForceLaw=[fc, 100.0, 200.0], damperForceLaw=[1.0, fd, 4.0],
                    rotSpringForceLaw=[5.0, 10.0, mc], rotDamperForceLaw=[0.1, md, 0.4], largeAngles=largeAngles)
)

boxBushing = @instantiateModel(buildModia3D(BoxBushing), aliasReduction=false, unitless=true)

stopTime = 5.0
dtmax = 0.1
if largeAngles
    requiredFinalStates = [-0.013214812736859366, 0.0005523898753356359, -0.04904666002334838, 0.0757946142513073, 0.003341416782112683, -4.954082753506823e-5, -0.056708142772310295, 0.0009597147602342456, 4.340509280339362e-5, 0.3809257838547459, 0.001097814904879805, -0.00018842580793933223]
else
    requiredFinalStates = [-0.01321449035293881, 0.0005522522622707078, -0.04904666423238891, 0.07579225811468479, 0.0033409499586830368, -4.943166272627969e-5, -0.008049782190986742, 0.00034916853581158153, 1.460039207707777e-6, 0.04510769348383226, 0.0018233644599616554, 9.80056841368889e-6]
end
simulate!(boxBushing, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(boxBushing, ["joint.r", "joint.v", "joint.rot", "joint.w"], figure=1)

end
