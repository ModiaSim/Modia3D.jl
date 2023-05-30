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

BoxBushing = Model3D(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=:(2*Length))),
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

boxBushing = @instantiateModel(BoxBushing, unitless=true, logCode=true)

stopTime = 5.0
dtmax = 0.1
if largeAngles
    requiredFinalStates = [-0.013214709281307713, 0.000552354087222612, -0.04904666187675468, 0.07579402127381728, 0.0033411984137335553, -4.9513118790869216e-5, -0.056707801161520716, 0.0009597189920446882, 4.333168563994031e-5, 0.380922690677635, 0.0010802088978311055, -0.000616198472040944]
else
    requiredFinalStates = [-0.013214485263238932, 0.0005522500941536969, -0.0490466643002863, 0.0757922207638451, 0.0033409424747158703, -4.942995177735604e-5, -0.00804978203315705, 0.0003491684885472317, 1.460136858508138e-6, 0.04510757535421865, 0.00182330830539622, -2.0621586293093137e-5]
end
simulate!(boxBushing, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(boxBushing, ["box.translation", "box.velocity", "box.rotation", "box.angularVelocity"], figure=1)
plot(boxBushing, ["force.translation", "force.velocity", "force.rotation", "force.rotationVelocity"], figure=2)
plot(boxBushing, ["force.springForce", "force.damperForce", "force.forceVector"], figure=3)
plot(boxBushing, ["force.springTorque", "force.damperTorque", "force.torque", "force.torqueVector"], figure=4)

end
