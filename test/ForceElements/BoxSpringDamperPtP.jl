module BoxSpringDamperPtP

using Modia3D

SpringDamper = Model3D(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    Stiffness = 100.0,
    Damping = 2.0,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]), nominalLength=:Length)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    box = Object3D(feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    boxCornerFrame = Object3D(parent=:box,
                              feature=Visual(shape=CoordinateSystem(length=:(Length/2))),
                              translation=:[Length/2, Length/2, Length/2]),
    joint = FreeMotion(obj1=:world, obj2=:box),
    force = SpringDamperPtP(obj1=:world, obj2=:boxCornerFrame, springForceLaw=:Stiffness, damperForceLaw=:Damping)
)

springDamper = @instantiateModel(SpringDamper, unitless=true)

stopTime = 5.0
dtmax = 0.1
requiredFinalStates = [-0.031789887795694936, -0.03179006302845493, -0.13960438757282567, -0.22639388796813553, -0.226393551737028, -0.0958553592848107, -0.04370189950475126, 0.0436602955184432, 0.0009569259867732385, 0.8435396531516037, -0.8435431841758767, 3.531021863184031e-6]
simulate!(springDamper, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(springDamper, ["joint.r", "joint.v", "joint.rot", "joint.w"], figure=1)

end
