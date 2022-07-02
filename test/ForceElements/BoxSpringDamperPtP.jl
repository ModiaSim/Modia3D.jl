module BoxSpringDamperPtP

using Modia3D

SpringDamper = Model3D(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    Stiffness = 100.0,
    Damping = 2.0,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=:Length)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    box = Object3D(parent=:world, fixedToParent=false,
                   feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    boxCornerFrame = Object3D(parent=:box,
                              feature=Visual(shape=CoordinateSystem(length=:(Length/2))),
                              translation=:[Length/2, Length/2, Length/2]),
    force = SpringDamperPtP(obj1=:world, obj2=:boxCornerFrame, springForceLaw=:Stiffness, damperForceLaw=:Damping)
)

springDamper = @instantiateModel(SpringDamper, unitless=true)

stopTime = 5.0
dtmax = 0.1
requiredFinalStates = [-0.0317899616936806, -0.031789987944650766, -0.13960438446373588, -0.22639392571053196, -0.22639355519359725, -0.09585537982617036, -0.04370174242082521, 0.04366049248771943, 0.0009545457607853683, 0.8435417114212301, -0.8435409897813606, 1.1525522669027586e-7]
simulate!(springDamper, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(springDamper, ["box.translation", "box.velocity", "box.rotation", "box.angularVelocity"], figure=1)

end
