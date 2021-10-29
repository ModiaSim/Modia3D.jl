module BoxSpringDamperPtP

using ModiaLang

import Modia3D
using  Modia3D.ModiaInterface

SpringDamper = Model(
    Length = 0.1,
    Mass = 1.0,
    Stiffness = 100.0,
    Damping = 2.0,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]), nominalLength=:Length)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    box = Object3D(feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=1.0, Ixx=0.1, Iyy=0.1, Izz=0.1),
                                 visualMaterial=:(visualMaterial))),
    joint = FreeMotion(obj1=:world, obj2=:box, r=Var(init=[0.1, 0.2, 0.3])),
    force = SpringDamperPtP(obj1=:world, obj2=:box, stiffness=:Stiffness, damping=:Damping)
)

springDamper = @instantiateModel(buildModia3D(SpringDamper), aliasReduction=false, unitless=true)

stopTime = 6.0
dtmax = 0.1
requiredFinalStates = [-0.026549009773552276, -0.05309801954749593, -0.10117097852565102, 0.0144068779028959, 0.028813755813570235, 0.02595802582858583, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
simulate!(springDamper, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(springDamper, ["joint.r", "joint.v"], figure=1)

end
