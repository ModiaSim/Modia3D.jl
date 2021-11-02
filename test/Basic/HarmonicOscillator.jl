module HarmonicOscillator

using ModiaLang

import Modia3D
using  Modia3D.ModiaInterface

Oscillator = Model(
    Length = 0.1,
    Mass = 1.0,
    Stiffness = 100.0,
    Damping = 2.0,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]), nominalLength=:Length)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    oscillator = Object3D(feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                        massProperties=MassProperties(; mass=1.0, Ixx=0.1, Iyy=0.1, Izz=0.1),
                                        visualMaterial=:(visualMaterial))),
    joint = Prismatic(obj1=:world, obj2=:oscillator, axis=3, s=Var(init=0.0), v=Var(init=0.0)),
    force = Bushing(obj1=:world, obj2=:oscillator, stiffness=:[0.0, 0.0, Stiffness], damping=:[0.0, 0.0, Damping])
)

oscillator = @instantiateModel(buildModia3D(Oscillator), aliasReduction=false, unitless=true)

stopTime = 5.0
dtmax = 0.1
requiredFinalStates = [0.0032763240987720987, -0.09755848695453585]
simulate!(oscillator, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(oscillator, ["joint.s", "joint.v"], figure=1)

end
