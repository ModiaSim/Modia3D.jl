module FreeShaftAdaptiveRotSequence

using Modia3D
using Modia3D.Unitful

startRotation = [30.0, 90.0, 10.0]u"°"

Shaft = Model3D(
    Length = 1.0,
    Diameter = 0.2,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=0.3, n=[0, 0, -1]))),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Diameter))),
    shaft = Object3D(parent=:world, fixedToParent=false,
                     rotation=startRotation,
                     velocity=[0.0, 0.1, 0.0]u"m/s",
                     angularVelocity=Modia3D.resolve1(startRotation, [-2.0, 0.0, 0.0]u"rad/s"),
                     feature=Solid(shape=Cylinder(axis=3, diameter=:Diameter, length=:Length),
                                   massProperties=MassProperties(; mass=84.7154, Ixx=7.2711, Iyy=7.2711, Izz=0.4230),
                                   visualMaterial=:(visualMaterial))),
    shaftFrame = Object3D(parent=:shaft, feature=Visual(shape=CoordinateSystem(length=0.4))),
)

shaft = @instantiateModel(Shaft, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 7.0
dtmax = 0.1
requiredFinalStates = [0.0, 0.7, -7.350074420637136, 0.0, 0.1, -2.1, 0.6981317007977381, 1.4336293856408397, 1.5707963267949017, 0.0, -1.2855752193730614, 1.5320888862379711]
simulate!(shaft, stopTime=stopTime, dtmax=dtmax, log=true, logEvents=true, logStates=false, requiredFinalStates=requiredFinalStates)
showInfo(shaft)

@usingModiaPlot
plot(shaft, ["shaft.rotation", "shaft.rotationXYZ", "shaft.angularVelocity", "shaft.translation", "shaft.velocity"], figure=1)

end
