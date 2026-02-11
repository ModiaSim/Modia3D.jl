module FreeShaftAdaptiveRotSequenceWithFreeMotion

using Modia3D

Shaft = Model3D(
    Length = 1.0,
    Diameter = 0.2,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=0.3, n=[0, 0, -1]))),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Diameter))),
    shaft = Object3D(feature=Solid(shape=Cylinder(axis=3, diameter=:Diameter, length=:Length),
                                   massProperties=MassProperties(; mass=84.7154, Ixx=7.2711, Iyy=7.2711, Izz=0.4230),
                                   visualMaterial=:(visualMaterial))),
    shaftFrame = Object3D(parent=:shaft, feature=Visual(shape=CoordinateSystem(length=0.4))),
    free = FreeMotion(obj1=:world, obj2=:shaft, v   = Var(init  = Modia.SVector{3,Float64}(0.0, 0.1, 0.0)),
                                                rot = Var(start = Modia.SVector{3,Float64}(deg2rad(30), deg2rad(90), deg2rad(10))),
                                                w   = Var(init  = Modia.SVector{3,Float64}(-2.0, 0.0, 0.0)))
)

#@showModel model

shaft = @instantiateModel(Shaft, unitless=true, log=false, logStateSelection=false, logCode=false)

#@showModel shaft.p[1]

stopTime = 7.0
dtmax = 0.1
requiredFinalStates = [0.0, 0.7, -7.350074420637136, 0.0, 0.1, -2.1, 0.6981317007977381, 1.4336293856408397, 1.5707963267949017, -2.0, 0.0, 0.0]
simulate!(shaft, stopTime=stopTime, dtmax=dtmax, log=true, logEvents=false, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(shaft, ["free.rot", "free.isrot123", "free.w", "free.r", "free.v"], figure=1)

simulate!(shaft, stopTime=stopTime, dtmax=dtmax, log=true, logEvents=false, logStates=false, requiredFinalStates=requiredFinalStates)

end
