module ShaftFreeMotion2

using ModiaLang

import Modia3D
using  Modia3D.ModiaInterface

Shaft = Model(
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
    free = FreeMotion(obj1=:world, obj2=:shaft, v   = Var(init = [0.0, 0.1, 0.0]),
                                                rot = Var(start = [deg2rad(30), deg2rad(90), deg2rad(10)]),
                                                w   = Var(init = [-2.0, 0.0, 0.0]))
)

#@showModel model

shaft = @instantiateModel(buildModia3D(Shaft), aliasReduction=false, unitless=true, log=false, logStateSelection=false, logCode=true)

#@showModel shaft.p[1]

stopTime = 1.0
requiredFinalStates = missing   # [-1.7224720653038268e-14, 0.4999999999999575, -0.750000016852883, -1.5681975307975595e-14, 0.0999999999999862, -0.8999999999999555, 12.518049838490617, -0.9018643787079454, 13.979844144766544, 2.0096818051930816, -0.9803470582617598, 2.9999999999999933]
simulate!(shaft, stopTime=stopTime, dtmax=0.1, log=true, logEvents=true, logStates=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(shaft, "free.rot", figure=1)
plot(shaft, ["free.r", "free.v", "free.w"], figure=2)
plot(shaft, ["free.z_rot123", "free.isrot123"], figure=3)
        
end
