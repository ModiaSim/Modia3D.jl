module FreeShaft

using Modia3D

Shaft = Model(
    Length = 1.0,
    Diameter = 0.2,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=0.3, n=[0, 0, -1]))),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Diameter))),
    shaft = Object3D(parent=:world, fixedToParent=false,
                     feature=Solid(shape=Cylinder(axis=3, diameter=:Diameter, length=:Length),
                                   massProperties=MassProperties(; mass=84.7154, Ixx=7.2711, Iyy=7.2711, Izz=0.4230),
                                   visualMaterial=:(visualMaterial))),
)

model = Model3D(
    shaft = Shaft | Map(Length=1.0, Diameter=0.2, shaft = Map(velocity=[0.0, 0.1, 0.6],
                                                              rotation=[30, 20, 10]u"°",
                                                              angularVelocity=Modia3D.resolve1([30, 20, 10]u"°", [1.0, 2.0, 3.0]))
                       )
)
#@showModel model

shaft = @instantiateModel(model, unitless=true, log=false, logStateSelection=false, logCode=false)

#@showModel shaft.p[1]

stopTime = 5.0
requiredFinalStates=[0.0, 0.5, -0.75, 0.0, 0.1, -0.9, 12.518049845080435, -0.901864379852841, 13.979844149553378, -1.556318674959713, 1.9667313208626755, 2.776623306112369]
simulate!(shaft, stopTime=stopTime, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(shaft, ["shaft.shaft.translation", "shaft.shaft.rotation", "shaft.shaft.velocity", "shaft.shaft.angularVelocity"], figure=1)

end
