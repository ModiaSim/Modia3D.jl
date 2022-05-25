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
                                                              angularVelocity=[1.0, 2.0, 3.0],
                                                              angularVelocityInParent=false)
                       )
)
#@showModel model

shaft = @instantiateModel(model, unitless=true, log=false, logStateSelection=false, logCode=false)

#@showModel shaft.p[1]

stopTime = 5.0
requiredFinalStates=[-1.7224720653038268e-14, 0.4999999999999575, -0.750000016852883, -1.5681975307975595e-14, 0.0999999999999862, -0.8999999999999555, 12.518049838490617, -0.9018643787079454, 13.979844144766544, 2.0096818051930816, -0.9803470582617598, 2.9999999999999933]
simulate!(shaft, stopTime=stopTime, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(shaft, ["shaft.shaft.translation", "shaft.shaft.rotation", "shaft.shaft.velocity", "shaft.shaft.angularVelocity"], figure=1)

end
