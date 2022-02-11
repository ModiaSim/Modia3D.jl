module ShaftFreeMotion

using Modia3D

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
    free = FreeMotion(obj1=:world, obj2=:shaft)
)

model = Model(
    shaft = buildModia3D(Shaft | Map(Length=1.0, Diameter=0.2, free = Map(v   = Var(init = ModiaBase.SVector{3,Float64}([0.0, 0.1, 0.6])),
                                                                          rot = Var(init = ModiaBase.SVector{3,Float64}([deg2rad(30), deg2rad(20), deg2rad(10)])),
                                                                          w   = Var(init = ModiaBase.SVector{3,Float64}([1.0, 2.0, 3.0]))
                                                                         )
                                    )
                        )
)
#@showModel model

shaft = @instantiateModel(model, aliasReduction=false, unitless=true, log=false, logStateSelection=false, logCode=false)

#@showModel shaft.p[1]

stopTime = 5.0
requiredFinalStates = [3.836315262587382e-12, 0.49999999999093686, -0.7500000168549844, 12.518049839479245, -0.901864378879273, 13.979844145484941, 1.1943971479556706e-13, 0.09999999999709909, -0.900000000001054, 2.009681805153073, -0.9803470582586878, 3.000000000000504]
simulate!(shaft, stopTime=stopTime, log=true, logStates=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(shaft, ["shaft.free.r", "shaft.free.rot", "shaft.free.v", "shaft.free.w"], figure=1)

end
