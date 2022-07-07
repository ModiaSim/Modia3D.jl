module DoublePendulumWithDampers

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")

Bar = Model(
    Lx = 0.1,
    Ly = Par(value=:(0.2*Lx)),
    Lz = Par(value=:(0.2*Lx)),
    vmat1 = VisualMaterial(color="SpringGreen4", transparency=0.5),
    vmat2 = VisualMaterial(color="Red"),
    frame0 = Object3D(feature=Solid(shape=Beam(axis=1, length=:Lx, width=:Ly, thickness=:Lz),
                                    solidMaterial="Aluminium",
                                    visualMaterial=:vmat1)),
	frame1 = Object3D(parent=:frame0,
                      feature=Visual(shape=Cylinder(axis=3, diameter=:(Ly/2), length=:(1.2*Lz)),
                                     visualMaterial=:(vmat2)),
                      translation=:[-Lx/2, 0.0, 0.0]),
	frame2 = Object3D(parent=:frame0,
                      translation=:[ Lx/2, 0.0, 0.0])
)

DoublePendulum = Model3D(
    Lx = 1.0,
    world = Object3D(feature=Scene()),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:(Lx/2)))),
    bar1 = Bar | Map(Lx=:Lx),
    bar2 = Bar | Map(Lx=:Lx),
    rev1 = RevoluteWithFlange(obj1=:world, obj2=:(bar1.frame1), phi=Var(init=1.0)),
    rev2 = RevoluteWithFlange(obj1=:(bar1.frame2), obj2=:(bar2.frame1)),

    damper1 = Damper | Map(d=20.0),
    fixed1  = Fixed,
    damper2 = Damper | Map(d=20.0),
    fixed2  = Fixed,
    connect = :[(damper1.flange_b, rev1.flange),
                (damper1.flange_a, fixed1.flange),
                (damper2.flange_b, rev2.flange),
                (damper2.flange_a, fixed2.flange)]
)

doublePendulum = @instantiateModel(DoublePendulum, unitless=true)

stopTime = 10.0
tolerance = 1.0e-8
requiredFinalStates = [-0.9608496178685947, 1.0258202272580021, -6.0828664801544345, -0.11676963250128454]
simulate!(doublePendulum, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(doublePendulum, [("rev1.phi", "rev2.phi"), ("rev1.w", "rev2.w")])

end
