module PendulumWithParameterizedDamper

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")

simulationModel = nothing
get_simulationModel() = simulationModel

Pendulum = Model(
    m = 1.0,
    Lx = 0.1,
    Ly = Par(value=:(0.2*Lx)),
    Lz = Par(value=:(0.2*Lx)),
    vmat1 = VisualMaterial(color="VioletRed", transparency=0.5),
    vmat2 = VisualMaterial(color="Red"),
    world = Object3D(feature=Scene()),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:(Lx/2)))),

    frame0 = Object3D(feature=Solid(shape=Beam(axis=1, length=:Lx, width=:Ly, thickness=:Lz),
                                    massProperties=MassProperties(mass=:m),
                                    visualMaterial=:(vmat1))),
	frame1 = Object3D(parent=:(frame0), translation=:[-Lx/2, 0.0, 0.0]),
    cyl    = Object3D(parent=:(frame1),
                      feature=Visual(shape=Cylinder(axis=3, diameter=:(Ly/2), length=:(1.2*Lz)),
                      visualMaterial=:(vmat2))),
    rev    = RevoluteWithFlange(obj1=:world, obj2=:frame1)
)

PendulumWithDamper = Model(
    pendulum = buildModia3D(Pendulum | Map(Lx=1.0, m=2.0, rev=Map(phi=Var(init=1.0)))),

    damper = Damper | Map(d=0.5),
    fixed = Fixed,
    connect = :[(damper.flange_b, pendulum.rev.flange),
                (damper.flange_a, fixed.flange)]
)

#@showModel PendulumWithDamper

simulationModel = @instantiateModel(PendulumWithDamper, aliasReduction=false, unitless=true, log=false, logStateSelection=false, logCode=false)

#@showModel simulationModel.p[1]

stopTime = 10.0
requiredFinalStates = [-1.578178283450938, 0.061515170100766486]
simulate!(simulationModel, stopTime=stopTime, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(simulationModel, "pendulum.rev.flange.phi", figure=1)

end
