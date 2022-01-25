module PendulumWithBar1

using ModiaLang

# ModiaLang models
include("$(ModiaLang.path)/models/Blocks.jl")
include("$(ModiaLang.path)/models/Electric.jl")
include("$(ModiaLang.path)/models/Rotational.jl")

using Modia

Bar = Model(
    m  = 0.1,
    Lx = 0.1,
    Ly = Par(value=:(0.2*Lx)),
    Lz = Par(value=:(0.2*Lx)),
    vmat1 = VisualMaterial(color="DeepSkyBlue2", transparency=0.5),
    vmat2 = VisualMaterial(color="Red"),
    frame0 = Object3D(feature=Solid(shape=Beam(axis=1, length=:Lx, width=:Ly, thickness=:Lz),
                                    massProperties=MassProperties(mass=:m),
                                    visualMaterial=:(vmat1))),
	frame1 = Object3D(parent=:frame0,
                      translation=:[-Lx/2, 0.0, 0.0],
                      feature=Visual(shape=Cylinder(axis=3, diameter=:(Ly/2), length=:(1.2*Lz)),
                                     visualMaterial=:(vmat2)))
)

Pendulum = Model(
    m = 1.0,
    Lx = 0.1,
    world = Object3D(feature=Scene() ),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:(Lx/2)))),
    bar = Bar | Map(m=:m, Lx=:Lx),
    rev = RevoluteWithFlange(obj1=:world, obj2=:(bar.frame1))
)

PendulumWithBar = Model(
    pendulum = buildModia3D(Pendulum | Map(Lx=1.0, m=2.0, rev=Map(phi=Var(init=1.0)))),

    damper = Damper | Map(d=0.5),
    fixed = Fixed,
    connect = :[(damper.flange_b, pendulum.rev.flange),
                (damper.flange_a, fixed.flange)]
)

pendulumWithBar = @instantiateModel(PendulumWithBar, unitless=true)

import DifferentialEquations
algorithm = DifferentialEquations.Tsit5()
stopTime = 10.0
requiredFinalStates = [-1.578178763749515, 0.06153191687388868]
simulate!(pendulumWithBar, algorithm, stopTime=stopTime, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pendulumWithBar, ["pendulum.rev.flange.phi", "pendulum.rev.variables[1]"], figure=1)

end
