module Object3DWithoutParentError

# Should give an error because of two Object3Ds without parent

using ModiaLang
using Test

# ModiaLang models
include("$(ModiaLang.path)/models/Blocks.jl")
include("$(ModiaLang.path)/models/Electric.jl")
include("$(ModiaLang.path)/models/Rotational.jl")

import Modia3D
using  Modia3D.ModiaInterface

Bar = Model(
    m  = 0.1,
    Lx = 0.1,
    Ly = Par(value=:(0.2*Lx)),
    Lz = Par(value=:(0.2*Lx)),
    vmat1 = VisualMaterial(color="LightBlue", transparency=0.5),
    vmat2 = VisualMaterial(color="Red"),
    frame0 = Object3D(feature=Solid(shape=Beam(axis=1, length=:Lx, width=:Ly, thickness=:Lz),
                                    massProperties=MassProperties(mass=:m),
                                    visualMaterial=:(vmat1))),
	frame1 = Object3D(parent=:frame0,
                      translation=:[-Lx/2, 0.0, 0.0],
                      feature=Visual(shape=Cylinder(axis=3, diameter=:(Ly/2), length=:(1.2*Lz)),
                                     visualMaterial=:(vmat2))),
    frame2 = Object3D()  # has no parent
)

Pendulum = Model(
    m = 1.0,
    Lx = 0.1,
    world = Object3D(feature=Scene()),
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

stopTime = 10.0
requiredFinalStates = [-1.5781788131493184, 0.06153205563040136]
simulate!(pendulumWithBar, stopTime=stopTime, log=true, logStates=true, requiredFinalStates=requiredFinalStates)
@test occursin("Object3Ds have no parent", pendulumWithBar.lastMessage)

#@usingModiaPlot
#plot(pendulumWithBar, ["pendulum.rev.flange.phi", "pendulum.rev.variables[1]"], figure=1)

end
