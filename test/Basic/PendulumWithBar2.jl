module PendulumWithBar2

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")

Bar = Model(
    Lx = 0.1,
    frame0 = Object3D(feature=Solid(shape=Beam(axis=1, length=:Lx, width=:(0.2*Lx), thickness=:(0.2*Lx)),
                                    solidMaterial="Aluminium",
                                    visualMaterial = VisualMaterial(color="Turquoise4", transparency=0.5))),
	frame1 = Object3D(parent=:frame0,
                      translation=:[-Lx/2, 0.0, 0.0],
                      feature = Visual(shape=Cylinder(axis=3, diameter=:(Lx/10), length=:(0.25*Lx)),
                                       visualMaterial=VisualMaterial(color="Red")))
)

Pendulum = Model(
    Lx = 1.0,
    world = Object3D(feature=Scene() ),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:(Lx/2)))),
    bar = Bar | Map(Lx=:Lx),
    rev = RevoluteWithFlange(obj1=:world, obj2=:(bar.frame1), phi=Var(init=1.0)),

    damper = Damper | Map(d=40.0),
    fixed  = Fixed,
    connect = :[(damper.flange_b, rev.flange),
                (damper.flange_a, fixed.flange)]
)

#@showModel Pendulum

pendulum = @instantiateModel(buildModia3D(Pendulum), unitless=true)

#@showModel pendulum.parameterExpressions
#@showModel pendulum.parameters

stopTime = 6.0
requiredFinalStates = [-1.47312952226279, -0.5146766053301051]
simulate!(pendulum, stopTime=stopTime, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pendulum, ["rev.phi", "rev.w"], figure=1)

end
