module PendulumWithSpring

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")

simulationModel = nothing
get_simulationModel() = simulationModel

Pendulum = Model(
    m = 1.0,
    g = 9.81,

    world = Object3D(feature=Scene()),
    body  = Object3D(feature = Solid(massProperties=MassProperties(mass=:m, centerOfMass=[0.5, 0.0, 0.0]))),
    rev   = RevoluteWithFlange(obj1=:world, obj2=:body)
)

PendulumWithSpr = Model(
    pendulum = buildModia3D(Pendulum | Map(Lx=0.5, m=2, rev=Map(phi=Var(init=1.0)))),
    spring   = Spring | Map(c=100.0u"N*m/rad"),
	support  = Fixed,

    connect = :[ (pendulum.rev.flange, spring.flange_b)
                 (spring.flange_a, support.flange)
               ]
)

pendulumWithSpring = @instantiateModel(PendulumWithSpr, aliasReduction=false, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.0
requiredFinalStates = [-0.1166492890518372, -15.325157691194002]
simulate!(pendulumWithSpring, stopTime=stopTime, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pendulumWithSpring, ["pendulum.rev.flange.phi", "pendulum.rev.variables[1]"], figure=1)

end
