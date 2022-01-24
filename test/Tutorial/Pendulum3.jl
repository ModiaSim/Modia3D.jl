module Pendulum3
import Modia3D # temporarily, needs to be deleted!
using Modia

# Modia equation-based models
include("$(Modia.modelsPath)/AllModels.jl")

Pendulum = Model(
    world = Object3D(feature=Scene(animationFile="Pendulum3.json")),
    obj1  = Object3D(feature=Solid(shape=Beam(axis=1, length=1.0, width=0.2, thickness=0.2),
                solidMaterial="Steel", visualMaterial=VisualMaterial(color="Blue"))),
    obj2  = Object3D(parent=:obj1, feature=Visual(shape=Cylinder(diameter=0.1, length=0.21),
                visualMaterial=VisualMaterial(color="Red")), translation=[-0.5, 0.0, 0.0]),
    rev   = RevoluteWithFlange(obj1=:world, obj2=:obj2),

    damper  = Damper | Map(d=100.0),
    fixed   = Fixed,
    connect = :[(damper.flange_b, rev.flange),
                (damper.flange_a, fixed.flange)]
)

pendulum = @instantiateModel(buildModia3D(Pendulum), unitless=true)
simulate!(pendulum, stopTime=3.0)

@usingModiaPlot
plot(pendulum, "rev.phi")

end
