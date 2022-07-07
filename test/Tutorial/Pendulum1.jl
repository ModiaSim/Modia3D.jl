module Pendulum1
using Modia3D

Pendulum = Model3D(
    world     = Object3D(feature=Scene()),
    body      = Object3D(feature=Solid(massProperties=MassProperties(mass=1.0))),
    bodyFrame = Object3D(parent=:body, translation=[-0.5, 0.0, 0.0]),
    rev       = Revolute(obj1=:world, obj2=:bodyFrame)
)

pendulum = @instantiateModel(Pendulum, unitless=true)
simulate!(pendulum, stopTime=3.0)

@usingPlotPackage     # use the plot package defined by ENV["MODIA_PLOT"]
plot(pendulum, "rev.phi")

end
