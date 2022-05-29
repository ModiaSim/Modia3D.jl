module PendulumModule

using Modia3D

Pendulum = Model3D(
    world     = Object3D(feature=Scene()),
    body      = Object3D(feature=Solid(massProperties=MassProperties(mass=1.0))),
    bodyFrame = Object3D(parent=:body, translation=[-0.5, 0.0, 0.0]),
    rev       = Revolute(obj1=:world, obj2=:bodyFrame)
)

pendulum = @instantiateModel(Pendulum, unitless=true, log=false, logDetails=false, logCode=true, logStateSelection=false, logCalculations=false)
requiredFinalStates = [5.970529829666215, -1.1395482781332746]
simulate!(pendulum, stopTime=3.0, log=true, requiredFinalStates = requiredFinalStates)

@usingModiaPlot
plot(pendulum, "rev.phi")


end
