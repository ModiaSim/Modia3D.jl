module BouncingSphere3DfreeMotion

using Modia3D
using Modia3D.StaticArrays

BouncingSphere = Model3D(
    boxHeigth = 0.1,
    world     = Object3D(feature=Scene(enableContactDetection=false)),
    ground    = Object3D(parent=:world,
                         translation=:[0.0,-boxHeigth/2,0.0],
                         feature=Solid(shape=Box(lengthX=4.0, lengthY=:boxHeigth, lengthZ=0.7),
                                       visualMaterial=VisualMaterial(color="DarkGreen"),
                                       solidMaterial="Steel",
                                       collision=true)),
    sphere    = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="Steel",
                                       massProperties=MassPropertiesFromShapeAndMass(mass=0.001),
                                       collision=true)),
    free = FreeMotion2(obj1=:world, obj2=:sphere, r=Var(init=SVector{3,Float64}(0.0, 1.0, 0.0)))
)

bouncingSphere = @instantiateModel(BouncingSphere, unitless=true, logCode=true)

simulate!(bouncingSphere, stopTime=0.2, dtmax=0.1, log=true, logStates=true) 
showInfo(bouncingSphere)

@usingModiaPlot
plot(bouncingSphere, "free.r", figure=1)

simulate!(bouncingSphere, IDA(), nlinearMinForDAE=1, stopTime=2.2, dtmax=0.1, log=true, logStates=true)  #, requiredFinalStates=requiredFinalStates)
showInfo(bouncingSphere)

end
