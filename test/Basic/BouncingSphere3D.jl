module BouncingSphere3D
using Modia3D

BouncingSphere = Model3D(
    boxHeigth = 0.1,
    world     = Object3D(feature=Scene()),
    ground    = Object3D(parent=:world, translation=:[0.0,-boxHeigth/2,0.0],
                         feature=Solid(shape=Box(lengthX=4.0, lengthY=:boxHeigth, lengthZ=0.7),
                                       visualMaterial=VisualMaterial(color="DarkGreen"),
                                       solidMaterial="Steel",
                                       collision=true)),
    sphere    = Object3D(parent=:world, fixedToParent=false, translation=[0.0, 1.0, 0.0], angularVelocityResolvedInParent=true,
                         feature=Solid(shape=Sphere(diameter=0.2),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="Steel",
                                       massProperties=MassPropertiesFromShapeAndMass(mass=0.001),
                                       collision=true)),
)

bouncingSphere = @instantiateModel(BouncingSphere, unitless=true, logCode=true)

requiredFinalStates = [-1.1158498420134682e-7, 0.09999999283736953, 1.9605796328352922e-10, -1.3226335283305713e-12, 3.726293803759173e-13, 6.442335795625085e-16, -4.97188563948566e-9, 6.3453729183502065e-15, -2.8482188606342696e-6, 6.351235882032115e-15, -4.677163261435682e-20, 1.3039308706792498e-11]
simulate!(bouncingSphere, stopTime=2.2, dtmax=0.1, log=true, logStates=true, requiredFinalStates=requiredFinalStates)
showInfo(bouncingSphere)

@usingModiaPlot
plot(bouncingSphere, ["sphere.translation", "sphere.velocity", "sphere.rotation123"], figure=1)

end
