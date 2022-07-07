module BouncingEllipsoidOnSphere

using Modia3D

BouncingEllipsoid = Model3D(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   mprTolerance=1.0e-20,
                                   defaultFrameLength=0.2,
                                   enableContactDetection=true)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0,-boxHeigth/2,0.0],
                      feature=Solid(shape=Sphere(diameter=1.5),
                                    visualMaterial=VisualMaterial(color="DarkGreen", transparency=0.5),
                                    solidMaterial="Steel",
                                    collision=true)),
    ellipsoid = Object3D(parent=:world, fixedToParent=false,
                         translation=[0.0, 1.0, 0.0],
                         angularVelocity=[5.0, 0.0, -2.0],
                         feature=Solid(shape=Ellipsoid(lengthX=0.1, lengthY=0.2, lengthZ=0.3),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="Steel",
                                       collision=true))
)

bouncingEllipsoid = @instantiateModel(BouncingEllipsoid, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 2.0
tolerance = 1e-8
requiredFinalStates = [1.8699989130901813, -5.222327808647324, -0.6154343686911469, 1.3616434929142358, -10.68139650349135, -0.33484894334316573, 2.493917143587957, 0.3726401558620851, -8.102293063844746, 10.930247061463712, -5.829926511030707, -7.212001322832058]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingEllipsoid, ["ellipsoid.translation" "ellipsoid.rotation"; "ellipsoid.velocity" "ellipsoid.angularVelocity"], figure=1)

end
