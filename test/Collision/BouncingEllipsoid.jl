module BouncingEllipsoidSimulation

using Modia3D

BouncingEllipsoid = Model3D(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0,-boxHeigth/2,0.0],
                      feature=Solid(shape=Box(lengthX=4.0, lengthY=:boxHeigth, lengthZ=3.0),
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
requiredFinalStates = [-0.4848756849364805, -0.3152596292050405, 1.8217924654907032, -0.5225235181495869, -2.951876778895525, 1.0276278036774111, 16.477475569988698, 0.4513592819174489, -0.5123012801778761, 7.880815138676465, 0.7279928544430719, 3.7799789451161594]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, requiredFinalStates=requiredFinalStates)

@usingPlotPackage
plot(bouncingEllipsoid, ["ellipsoid.translation" "ellipsoid.rotation"; "ellipsoid.velocity" "ellipsoid.angularVelocity"], figure=1)

end
