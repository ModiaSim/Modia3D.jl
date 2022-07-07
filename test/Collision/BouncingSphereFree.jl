module BouncingSphereFree

using Modia3D

BouncingSphere = Model3D(
    boxHeigth = 0.05,
    groundMaterial = VisualMaterial(color="DarkGreen", transparency=0.5),
    sphereMaterial = VisualMaterial(color="Red"),
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox=true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, -boxHeigth/2, 0.0],
                      feature=Solid(shape=Box(lengthX=0.8, lengthY=:boxHeigth, lengthZ=0.5),
                                    visualMaterial=:groundMaterial,
                                    solidMaterial="Steel",
                                    collision=true)),
    bottom = Object3D(parent=:world,
                      translation=:[0.0, -0.4-boxHeigth/2, 0.5],
                      feature=Solid(shape=Box(lengthX=0.8, lengthY=:boxHeigth, lengthZ=0.5),
                                    visualMaterial=:groundMaterial,
                                    solidMaterial="Steel",
                                    collision=true)),
    wall = Object3D(parent=:world,
                    translation=:[0.0, -0.2, 0.75+boxHeigth/2],
                    feature=Solid(shape=Box(lengthX=0.8, lengthY=0.4, lengthZ=:boxHeigth),
                                  visualMaterial=:groundMaterial,
                                  solidMaterial="Steel",
                                  collision=true)),
    sphere = Object3D(parent=:world, fixedToParent=false,
                      translation=[0.0, 1.0, 0.0],
                      angularVelocity=[10.0, 0.0, -5.0],
                      feature=Solid(shape=Sphere(diameter=0.2),
                                    visualMaterial=:sphereMaterial,
                                    solidMaterial="Steel",
                                    collision=true))
)

bouncingSphere = @instantiateModel(BouncingSphere, unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingSphere.parameterExpressions
#@show bouncingSphere.parameters

stopTime = 2.7
tolerance = 1e-8
requiredFinalStates = [0.292862783655338, -1.2390369485250046, -0.13258270312197515, 0.11349011490811044, -4.416810256880555, -0.8737129049845306, 2.3699488443877197, -0.4268724068718218, 0.1382090729085924, -8.738616638760442, -0.8148444006512789, -1.1350943780421976]
simulate!(bouncingSphere, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false,
          requiredFinalStates_rtol=0.2, requiredFinalStates_atol=0.2, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingSphere, ["sphere.translation" "sphere.rotation"; "sphere.velocity" "sphere.angularVelocity"], figure=1)

end
