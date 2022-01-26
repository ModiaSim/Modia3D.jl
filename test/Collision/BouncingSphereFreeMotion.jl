module BouncingSphereFreeMotion

using Modia

BouncingSphere = Model(
    boxHeigth = 0.05,
    groundMaterial = VisualMaterial(color="DarkGreen", transparency=0.5),
    sphereMaterial = VisualMaterial(color="Red"),
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false, defaultFrameLength=0.2, visualizeBoundingBox = true,
                                   enableContactDetection=true, visualizeContactPoints=false)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world, translation=:[0.0, -boxHeigth/2, 0.0],
                      feature=Solid(shape=Box(lengthX=0.8, lengthY=:boxHeigth, lengthZ=0.5),
                                    visualMaterial=:groundMaterial, solidMaterial="Steel",
                                    collision=true)),
    bottom = Object3D(parent=:world, translation=:[0.0, -0.4-boxHeigth/2, 0.5],
                      feature=Solid(shape=Box(lengthX=0.8, lengthY=:boxHeigth, lengthZ=0.5),
                                    visualMaterial=:groundMaterial, solidMaterial="Steel",
                                    collision=true)),
    wall = Object3D(parent=:world, translation=:[0.0, -0.2, 0.75+boxHeigth/2],
                    feature=Solid(shape=Box(lengthX=0.8, lengthY=0.4, lengthZ=:boxHeigth),
                                  visualMaterial=:groundMaterial, solidMaterial="Steel",
                                  collision=true)),
    sphere = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                                    visualMaterial=:sphereMaterial, solidMaterial="Steel",
                                    collision=true)),
    free = FreeMotion(obj1=:world, obj2=:sphere, r=Var(init=[0.0, 1.0, 0.0]), w=Var(init=[10.0, 0.0, -5.0]))
)

bouncingSphere = @instantiateModel(buildModia3D(BouncingSphere), unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingSphere.parameterExpressions
#@show bouncingSphere.parameters

stopTime = 2.7
tolerance = 1e-8
requiredFinalStates = [0.293772074878505, -1.2391513658578504, -0.11411843291553968, 0.11420129485535506, -4.417120013752265, -0.8579844538520917, 2.5996315776217718, -0.42018472064783946, 0.10773235538046359, -7.2176708654581585, 0.8693271212008165, 4.766350077776115]
simulate!(bouncingSphere, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingSphere, ["free.r" "free.rot"; "free.v" "free.w"], figure=1)

end
