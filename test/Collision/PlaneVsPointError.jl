module LineLineSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

LineLine = Model3D(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, 0, -1]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox=true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, 0.0],
                      feature=Solid(shape=Box(lengthX=5.0, lengthY=3.0, lengthZ=0.0),
                                    visualMaterial=vmatGrey,
                                    solidMaterial="Steel",
                                    massProperties = MassProperties(mass=12000.0),
                                    collision=true)),
    frameX = Object3D(parent=:world,
                      translation=:[0.1, 0.1, 0.0],
                      rotation=:[0.0, 0.0, 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    capsuleX = Object3D(parent=:frameX, fixedToParent=false,
                        velocity=[0.0, 1.0, 0.0],
                        feature=Solid(shape=Sphere(diameter=0.0),
                                      visualMaterial=vmatRed,
                                      solidMaterial="Steel",
                                      massProperties=MassProperties(mass=1273.39, Ixx=24.39551415267595, Iyy=185.24505801647337, Izz=185.24505801647337, Ixy=0.0, Ixz=0.0, Iyz=0.0),
                                      collision=true))
)

lineLine = @instantiateModel(LineLine, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 0.0
tolerance = 1e-8
requiredFinalStates = missing
simulate!(lineLine, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

end
