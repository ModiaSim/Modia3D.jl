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
    ellipsoid = Object3D(feature=Solid(shape=Ellipsoid(lengthX=0.1, lengthY=0.2, lengthZ=0.3),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="Steel",
                                       collision=true)),
    free = FreeMotion(obj1=:world, obj2=:ellipsoid, r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 1.0, 0.0)), w=Var(init=ModiaBase.SVector{3,Float64}(5.0, 0.0, -2.0)))
)

bouncingEllipsoid = @instantiateModel(BouncingEllipsoid, unitless=true, log=false, logStateSelection=false, logCode=false)


stopTime = 2.0
tolerance = 1e-8
requiredFinalStates = [-0.48480963673904126, -0.31544579779174053, 1.8217647164503947, -0.5223680077157852, -2.9525461969237083, 1.0275379269471818, 16.477526009450994, 0.45153448141912156, -0.5122120185437135, 8.566326595961337, 1.198449699792746, 1.45190940975166]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingEllipsoid, ["free.r" "free.rot"; "free.v" "free.w"], figure=1)

end
