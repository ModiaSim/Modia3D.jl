module BouncingEllipsoidOnSphere

using Modia3D

BouncingEllipsoid = Model3D(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
    mprTolerance=1.0e-19,
                                   defaultFrameLength=0.2,
                                   enableContactDetection=true)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0,-boxHeigth/2,0.0],
                      feature=Solid(shape=Sphere(diameter=1.5),
                                    visualMaterial=VisualMaterial(color="DarkGreen", transparency=0.5),
                                    solidMaterial="Steel",
                                    collision=true)),
    ellipsoid = Object3D(feature=Solid(shape=Ellipsoid(lengthX=0.1, lengthY=0.2, lengthZ=0.3),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="Steel",
                                       collision=true)),
    free = FreeMotion(obj1=:world, obj2=:ellipsoid, r=Var(init=Modia.SVector{3,Float64}(0.0, 1.0, 0.0)), w=Var(init=Modia.SVector{3,Float64}(5.0, 0.0, -2.0)))
)

bouncingEllipsoid = @instantiateModel(BouncingEllipsoid, unitless=true, log=false, logStateSelection=false, logCode=false)


stopTime = 2.0
tolerance = 1e-8
requiredFinalStates = [1.8699971636706905, -5.22232657773475, -0.6154352883726985, 1.361642114424583, -10.681395509044837, -0.3348496967765453, 2.49391204993144, 0.3726321363358523, -8.102291890637183, 6.4559770207821305, -3.701868523562044, -12.251027809172985]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingEllipsoid, ["free.r" "free.rot"; "free.v" "free.w"], figure=1)

end
