module BouncingEllipsoidOnSphere

using Modia3D

BouncingEllipsoid = Model(
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
    free = FreeMotion(obj1=:world, obj2=:ellipsoid, r=Var(init=[0.0, 1.0, 0.0]), w=Var(init=[5.0, 0.0, -2.0]))
)

bouncingEllipsoid = @instantiateModel(buildModia3D(BouncingEllipsoid), unitless=true, log=false, logStateSelection=false, logCode=false)


stopTime = 2.0
tolerance = 1e-8
requiredFinalStates = [1.9013751140911312, -5.182612613347552, -0.6285338998457255, 1.3889960373284438, -10.631628266236548, -0.35235710867283077, 2.6873629925159035, 0.23896916015991412, -8.030636545239265, 6.713810967152868, -1.2032068837594854, -12.984033346958665]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingEllipsoid, ["free.r" "free.rot"; "free.v" "free.w"], figure=1)

end
