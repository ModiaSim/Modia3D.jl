module CollidingSphereWithBunnies

using Modia3D

filenameBunny = joinpath(Modia3D.path, "objects", "bunny", "bunny.obj")

# Properties
mat1 = "Red"

massAndGeoSphere = MassPropertiesFromShape()

# Objects3D
ConvexPartitions = Model(
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField, mprTolerance = 1.0e-13)),

    sphere = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                      visualMaterial=mat1, solidMaterial="Steel",
                      massProperties=massAndGeoSphere, collision=true)),
    free = FreeMotion(obj1=:world, obj2=:sphere, r=Var(init=[0.0, 2.0, 0.0])),

    bunnySolid = Object3D(parent=:world, feature=Solid(shape =
        FileMesh(filename=filenameBunny, scale=[0.1, 0.1, 0.1]), solidMaterial="Steel", collision=true )),

    bunnyPartitionSolid = Object3D(parent=:world, translation=[0.5, -2.0, 2.0], feature=Solid(shape =
        FileMesh(filename=filenameBunny, scale=[0.1, 0.1, 0.1], convexPartition=true), solidMaterial="Steel", collision=true)),
)

convexPartitions = @instantiateModel(buildModia3D(ConvexPartitions), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.3
if Sys.iswindows()
    requiredFinalStates = [1.6852436731117506, -1.9602009870847892, 1.5537079430670875, 4.576280426855566, -3.6359538989543787, -1.8281828479408744, 16.048751238499957, 0.11284016337050634, 8.101268624505751, 28.746819815200922, 8.39876218585158, 37.19093884427305]
else
    requiredFinalStates = [1.6863191515678857, -1.959326139586727, 1.5524868091473853, 4.582527752958374, -3.630808257742642, -1.8352059702486623, 16.048770476114274, 0.11319234251167573, 8.101743640120217, 28.75572277458283, 8.390512432007, 37.189873260046895]
end
simulate!(convexPartitions, stopTime=stopTime, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

end
