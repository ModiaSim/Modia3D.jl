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
    requiredFinalStates = [1.6872544817380444, -1.9587432095329889, 1.553289445400032, 4.5870758721172, -3.6273026821182888, -1.8308615472313539, 16.048916787369645, 0.11940970690989604, 8.103360754506756, 28.819004920911127, 8.50096951972608, 37.13678202093278]
elseif Sys.isapple()
    requiredFinalStates = [1.6869846641079747, -1.9588606270293238, 1.5530247319055936, 4.5856253956554225, -3.6281065583891867, -1.8321913845833115, 16.05045750700684, 0.11060518842830577, 8.104158851820499, 28.80641529588638, 8.168480244718031, 37.20010873664111]
else
    requiredFinalStates = [1.6863191515678857, -1.959326139586727, 1.5524868091473853, 4.582527752958374, -3.630808257742642, -1.8352059702486623, 16.048770476114274, 0.11319234251167573, 8.101743640120217, 28.75572277458283, 8.390512432007, 37.189873260046895]
end
simulate!(convexPartitions, stopTime=stopTime, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

end
