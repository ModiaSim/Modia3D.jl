module CollidingSphereWithBunnies

using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface

filenameBunny = joinpath(Modia3D.path, "objects", "bunny", "bunny.obj")

# Properties
mat1 = "Red"

massAndGeoSphere = MassPropertiesFromShape()

# Objects3D
ConvexPartitions = Model(
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField, visualizeBoundingBox = true)),

    sphere = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                      visualMaterial=mat1, solidMaterial="Steel",
                      massProperties=massAndGeoSphere, collision=true)),
    free = FreeMotion(obj1=:world, obj2=:sphere, r=Var(init=[0.0, 2.0, 0.0])),


    bunnySolid = Object3D(parent=:world, feature=Solid(shape =
        FileMesh(filename=filenameBunny, scale=[0.1, 0.1, 0.1]), solidMaterial="Steel", collision = true )),

    bunnyPartitionSolid = Object3D(parent=:world, translation=[0.5, -2.0, 2.0], feature=Solid(shape =
        FileMesh(filename = filenameBunny, scale=[0.1, 0.1, 0.1], convexPartition=true), solidMaterial="Steel", collision=true)),
)

convexPartitions = @instantiateModel(buildModia3D(ConvexPartitions), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.3
requiredFinalStates = [1.6852536030611518, -1.9601748772369065, 1.553773946072609, 4.576339600748371, -3.635822903848681, -1.8278191918133224, 16.048785711292467, 0.11321559138403894, 8.101411301098336, 28.75260252729643, 8.402335107079937, 37.18798381568381]
@time simulate!(convexPartitions, stopTime=stopTime, log=true, logStates=false, logEvents=true, requiredFinalStates=requiredFinalStates)

end
