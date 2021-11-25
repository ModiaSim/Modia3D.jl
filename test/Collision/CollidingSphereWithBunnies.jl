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
    requiredFinalStates = [1.686546063766475, -1.9591644219830313, 1.5526909426244875, 4.583598493384871, -3.6298844390498335, -1.8340691154926712, 16.049366198807483, 0.11239518445646898, 8.102435967337463, 28.773459446699306, 8.32007309434159, 37.191491434773106]
end
simulate!(convexPartitions, stopTime=stopTime, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

end
