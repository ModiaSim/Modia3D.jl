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
    world = Object3D(feature=Scene(gravityField=:gravField)),

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
    requiredFinalStates = [1.685933864090563, -1.959541492534972, 1.5521906830005383, 4.580925682845578, -3.632132649612497, -1.8369670152487543, 16.04763987164082, 0.11588729589359063, 8.099403988427044, 28.722584887014012, 8.588857729456306, 37.170274099922]
else
    requiredFinalStates = [1.6853549405539088, -1.9603839264641005, 1.5512239899411384, 4.577225786420592, -3.6364005190730575, -1.841958211129179, 16.04685907044025, 0.10996251002097332, 8.10149898250639, 28.66624352831703, 8.399969490696167, 37.24515015558254]
end
@time simulate!(convexPartitions, stopTime=stopTime, log=false, logStates=false, logEvents=true, requiredFinalStates=requiredFinalStates)

end
