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
    requiredFinalStates = [1.687587439697609, -1.958402549228965, 1.5536690340762165, 4.588930235927663, -3.6255044015487035, -1.8288741879694177, 16.049606070613557, 0.12014325285843275, 8.103183190218997, 28.842653455779967, 8.498278222288814, 37.12003462990435]
else
    requiredFinalStates = [1.6872499694410275, -1.9587662078464587, 1.5532215079622953, 4.587022425608039, -3.62740095686553, -1.8312196793551516, 16.048892412746557, 0.1191036977124399, 8.10351967977002, 28.817235547914752, 8.48856343583909, 37.14063532251365]
end
simulate!(convexPartitions, stopTime=stopTime, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

end
