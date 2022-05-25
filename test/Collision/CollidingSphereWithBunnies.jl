module CollidingSphereWithBunnies

using Modia3D

filenameBunny = joinpath(Modia3D.path, "objects", "bunny", "bunny.obj")

# Properties
mat1 = "Red"

massAndGeoSphere = MassPropertiesFromShape()

# Objects3D
ConvexPartitions = Model3D(
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   mprTolerance = 1.0e-13)),

    sphere = Object3D(parent=:world, fixedToParent=false,
                      translation=[0.0, 2.0, 0.0],
                      feature=Solid(shape=Sphere(diameter=0.2),
                                    visualMaterial=mat1,
                                    solidMaterial="Steel",
                                    massProperties=massAndGeoSphere,
                                    collision=true)),

    bunnySolid = Object3D(parent=:world,
                          feature=Solid(shape=FileMesh(filename=filenameBunny, scale=[0.1, 0.1, 0.1]),
                                        solidMaterial="Steel",
                                        collision=true)),

    bunnyPartitionSolid = Object3D(parent=:world,
                                   translation=[0.5, -2.0, 2.0],
                                   feature=Solid(shape=FileMesh(filename=filenameBunny, scale=[0.1, 0.1, 0.1], convexPartition=true),
                                                 solidMaterial="Steel",
                                                 collision=true)),
)

convexPartitions = @instantiateModel(ConvexPartitions, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.3
if Sys.iswindows()
    requiredFinalStates = [1.6940802130682324, -1.9512207990517003, 1.5606303878814574, 4.625880034489348, -3.589924177921963, -1.7907826612627857, 16.0608195556636, 0.13780357845450728, 8.105673207978326, 29.30284410483668, 8.405274634598163, 36.802065972148036]
elseif Sys.isapple()
    requiredFinalStates = [1.6880063890644685, -1.9582632262628437, 1.5539365583304472, 4.590730712867433, -3.6244054755632917, -1.8273707703667448, 16.049119801592937, 0.12408137853889413, 8.10459119196184, 28.869167610906487, 8.579225687514493, 37.09510325836121]
else
    requiredFinalStates = [1.688003406721993, -1.9582680842363085, 1.5538964043050503, 4.5907238626516245, -3.624424998072034, -1.8275944766779806, 16.049076939228613, 0.12407802433727105, 8.10458100514745, 28.86780633478002, 8.581294947684691, 37.09568176938176]
end
simulate!(convexPartitions, stopTime=stopTime, log=true, logStates=false, logEvents=false,
          requiredFinalStates_rtol=0.1, requiredFinalStates_atol=0.1, requiredFinalStates=requiredFinalStates)

end
