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
    requiredFinalStates = missing
elseif Sys.isapple()
    requiredFinalStates = missing
else
    requiredFinalStates = [1.688018321309494, -1.9582483904718009, 1.5539704760382338, 4.590793306938194, -3.6243296926379154, -1.8271865809252648, 16.049161819683633, 0.1240921474760183, 8.104560487622566, -10.760004116947046, -11.394536761055607, -45.138186381013554]
end
simulate!(convexPartitions, stopTime=stopTime, log=true, logStates=false, logEvents=false,
          requiredFinalStates_rtol=0.1, requiredFinalStates_atol=0.1, requiredFinalStates=requiredFinalStates)

end
