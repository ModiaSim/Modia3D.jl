module PCMBubbleFunnel

using Modia3D

fileMeshFunnel = joinpath(Modia3D.path, "objects/pcm", "funnel.obj")
fileMeshBubble = joinpath(Modia3D.path, "objects/pcm", "bubble.obj")
visualMaterialFunnel = VisualMaterial(color="Blue")
visualMaterialBubble = VisualMaterial(color="Yellow")

BubbleFunnel = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=1.0)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.2))),
    funnel = Object3D(parent=:world,
                      feature=Solid(shape=FileMesh(filename=fileMeshFunnel, useMaterialColor=true, doubleSided=true),
                                    massProperties=MassProperties(; mass=1.0, Ixx=0.1, Iyy=0.1, Izz=0.1),
                                    visualMaterial=visualMaterialFunnel)),
    bubble = Object3D(parent=:world, fixedToParent=false,
                      translation=[0.0, 0.0, 1.0],
                      rotation=[-30*pi/180, 15*pi/180, 0.0],
                      feature=Solid(shape=FileMesh(filename=fileMeshBubble, useMaterialColor=true),
                                    massProperties=MassProperties(; mass=2.0, Ixx=0.02, Iyy=0.01, Izz=0.02),
                                    visualMaterial=visualMaterialBubble)),
    force = PolygonalContactModel(obj1=:funnel, obj2=:bubble,
        task=3,
        bodyF_YoungsModulus=5500.0,
        bodyF_PoissonsRatio=0.4,
        bodyF_LayerDepth=0.01,
        maxPenetration=0.05,
        compressionArealDampingFactor=500.0,
        expansionArealDampingFactor=500.0,
        expansionDampingMode=1,
        dampingTransitionDepth=0.001,
        frictionCoefficient=0.25,
        frictionRegularizationVelocity=0.0005)
)

pcmBubbleFunnel = @instantiateModel(BubbleFunnel, aliasReduction=false, unitless=true)

stopTime = 5.0
tolerance = 1e-6
dtmax = 0.01
#if Sys.iswindows()
#    requiredFinalStates = [0.0638936091654278, 0.06409563083976627, 0.13941982831466362, -0.0003008783973786521, 0.0004159697615743784, 4.5354518746797206e-5, -1.020210662992634, -0.8315221323544396, 1.6461865015635462, -0.000435788082208301, 0.00501850575137954, 0.0005270578867442704]
#elseif Sys.isapple()
#    requiredFinalStates = [0.06457076328176393, 0.06412676867271479, 0.13947517345242724, -0.0025407027108012652, 0.0025806810548322204, -0.00017121144117383113, -1.0176852609422187, -0.8350544337027049, 1.648126653819517, -0.002876751935016818, 0.03663408884741272, 0.0005195024017150574]
#elseif Sys.islinux()
#    requiredFinalStates = [0.06377455426382193, 0.06452812521378912, 0.1394602158004863, 0.0003510332026172724, -0.00029575375397251983, -5.531647851029789e-6, -0.9844724960963392, -0.8294680298421877, 1.6726513825863847, 0.00317058514594142, 0.0031672104289462754, 0.0012711469334136117]
#else
    requiredFinalStates = missing  # unreproducible results on GitHub
#end
simulate!(pcmBubbleFunnel, stopTime=stopTime, tolerance=tolerance, dtmax=dtmax, log=true, logTiming=false, requiredFinalStates_rtol=100*tolerance, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pcmBubbleFunnel, ["bubble.translation"], figure=1)
plot(pcmBubbleFunnel, ["force.numBVChecks", "force.numPenChecks", "force.numContactElements", "force.contactPatchArea"], figure=2)
plot(pcmBubbleFunnel, ["force.forceVector", "force.torqueVector"], figure=3)

end
