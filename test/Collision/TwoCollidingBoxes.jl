module TwoCollidingBoxes

using Modia

vmat1 = VisualMaterial(color="Green", transparency=0.6)
vmat2 = VisualMaterial(color="Red"  , transparency=0.6)
cmat = "Steel"

collidingBoxes = Model(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[-1, 0, 0]), mprTolerance = 1.0e-9,
                                          enableContactDetection=true)),

    fixedBox = Object3D(parent=:world,
                        translation=[-3.0, 0.0, -2.5],
                        feature=Solid(shape=Box(lengthX=3.0, lengthY=2.0, lengthZ=5.0),
                                      solidMaterial="Aluminium",
                                      visualMaterial=vmat1,
                                      contactMaterial=cmat,
                                      collision=true,
                                      collisionSmoothingRadius=0.001)),

    movingBox = Object3D(feature=Solid(shape=Box(lengthX=0.5, lengthY=0.5, lengthZ=0.5),
                                       solidMaterial="Aluminium",
                                       visualMaterial=vmat2,
                                       contactMaterial=cmat,
                                       collision=true,
                                       collisionSmoothingRadius=0.001)),
    movingBoxJoint = FreeMotion(obj1=:world, obj2=:movingBox,
                                r=Var(init=[0.0, 0.0, 0.0]),
                                rot=Var(init=[pi/2, 0.0, 0.0]))
)

twoCollidingBoxes = @instantiateModel(buildModia3D(collidingBoxes), unitless=true, log=false, logStateSelection=false, logCode=false)


stopTime = 2.0
tolerance = 1e-6
interval = 0.001
if Sys.iswindows()
    requiredFinalStates = [-1.2407908396727925, -2.3572283965664513e-6, 0.06989496052068897, 0.7494258087005188, -4.823581569286559e-6, 0.5595405151042986, 1.570820906936134, 4.473612187266959e-6, 0.19274155157960468, 1.4898946994571155e-5, 2.2913495764710882e-5, 1.6053672226621267]
else
    requiredFinalStates = [-1.2487260400917095, 5.173797710712821e-7, 0.06612781902011293, 0.8277764440525651, 1.9424683436941025e-6, 0.5711299184911467, 1.570795378312463, -1.1347368868523844e-7, 0.18165762561640364, 1.0647188881454766e-5, 4.621631971845376e-6, 1.6455580065333635]
end
simulate!(twoCollidingBoxes, stopTime=stopTime, tolerance=tolerance, interval=interval, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(twoCollidingBoxes, ["movingBoxJoint.r" "movingBoxJoint.rot"; "movingBoxJoint.v" "movingBoxJoint.w"], figure=1)

end
