module TwoCollidingBoxes

using ModiaLang
import Modia3D
using  Modia3D.ModiaInterface

vmat1 = Modia3D.VisualMaterial(color="Green", transparency=0.6)
vmat2 = Modia3D.VisualMaterial(color="Red"  , transparency=0.6)
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
    requiredFinalStates = [-1.2490615356331602, 8.281738975240017e-7, 0.06612504512432218, 0.8513146411757051, -1.4194206015750393e-5, 0.5773901055263853, 1.5707853269807777, -2.0214977445980814e-6, 0.18117838351596363, 0.00017232488403129057, -2.067321556783797e-5, 1.6569379339855361]
else
    requiredFinalStates = [-1.2491609001458825, 3.4171296735804487e-7, 0.06689208772957456, 0.8462494065123138, 1.6721279542955506e-6, 0.5829025911027681, 1.5707788581459448, -3.2574807995825715e-6, 0.183216453113141, -0.00012373469585807555, -3.169934525839379e-5, 1.6744038834216513]
end
simulate!(twoCollidingBoxes, stopTime=stopTime, tolerance=tolerance, interval=interval, log=true, logStates=true, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(twoCollidingBoxes, ["movingBoxJoint.r" "movingBoxJoint.rot"; "movingBoxJoint.v" "movingBoxJoint.w"], figure=1)

end
