module MobileWithLogTiming

# For profiling
using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")


const depthMax = 6
const enableVisualization = false

const rodLength       = 0.1
const rodDiameter     = 0.05*rodLength
const rodMaterial     = "DryWood"
const rodColor        = VisualMaterial(color="Brown", transparency=0.5)
const rodTranslation1 = [0.0,  rodLength/2, 0.0]
const rodTranslation2 = [0.0, -rodLength/2, 0.0]

const sphereDiameter  = 0.3*rodLength
const sphereMaterial  = "DryWood"
const sphereColor     = VisualMaterial(color="Blue")

const barWidth       = 0.3*rodLength
const barThickness   = 0.3*rodLength
const barMaterial    = "DryWood"
const barColor       = VisualMaterial(color="DeepSkyBlue3", transparency=0.5)
const leaveBarLength = 0.8*rodLength

const revDiameter = rodDiameter
const revLength   = 1.2*barThickness
const revColor    = VisualMaterial(color="Red")

const damping = 0.006

Rod = Model(
    frame0 = Object3D(feature = Solid(shape = Cylinder(axis=2, length=rodLength, diameter=rodDiameter),
                                    solidMaterial  = rodMaterial,
                                    visualMaterial = rodColor)),
    frame1 = Object3D(parent = :frame0, translation = rodTranslation1),
    frame2 = Object3D(parent = :frame0, translation = rodTranslation2)
)
Bar = Model(
    L = missing,
    frame0 = Object3D(feature = Solid(shape = Beam(axis = 1, length = :L, width=barWidth, thickness=barThickness),
                                    solidMaterial  = barMaterial,
                                    visualMaterial = barColor)),
    frame1 = Object3D(parent = :frame0, translation = :[-L/2, 0.0, 0.0]),
    frame2 = Object3D(parent = :frame0, translation = :[ L/2, 0.0, 0.0])
)
RevoluteWithDamping(;obj1, obj2, axis=3, phi_start=0.0) = Model(
    rev     = RevoluteWithFlange(obj1=obj1, obj2=obj2, axis=axis, phi=Var(init=phi_start)),
    cyl     = Object3D(parent  = obj1,
                    feature = Visual(shape = Cylinder(axis=axis, diameter=revDiameter, length=revLength),
                                        visualMaterial = revColor)),
    damper  = Damper | Map(d=damping),
    fixed   = Fixed,
    connect = :[(damper.flange_b, rev.flange),
                (damper.flange_a, fixed.flange)]
)
function barLength(depth)
    if depth<=2
        return leaveBarLength
    end
    return 2*barLength(depth-1)
end
function createMobile(depth)
    if depth == 1
        Model(
            rod    = Rod,
            sphere = Object3D(parent = :(rod.frame0), translation = rodTranslation2,
                              feature = Solid(shape = Sphere(diameter=sphereDiameter),
                                              solidMaterial  = sphereMaterial,
                                              visualMaterial = sphereColor))
        )
    else
        Model(
            rod  = Rod,
            bar  = Bar | Map(L=barLength(depth)),
            sub1 = createMobile(depth-1),
            sub2 = createMobile(depth-1),
            rev0 = RevoluteWithDamping(obj1=:(rod.frame2), obj2=:(bar.frame0)),
            rev1 = RevoluteWithDamping(obj1=:(bar.frame1), obj2=:(sub1.rod.frame1)),
            rev2 = RevoluteWithDamping(obj1=:(bar.frame2), obj2=:(sub2.rod.frame1))
        )
    end
end
Mobile = Model3D(
    world      = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, -1, 0]),
                                        enableContactDetection=false,
                                        enableVisualization=enableVisualization)),
    worldFrame = Object3D(parent = :world, feature = Visual(shape = CoordinateSystem(length=rodLength))),
    top        = createMobile(depthMax),
    rev0       = RevoluteWithDamping(obj1 = :world, obj2 = :(top.rod.frame1), phi_start=0.2)
)

println("... @instantiateModel:")
@time mobile = @instantiateModel(Mobile, unitless=true, log=false, logDetails=false, logModel=false, logStateSelection=false,
                                 logCode=false, logExecution=true, logTiming=false, evaluateParameters=true)

const stopTime = 5.0
const tolerance = 1e-4
const requiredFinalStates = missing

println("... first simulation:")
@time simulate!(mobile, stopTime=stopTime, tolerance=tolerance, logTiming=false, requiredFinalStates=requiredFinalStates)

println("... second simulation:")
@time simulate!(mobile, stopTime=stopTime, tolerance=tolerance, logTiming=true, log=true, requiredFinalStates=requiredFinalStates)

#@usingModiaPlot
#plot(mobile, "rev0.rev.phi")

end
