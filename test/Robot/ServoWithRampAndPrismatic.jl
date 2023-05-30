module ServoWithRampAndPrismatic

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Translational.jl")



ControllerPrism = Model(
    # Interface
    refLoadForce     = input,
    actualMotorPos   = input,
    actualMotorSpeed = input,
    refForce         = output,
    # Components
    gainOuter     = Gain,
    posFeedback   = Feedback,
    gainInner     = Gain,
    speedFeedback = Feedback,
    PI = PI,

    connect = :[
        (refLoadForce, gainOuter.u)
        (gainOuter.y,  posFeedback.u1)
        (actualMotorPos, posFeedback.u2)
        (posFeedback.y,  gainInner.u)
        (gainInner.y,    speedFeedback.u1)
        (actualMotorSpeed, speedFeedback.u2)
        (speedFeedback.y,  PI.u)
        (PI.y, refForce)
    ]
)

DrivePrism = Model(
    # Interface
    refForce    = input,
    actualSpeed = output,
    actualPos   = output,
    flange      = TranslationalFlange,
    # Components
    force        = UnitlessForce,
    speedSensor  = UnitlessVelocitySensor,
    posSensor    = UnitlessPositionSensor,
#    motorInertia = Inertia,
#    idealGear    = IdealGear,

    connect = :[
        (refForce, force.f)
        (force.flange, speedSensor.flange, posSensor.flange, flange)
        #(motorInertia.flange_b, idealGear.flange_a)
        #(idealGear.flange_b, flange)
        (speedSensor.v, actualSpeed)
        (posSensor.s, actualPos)
    ]
)


ServoPrism = Model(
    refLoadForce = input,
    flange       = TranslationalFlange,
    ppi          = ControllerPrism,
    drive        = DrivePrism,

    connect = :[
        # inputs of ppi
        (refLoadForce, ppi.refLoadForce)
        (ppi.actualMotorSpeed, drive.actualSpeed)
        (ppi.actualMotorPos,   drive.actualPos)
        # output of ppi --> input of motor
        (ppi.refForce, drive.refForce)
        # Flange of drive
        (drive.flange, flange)
    ]
)

arm_joint_1_obj = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_1.obj")

m = 2
axis = 3
vmat1 = VisualMaterial(color="LightBlue", transparency=0.5)
m1=1.390
translation1 = [0.033, 0, 0]
rotation1 = [180u"°", 0, 0]

k1 = 50.0
k2 = 0.1
T2 = 1.0
gearRatio = 156.0
motorInertia = (0.0000135 + 0.000000409)*u"kg*m^2"
J1 = 0.32

servoParameters = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio),
                          gainInner = Map(k=k1),
                          PI        = Map(k=k2, T=T2)
                      ),
                      drive = Map(
                          #motorInertia = Map(J=motorInertia),
                          #idealGear = Map(ratio=gearRatio)
                      )
                  )

TestServo = Model3D(
    world = Object3D(feature=Scene(nominalLength=0.2)),
    body  = Object3D(feature=Solid(shape=FileMesh(filename=arm_joint_1_obj),
                                   massProperties=MassPropertiesFromShapeAndMass(mass=m1),
                                   visualMaterial=vmat1)),
    obj2  = Object3D(parent=:body,
                     translation=translation1,
                     rotation=rotation1),

    rev = PrismaticWithFlange(obj1=:world, obj2=:obj2, axis=axis, s=Var(init=0.0), v=Var(init=0.0)),

    ramp  = Ramp | Map(duration=1.18u"s", height=2.95),
    servo = ServoPrism | servoParameters,
    connect = :[
        (ramp.y, servo.refLoadForce)
        (servo.flange, rev.flange)
    ]
)

servo = @instantiateModel(TestServo, unitless=true, logCode=false, log=false)

stopTime = 4.0
tolerance = 1e-6
requiredFinalStates = [-269.9911219480055, 1594.1933857866297, 25436.042732813803]
simulate!(servo, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plotVariables = [("ramp.y", "rev.s"); "rev.v"; "servo.ppi.PI.x"; "servo.ppi.refForce"]
plot(servo, plotVariables, figure=1)

end
