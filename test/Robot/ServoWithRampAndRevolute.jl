module ServoWithRampAndRevolute

using ModiaLang
using Unitful
import Modia3D

# ModiaLang models
include("$(ModiaLang.path)/models/Blocks.jl")
include("$(ModiaLang.path)/models/Electric.jl")
include("$(ModiaLang.path)/models/Rotational.jl")

import Modia3D
using  Modia3D.ModiaInterface

Controller = Model(
    # Interface
    refLoadAngle     = input,
    actualMotorAngle = input,
    actualMotorSpeed = input,
    refTorque        = output,
    # Components
    gainOuter     = Gain,
    angleFeedback = Feedback,
    gainInner     = Gain,
    speedFeedback = Feedback,
    PI = PI,

    connect = :[
        (refLoadAngle, gainOuter.u)
        (gainOuter.y, angleFeedback.u1)
        (actualMotorAngle, angleFeedback.u2)
        (angleFeedback.y, gainInner.u)
        (gainInner.y, speedFeedback.u1)
        (actualMotorSpeed, speedFeedback.u2)
        (speedFeedback.y, PI.u)
        (PI.y, refTorque)]
)

Drive = Model(
    # Interface
    refTorque   = input,
    actualSpeed = output,
    actualAngle = output,
    flange      = Flange,
    # Components
    torque       = UnitlessTorque,
    speedSensor  = UnitlessSpeedSensor,
    angleSensor  = UnitlessAngleSensor,
    motorInertia = Inertia,
    idealGear    = IdealGear,

    connect = :[
        (refTorque, torque.tau)
        (torque.flange, speedSensor.flange, angleSensor.flange, motorInertia.flange_a)
        (motorInertia.flange_b, idealGear.flange_a)
        (idealGear.flange_b, flange)
        (speedSensor.w, actualSpeed)
        (angleSensor.phi, actualAngle)
        ]
)

Servo = Model(
    refLoadAngle = input,
    flange       = Flange,
    ppi          = Controller,
    drive        = Drive,

    connect = :[
        # inputs of ppi
        (refLoadAngle, ppi.refLoadAngle)
        (ppi.actualMotorSpeed, drive.actualSpeed)
        (ppi.actualMotorAngle, drive.actualAngle)
        # output of ppi --> input of motor
        (ppi.refTorque, drive.refTorque)
        # Flange of drive
        (drive.flange, flange)
        ]
)

arm_joint_1_obj = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_1.obj")

m = 2
axis = 3
vmat1 = VisualMaterial(color="LightBlue", transparency=0.5)
m1=1.390
translation1 =[0.033,0,0]
rotation1 = [180u"°", 0, 0]

k1=50.0
k2=0.1
T2=1.0
gearRatio=156.0
motorInertia = (0.0000135 + 0.000000409)*u"kg*m^2"
J1 = 0.32

servoParameters = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio),
                          gainInner = Map(k=k1),
                          PI        = Map(k=k2, T=T2)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia),
                          idealGear = Map(ratio=gearRatio)
                      )
                  )

TestServo = Model(
    world = Object3D(feature=Scene()),
    body  = Object3D(feature = Solid(
            shape = FileMesh(filename = arm_joint_1_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m1), visualMaterial=vmat1)),
    obj2  = Object3D(parent = :body, translation = translation1, rotation = rotation1),

    rev = RevoluteWithFlange(obj1 = :world, obj2 = :obj2, axis=axis, phi = Var(init = 0.0), w=Var(init=0.0)),

    ramp  = Ramp | Map(duration=1.18u"s", height=2.95),
    servo = Servo | servoParameters,
    connect = :[
        (ramp.y, servo.refLoadAngle)
        (servo.flange, rev.flange)
        ]
)

servo = @instantiateModel(buildModia3D(TestServo), unitless=true, logCode=true, log=true)

stopTime = 4.0
tolerance = 1e-6
requiredFinalStates = [2.949999509178657, 4.908194611512363e-7, 0.02159751818019237]
simulate!(servo, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plotVariables = [("ramp.y", "rev.phi"); "rev.w"; "servo.ppi.PI.x"]
plot(servo, plotVariables, figure=1)

end
