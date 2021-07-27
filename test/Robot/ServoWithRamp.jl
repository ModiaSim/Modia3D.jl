module ServoWithRamp

using ModiaLang
using Unitful

# ModiaLang models
include("$(ModiaLang.path)/models/Blocks.jl")
include("$(ModiaLang.path)/models/Electric.jl")
include("$(ModiaLang.path)/models/Rotational.jl")

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
        (PI.y, refTorque)
    ]
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
    #gearRatio = 1.0,
    refLoadAngle = input,
    flange = Flange,
    ppi   = Controller, # | Map(gearRatio=:gearRatio),
    drive = Drive, # | Map(gearRatio=:gearRatio),

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

servoParameters = Map(gearRato = 156.0,
                      ppi = Map(
                          gainOuter = Map(k=156.0),
                          gainInner = Map(k=50.0),
                          PI        = Map(k=0.1, T=1.0)
                      ),
                      drive = Map(
                          motorInertia = Map(J= 0.0000135 + 0.000000409),
                          idealGear = Map(ratio=156.0)
                      )
                  )

TestServo = Model(
    ramp  = Ramp | Map(duration=1.18u"s", height=2.95),
    servo = Servo | servoParameters,
    load  = Inertia | Map(J=0.32u"kg*m^2"),
    equations =:[load.flange_b.tau = 0u"N*m"],
    connect = :[
        (ramp.y, servo.refLoadAngle)
        (servo.flange, load.flange_a)
    ]
)

testServo = @instantiateModel(TestServo, unitless=true, logCode=false, log=false)

stopTime = 4.0
tolerance = 1e-6
requiredFinalStates = [-0.004445925511355035, 2.9499994183801284, 5.816286501116783e-7]
simulate!(testServo, stopTime=stopTime, tolerance=tolerance, log=true,logStates=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plotVariables = [("ramp.y", "load.phi"); "load.w"; "servo.ppi.PI.x"; "servo.ppi.refTorque"]
plot(testServo, plotVariables, figure=1)

end
