module Simulate_YouBot

using  Modia3D
import Modia3D.ModiaMath
using  Modia3D.ModiaMath.Unitful

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.8)    # material of SolidFileMesh

base_frame_obj           = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/base_frame.obj")
plate_obj                = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/plate.obj")
front_right_wheel_obj    = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/front-right_wheel.obj")
front_left_wheel_obj     = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/front-left_wheel.obj")
back_right_wheel_obj     = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/back-right_wheel.obj")
back_left_wheel_obj      = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/back-left_wheel.obj")

arm_base_frame_obj       = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_base_frame.obj")
arm_joint_1_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_1.obj")
arm_joint_2_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_2.obj")
arm_joint_3_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_3.obj")
arm_joint_4_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_4.obj")
arm_joint_5_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_5.obj")
gripper_base_frame_obj   = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_base_frame.obj")
gripper_left_finger_obj  = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_left_finger.obj")
gripper_right_finger_obj = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_right_finger.obj")


# Damper
@forceElement Damper(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
    damper.tau.value = -damper.d*damper.w.value
end


# Controller
@forceElement P_PI_Controller(; k1=1.0, k2=10.0, T2=0.01, freqHz=0.05, A=1.0) begin
    PI_x    = ModiaMath.RealScalar("PI_x"   , start=0.0, info="State of PI controller", numericType=ModiaMath.XD_EXP)
    PI_derx = ModiaMath.RealScalar("PI_derx", start=0.0, info="= der(PI_x)"           , numericType=ModiaMath.DER_XD_EXP, integral=PI_x)
    sine_y  = ModiaMath.RealScalar("sine_y" , start=0.0, info="= sin(2*pi*f*time)"    , numericType=ModiaMath.WR)
    phi = ModiaMath.RealScalar("phi", causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(c::P_PI_Controller, sim::ModiaMath.SimulationState)
    c.sine_y.value  = c.A*sin(2*pi*c.freqHz*sim.time)
    gain_y          = c.k1*(c.sine_y.value - c.phi.value)
    PI_u            = gain_y - c.w.value
    c.PI_derx.value = PI_u/c.T2
    c.tau.value = c.k2*(c.PI_x.value + PI_u)
end


@signal Sine(;A=1.5, freqHz = 0.5, offset=0.0) begin
   y = ModiaMath.RealScalar(causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::Sine, sim::ModiaMath.SimulationState)
    signal.y.value  = signal.offset + signal.A*sin(2*pi*signal.freqHz*sim.time)
end


@assembly Link(frame_a; axis=3, fileMesh="", m=0.0, r_rev_b=[0.0, 0.0, 0.0], R_a_rev=ModiaMath.NullRotation,
               d=0.0, k1=1.0, k2=10.0, T2=0.01, freqHz=0.1, A=0.1, offset=0.0) begin
    rev_a   = Object3D(frame_a, R=R_a_rev, visualizeFrame=false)
    rev_b   = Object3D(Solid(SolidFileMesh(fileMesh),m,vmat1))
    rev     = Revolute(rev_a, rev_b, phi_start=offset)
    frame_b = Object3D(rev_b, r=r_rev_b, visualizeFrame=false)

#=
    # Kinematic movement of joint
    sig         = Sine(A=A,freqHz=freqHz,offset=offset)
    sig_adaptor = Modia3D.SignalToFlangeAngle(sig.y)
    Modia3D.connect(sig_adaptor, rev)
=#

#=
    # Damping in the joint
    damper         = Damper(d=d)
    damper_adaptor = Modia3D.AdaptorForceElementToFlange(w=damper.w, tau=damper.tau)
    Modia3D.connect(damper_adaptor, rev)
=#

#=
    # P-PI controller in the joint
    controller         = P_PI_Controller(k1=k1, k2=k2, T2=T2, freqHz=freqHz, A=A)
    controller_adaptor = Modia3D.AdaptorForceElementToFlange(phi=controller.phi, w=controller.w, tau=controller.tau)
    Modia3D.connect(controller_adaptor, rev)
=#
end

@assembly Gripper(frame_a) begin
    frame1                   = Object3D(frame_a, R=ModiaMath.rot3(-180u"°"))
    gripper_base_frame       = Object3D(frame1,Solid(SolidFileMesh(gripper_base_frame_obj),0.199,vmat1), visualizeFrame=true)
    gripper_left_finger      = Object3D(gripper_base_frame, Solid(SolidFileMesh(gripper_left_finger_obj),0.010,vmat1), r=[0, 0.0082,0])
    gripper_right_finger     = Object3D(gripper_base_frame, Solid(SolidFileMesh(gripper_right_finger_obj),0.010,vmat1), r=[0,-0.0082,0])
end

@assembly Base(world) begin
    base_frame        = Object3D(world, Solid(SolidFileMesh(base_frame_obj),19.803,vmat1), r=[0,0,0.084])
    plate             = Object3D(base_frame, Solid(SolidFileMesh(plate_obj),MassProperties(m=2.397),vmat1), r=[-0.159,0,0.046])
    front_right_wheel = Object3D(base_frame, Solid(SolidFileMesh(front_right_wheel_obj),1.4,vmat1), r=[0.228, -0.158, -0.034])
    front_left_wheel  = Object3D(base_frame, Solid(SolidFileMesh(front_left_wheel_obj),1.4,vmat1) , r=[0.228, 0.158, -0.034])
    back_right_wheel  = Object3D(base_frame, Solid(SolidFileMesh(back_right_wheel_obj),1.4,vmat1) , r=[-0.228, -0.158, -0.034])
    back_left_wheel   = Object3D(base_frame, Solid(SolidFileMesh(back_left_wheel_obj),1.4,vmat1)  , r=[-0.228, 0.158, -0.034])
end

@assembly YouBot begin
    world           = Object3D(visualizeFrame=false)
    base            = Base(world)
    arm_base_frame  = Object3D(base.base_frame, Solid(SolidFileMesh(arm_base_frame_obj), 0.961, vmat1), r=[0.143, 0.0, 0.046], visualizeFrame=false)
    armBase_b       = Object3D(arm_base_frame, r=[0.024, 0, 0.115])
    link1   = Link(armBase_b    , fileMesh=arm_joint_1_obj, m=1.390, r_rev_b=[0.033,0,0], R_a_rev = ModiaMath.rot1(180u"°"))
    link2   = Link(link1.frame_b, fileMesh=arm_joint_2_obj, m=1.318, r_rev_b=[0.155,0,0], R_a_rev = ModiaMath.rot123(90u"°", 0.0,-90u"°"))
    link3   = Link(link2.frame_b, fileMesh=arm_joint_3_obj, m=0.821, r_rev_b=[0,0.135,0], R_a_rev = ModiaMath.rot3(-90u"°"), offset=pi/2, A=0.0)
    link4   = Link(link3.frame_b, fileMesh=arm_joint_4_obj, m=0.769, r_rev_b=[0,0.11316,0])
    link5   = Link(link4.frame_b, fileMesh=arm_joint_5_obj, m=0.687, r_rev_b=[0,0,0.05716], R_a_rev = ModiaMath.rot1(-90u"°"))
    gripper = Gripper(link5.frame_b)
end

gravField = UniformGravityField(g=9.81, n=[0,0,-1])
youBot    = YouBot(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=true,
                                             defaultFrameLength=0.1, enableContactDetection=false))

println("... arm_joint_1: massProperties = ", youBot.link1.rev_b.data.massProperties)

#Modia3D.visualizeAssembly!(youBot)
#model  = Modia3D.SimulationModel( youBot, analysis=ModiaMath.KinematicAnalysis )
model  = Modia3D.SimulationModel( youBot )
result = ModiaMath.simulate!(model, stopTime=0.07, log=true, tolerance=1e-4)
end