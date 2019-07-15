module Simulate_Robot

using  Modia3D
import Modia3D.ModiaMath
using  Modia3D.ModiaMath.Unitful

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh

file_arm1   = joinpath(Modia3D.path, "objects/robot/arm1.obj")
file_arm2   = joinpath(Modia3D.path, "objects/robot/arm2.obj")
file_arm3   = joinpath(Modia3D.path, "objects/robot/arm3.obj")
file_hand   = joinpath(Modia3D.path, "objects/robot/hand.obj")
file_base   = joinpath(Modia3D.path, "objects/robot/base.obj")
file_turret = joinpath(Modia3D.path, "objects/robot/turret.obj")

r_offset = [7.52, -0.23, 0]
R_offset = ModiaMath.rot1(90u"°")
r1       = [0, 0, 0.9]
r2       = [0.8, 0, 0.7]
r3       = [0, -0.65, 2.0]
r4       = [0, 0, 1.0]

R2_offset = ModiaMath.rot2(60u"°")
R3_offset = ModiaMath.rot2(-170u"°")

@assembly Part(;fileMesh="", r_ab=[1.0,0,0], r_pre=[0,0,0], R_pre=ModiaMath.NullRotation) begin
    frame_a   = Object3D(visualizeFrame=true)
    frame_a2  = Object3D(frame_a, R=R_pre)
    frame_b   = Object3D(frame_a, r=r_ab, visualizeFrame=false)
    frame_vis = Object3D(frame_a2, Solid(SolidFileMesh(fileMesh), nothing, vmat1),
                         r=r_offset-r_pre, R=R_offset)
end

@assembly Robot begin
    world    = Object3D(visualizeFrame=true)
    base_a   = Object3D(world, r=r1, visualizeFrame=true)
    base     = Object3D(base_a, Solid(SolidFileMesh(file_base), nothing, vmat1); r=r_offset-r1, R=R_offset)
    turret   = Part(fileMesh=file_turret, r_ab=r2, r_pre=r1)
    rev1     = Revolute(base_a,turret.frame_a; axis=3, phi_start=0)
    arm1     = Part(fileMesh=file_arm1, r_ab=r3, r_pre=r1+r2)   # R_pre=R2_offset
    rev2     = Revolute(turret.frame_b, arm1.frame_a; axis=2, phi_start=0)
    arm2     = Part(fileMesh=file_arm2, r_ab=r4, r_pre=r1+r2+r3)  # R_pre=R3_offset
    rev3     = Revolute(arm1.frame_b, arm2.frame_a, axis=2, phi_start=0)

   # arm2    = Object3D(turret, Solid(SolidFileMesh(file_arm2)  , nothing, vmat1))
   #arm3    = Object3D(turret, Solid(SolidFileMesh(file_arm3)  , nothing, vmat1))
   # hand    = Object3D(turret, Solid(SolidFileMesh(file_hand)  , nothing, vmat1))

end

robot = Robot(sceneOptions=SceneOptions(visualizeFrames=false,defaultFrameLength=0.7,enableContactDetection=false) )

Modia3D.visualizeAssembly!(robot)
end