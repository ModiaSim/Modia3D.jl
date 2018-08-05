# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

@assembly Part(data::Modia3D.AbstractObject3Ddata, rFrames::AbstractVector; visualizeFrames::Union{Bool,Modia3D.Ternary}=Modia3D.Inherited) begin
   frame0 = Modia3D.Object3D(data)
   frames = [ Modia3D.Object3D(frame0; r = rFrame, visualizeFrame = visualizeFrames) for rFrame in rFrames ]
end

@assembly BodyWithTwoFrames(data::Modia3D.AbstractObject3Ddata, r_ab::AbstractVector; visualizeFrames::Union{Bool,Modia3D.Ternary}=Modia3D.Inherited) begin
   frame_a = Modia3D.Object3D(data, visualizeFrame = visualizeFrames)
   frame_b = Modia3D.Object3D(frame_a; r = r_ab, visualizeFrame = visualizeFrames)
end
