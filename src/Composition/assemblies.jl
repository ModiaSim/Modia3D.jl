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

@assembly ContactBox(parent::Object3D;
                     scale::AbstractVector = [1.0, 1.0, 1.0],
                     rsmall::Float64=0.001,
                     fixed::Bool = true,
                     r::AbstractVector = ModiaMath.ZeroVector3D,
                     R::Union{ModiaMath.RotationMatrix,NOTHING} = nothing,
                     q::Union{ModiaMath.Quaternion,NOTHING} = nothing,
                     v_start::AbstractVector = ModiaMath.ZeroVector3D,
                     w_start::AbstractVector = ModiaMath.ZeroVector3D,
                     visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited,
                     massProperties::Union{Solids.MassProperties,Number,AbstractString,Solids.SolidMaterial,NOTHING} = nothing,
                     material::Union{Graphics.Material,NOTHING} = Graphics.Material(),
                     contactMaterial::String = "",
                     ) begin
    #box    = Modia3D.Object3D(parent, Modia3D.Solid(Modia3D.SolidBox(scale[1]-2*rsmall, scale[2]-2*rsmall, scale[3]-2*rsmall; rsmall=rsmall),
    box    = Modia3D.Object3D(parent, Modia3D.Solid(Modia3D.SolidBox(scale[1]-2*rsmall, scale[2]-2*rsmall, scale[3]-2*rsmall; rsmall=rsmall),
                              massProperties,material; contactMaterial=contactMaterial),
                              fixed=fixed, r=r, R=R, q=q, v_start=v_start, w_start=w_start, visualizeFrame=visualizeFrame)
    sphere = Modia3D.Solid(Modia3D.SolidSphere(2*rsmall), nothing, Modia3D.Material(color="Red"); contactMaterial=contactMaterial)
    ball1  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall,  scale[2]/2-rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    ball2  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall,  scale[2]/2-rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    ball3  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall, -scale[2]/2+rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    ball4  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall, -scale[2]/2+rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    ball5  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall,  scale[2]/2-rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
    ball6  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall,  scale[2]/2-rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
    ball7  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall, -scale[2]/2+rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
    ball8  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall, -scale[2]/2+rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
end

@assembly ContactBox2(parent::Object3D;
                     scale::AbstractVector = [1.0, 1.0, 1.0],
                     rsmall::Float64=0.001,
                     fixed::Bool = true,
                     r::AbstractVector = ModiaMath.ZeroVector3D,
                     R::Union{ModiaMath.RotationMatrix,NOTHING} = nothing,
                     q::Union{ModiaMath.Quaternion,NOTHING} = nothing,
                     v_start::AbstractVector = ModiaMath.ZeroVector3D,
                     w_start::AbstractVector = ModiaMath.ZeroVector3D,
                     visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited,
                     massProperties::Union{Solids.MassProperties,Number,AbstractString,Solids.SolidMaterial,NOTHING} = nothing,
                     material::Union{Graphics.Material,NOTHING} = Graphics.Material(),
                     contactMaterial::String = "",
                     ) begin
    #box    = Modia3D.Object3D(parent, Modia3D.Solid(Modia3D.SolidBox(scale[1]-2*rsmall, scale[2]-2*rsmall, scale[3]-2*rsmall; rsmall=rsmall),
    box    = Modia3D.Object3D(parent, Modia3D.Solid(Modia3D.SolidBox(scale[1]-2*rsmall, scale[2]-2*rsmall, scale[3]-2*rsmall; rsmall=rsmall),
                              massProperties,material; contactMaterial=contactMaterial),
                              fixed=fixed, r=r, R=R, q=q, v_start=v_start, w_start=w_start, visualizeFrame=visualizeFrame)
    sphere = Modia3D.Solid(Modia3D.SolidSphere(2*rsmall), nothing, Modia3D.Material(color="Red"); contactMaterial=contactMaterial)
    #ball1  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall,  scale[2]/2-rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    #ball2  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall,  scale[2]/2-rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    #ball3  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall, -scale[2]/2+rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    #ball4  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall, -scale[2]/2+rsmall,  scale[3]/2-rsmall], visualizeFrame=false)
    ball5  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall,  scale[2]/2-rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
    ball6  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall,  scale[2]/2-rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
    ball7  = Modia3D.Object3D(box, sphere, fixed=true, r=[ scale[1]/2-rsmall, -scale[2]/2+rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
    ball8  = Modia3D.Object3D(box, sphere, fixed=true, r=[-scale[1]/2+rsmall, -scale[2]/2+rsmall, -scale[3]/2+rsmall], visualizeFrame=false)
end
