# a free moving object is fixed to his parent (gripping)
function changeJointFromFreeMotionToFix!(obj1::Object3D{F}, obj2::Object3D{F}) where F <: Modia3D.VarFloatType
    FixInternal{Float64}(child=obj2) #fixedJoint
    obj2.fixedToParent = true
    return nothing
end

# a fixed object is allowed to move freely (release)
function changeJointFromFixToFreeMotion!(obj1::Object3D{F}, obj2::Object3D{F}) where F <: Modia3D.VarFloatType

    FreeMotion{Float64}(; obj1=obj1, obj2=obj2, path=obj2.path, r=obj2.r_abs, rot=Modia3D.rot123fromR(obj2.R_abs), v=obj2.v0, w=obj2.w, hiddenStates=true, wResolvedInParent=true)

    obj2.fixedToParent = false
    return nothing
end
