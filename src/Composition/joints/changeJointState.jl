
# a free moving object will be gripped
function changeJointFromFreeMotionToFix!(obj)
    obj.joint = fixedJoint
#    obj.parent.hasChildJoint = true
end

# a fixed object will be released
function changeJointFromFixToFreeMotion!(obj)
    r = obj.r_rel
    q = Modia3D.from_R(obj.R_rel)
    v = Modia3D.resolve2(obj.parent.R_abs,
        (obj.v0 - obj.parent.v0) )
    w = Modia3D.resolve2(obj.parent.R_abs,
        (obj.w - obj.parent.w) )
    #=
    println("r ", r)
    println("q ", q)
    println("v ", v)
    println("w ", w)
    println("obj.v0 ", obj.v0)
    println("obj.parent.v0 ", obj.parent.v0)
    println("obj.w ", obj.w)
    println("obj.parent.w ", obj.parent.w)
    =#
    obj.joint = FreeMotion(obj;
                r_start = r,
                q_start = q,
                v_start = v,
                w_start = w)
#    obj.parent.hasChildJoint = true

# resolve2 R*v
# resolve1 R'*v

#    obj.joint = FreeMotion(obj;
#                    r_start = obj.r_abs,
#                    q_start = Modia3D.from_R(obj.R_abs) )
                   # v_start=v_start,
                   # w_start=w_start) #, v_start = v_start, q_start = obj.R_abs,

end
