# function computeInertiaTensorForTwoBodies!
# computes a common mass, inertia tensor and center of mass of all object3Ds of a super object
# I_steiner = E*I*E' + m * skew(rCM)' * skew(rCM)
function computeInertiaTensorForTwoBodies!(rootSuperObj::Object3D, actualMassSuperObject::Array{Object3D,1})
  if length(actualMassSuperObject) > 0
    for i=1:length(actualMassSuperObject)
      obj = actualMassSuperObject[i]
      if rootSuperObj != obj
        m_child = obj.data.massProperties.m
        m_root  = rootSuperObj.massProperties.m
        rCM_child_old = obj.data.massProperties.rCM
        R_child = obj.R_rel
        r_child = obj.r_rel
        rCM_root = rootSuperObj.massProperties.rCM
        I_child = obj.data.massProperties.I
        I_root  = rootSuperObj.massProperties.I

        # new mass
        m_new = m_root + m_child

        # new center of mass: root to child
        rCM_child_new = r_child + R_child' * rCM_child_old

        # new center of mass of both mass
        @assert(m_new > 0.0)
        rCM_new = (rCM_child_new * m_child + rCM_root * m_root)/m_new

        # new I_child_steiner and I_root_steiner & I_new
        I_child_steiner = ModiaMath.NullRotation*I_child*ModiaMath.NullRotation' + m_child * ModiaMath.skew(rCM_child_new)' * ModiaMath.skew(rCM_child_new)
        I_root_steiner  =          I_root          + m_root  * ModiaMath.skew(rCM_root)' * ModiaMath.skew(rCM_root)
        I_sum = I_child_steiner + I_root_steiner
        I_new = I_sum - m_new * ModiaMath.skew(rCM_new)' * ModiaMath.skew(rCM_new)

        # assign mass properties to rootSuperObj
        rootSuperObj.massProperties.m = m_new
        rootSuperObj.massProperties.rCM = rCM_new
        rootSuperObj.massProperties.I = I_new


        # other way to compute inertia tensor
        #=
        a1 = rCM_new - rCM_child_new
        a2 = rCM_root - rCM_new
        IchildNew =  I_child + m_child * ModiaMath.skew(a1)' * ModiaMath.skew(a1)
        IrootNew  =  I_root  + m_root  * ModiaMath.skew(a2)' * ModiaMath.skew(a2)
        InewNew = IchildNew + IrootNew
        =#
      end
    end
  else
    rootSuperObj.massProperties = nothing
  end
  return nothing
end

# assigns mass properties to the root of a super object
# differe betweeen root objects has already mass properties or not.
function initializeMassComputation!(scene::Scene)
  if scene.initMassComp != true
    superObjs = scene.superObjs
    buffer    = scene.buffer

    for i = 1:length(superObjs)
      rootSuperObj = buffer[i]
      if dataHasMass(rootSuperObj)
        rootSuperObj.massProperties     = Solids.InternalMassProperties()
        rootSuperObj.massProperties.m   = rootSuperObj.data.massProperties.m
        rootSuperObj.massProperties.rCM = rootSuperObj.data.massProperties.rCM
        rootSuperObj.massProperties.I   = rootSuperObj.data.massProperties.I
        computeInertiaTensorForTwoBodies!(rootSuperObj, superObjs[i].superObjMass.superObj)
      else
        if length(superObjs[i].superObjMass.superObj) > 0
          rootSuperObj.massProperties = Solids.InternalMassProperties()
          computeInertiaTensorForTwoBodies!(rootSuperObj, superObjs[i].superObjMass.superObj)
    end; end; end
    scene.initMassComp = true
  end
  return nothing
end
