function computeInertiaTensorForTwoBodies!(rootSuperObj::Object3D, actualMassSuperObject::Array{Object3D,1})
  for obj in actualMassSuperObject
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
    I_child_steiner = R_child'*I_child*R_child + m_child * ModiaMath.skew(rCM_child_new)' * ModiaMath.skew(rCM_child_new)
    I_root_steiner  =          I_root          + m_root  * ModiaMath.skew(rCM_root)' * ModiaMath.skew(rCM_root)
    I_sum = I_child_steiner + I_root_steiner
    I_new = I_sum - m_new * ModiaMath.skew(rCM_new)' * ModiaMath.skew(rCM_new)

    # assign mass properties to rootSuperObj
    rootSuperObj.massProperties.m = m_new
    rootSuperObj.massProperties.rCM = rCM_new
    rootSuperObj.massProperties.I = I_new
  end
  return nothing
end


function initializeMassComputation!(scene::Scene)
  println("bis zu initializeMassComputation funkt es noch")
  superObjs = scene.superObjs
  buffer    = scene.buffer

  for i = 1:length(superObjs)
    rootSuperObj = buffer[i]
    if dataHasMass(rootSuperObj)
      # @error("... initializeMassComputation: dataHasMass = true muss noch gecheckt werden!")
      #println(ModiaMath.fullName(rootSuperObj))
      #println("rootSuperObj.data.massProperties = ", rootSuperObj.data.massProperties)
      rootSuperObj.massProperties = Solids.InternalMassProperties()
      rootSuperObj.massProperties.m = rootSuperObj.data.massProperties.m
      rootSuperObj.massProperties.rCM = rootSuperObj.data.massProperties.rCM
      rootSuperObj.massProperties.I = rootSuperObj.data.massProperties.I
      #println("rootSuperObj.massProperties = ", rootSuperObj.massProperties)
      computeInertiaTensorForTwoBodies!(rootSuperObj, superObjs[i].superObjMass.superObj)
    else
      if length(superObjs[i].superObjMass.superObj) > 0
        rootSuperObj.massProperties = Solids.InternalMassProperties()
        computeInertiaTensorForTwoBodies!(rootSuperObj, superObjs[i].superObjMass.superObj)
      end
    end
  end
  return nothing
end
