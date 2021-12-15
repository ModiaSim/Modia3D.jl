# It is about initializing, adding and substracting common mass properties
# consisting of mass m, center of mass rCM and inertia tensor I.
# At initialization common mass properties are computed of each mass super object.
# The result is stored in the root (= element of buffer).
# If the structure changes dynamically, because of gripping, mass properties must be updated
# (e.g. a robot picks something up (compute resulting common mass properties)).

### ---------------------- initialize common mass properties ----------------------
# initializeMassComputation! is called at initialization for computing common mass properties
# of each mass super object. Common mass properties are stored and assigned to the root of
# a super object (root of a super object ^= element of buffer).
# At initialization it needs to differed if a root object has already mass properties,
# otherwise dummy mass properties must be assigned, before computing common properties.
# The initial common mass properties are stored separatelly.
# This information is used for computing common mass for dynamically changing structures.
function initializeMassComputation!(scene::Scene{F}) where {F}
    if scene.initMassComp != true
        superObjs = scene.superObjs
        buffer    = scene.buffer
        for i = 1:length(buffer)
            rootSuperObj = buffer[i]

            if featureHasMass(rootSuperObj) # feature of root object has already mass properties
                rootSuperObj.hasMass = true
                rootSuperObj.m       = rootSuperObj.feature.massProperties.m
                rootSuperObj.r_CM    = rootSuperObj.feature.massProperties.rCM
                rootSuperObj.I_CM    = rootSuperObj.feature.massProperties.I
                addMassPropertiesOfAllSuperObjChildsToRootSuperObj!(rootSuperObj, superObjs[i].superObjMass.superObj)
            else # root obj's feature has no mass properties, but common mass super object do
                if length(superObjs[i].superObjMass.superObj) > 0
                    rootSuperObj.hasMass = true
                    rootSuperObj.m       = F(0.0)
                    rootSuperObj.r_CM    = Modia3D.ZeroVector3D(F)
                    rootSuperObj.I_CM    = SMatrix{3,3,F,9}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    addMassPropertiesOfAllSuperObjChildsToRootSuperObj!(rootSuperObj, superObjs[i].superObjMass.superObj)
                end
            end
        end
        scene.initMassComp = true
    end
end

# It loops over all objects of an actual mass super object.
# It sums up common mass, inertia tensor and center of mass.
# The results are stored twice in actual root of super object:
#       massProperties: container for actual values
function addMassPropertiesOfAllSuperObjChildsToRootSuperObj!(rootSuperObj::Object3D, actualMassSuperObject::Vector{Object3D{F}}) where {F}
    if length(actualMassSuperObject) > 0
        for i=1:length(actualMassSuperObject)
            obj = actualMassSuperObject[i]
            if rootSuperObj != obj && rootSuperObj == obj.parent
                addOrSubtractMassPropertiesOfChildToRoot!(rootSuperObj, obj; add=true)
        end; end
    else
        rootSuperObj.hasMass = false
    end
#  println(Modia3D.fullName(rootSuperObj), " m ", rootSuperObj.massProperties.m )
#  println(Modia3D.fullName(rootSuperObj), " rCM ", rootSuperObj.massProperties.rCM )
#  println(Modia3D.fullName(rootSuperObj), " I ", rootSuperObj.massProperties.I )
end

### --------------- computing common mass properties for two bodies ---------------
# A movable unit is added or substracted from its parent object.
# Only the parent (root of the super object) and its child (= root of a movable unit)
# hold information about their mass properties.
# First, the childs center of mass must be transformed to parents coordinate system.
# Further, the common center of mass with the help of the common mass can be caluclated.
# Both inertia tensors of child and parent are transformed with Satz von Steiner
#             I_steiner = E*I*E' + m * skew(rCM)' * skew(rCM),
# summed up and again transformed with the common center of mass.
# Adding: add mass properties of a child to its parents mass properties
# Substracting: substract mass properties of a child from its parents mass properties

# function computeInertiaTensorForTwoBodies!(massPropParent, massPropObj, obj; add=true)
function addOrSubtractMassPropertiesOfChildToRoot!(obj_root, obj_child; add=true)::Nothing
    massProperties_child = obj_child.feature.massProperties

    R_child   = obj_child.R_rel
    r_child   = obj_child.r_rel
    m_child   = massProperties_child.m
    rCM_child = massProperties_child.rCM
    I_child   = massProperties_child.I

    m_root    = obj_root.m
    rCM_root  = obj_root.r_CM
    I_root    = obj_root.I_CM

    # transform childs center of mass to parents coordinate system
    rCM_child_new = r_child + R_child' * rCM_child

    # I_child_steiner: I_child needs to be transformed to parents coordinate system
    # I_root_steiner: I_root needs to be transformed to parents coordinate system
    #                 (no need of rotation matrices)
    I_child_steiner = Modia3D.NullRotation(Float64) * I_child * Modia3D.NullRotation(Float64)' +
                    m_child * Modia3D.skew(rCM_child_new)' * Modia3D.skew(rCM_child_new)
    I_root_steiner = I_root +
                    m_root * Modia3D.skew(rCM_root)' * Modia3D.skew(rCM_root)

    if add
        # ----------- adding mass properties (parent + child) ----------------------
        # common mass (parent + child)
        m = m_root + m_child

        # common center of mass (parent + child)
        @assert(m > 0.0)
        rCM = (m_root * rCM_root + m_child * rCM_child_new)/m

        # I: substract new common mass multiplied with skew matrices of
        #    center of mass from the sum of I_root_steiner and I_child_steiner
        I = I_root_steiner + I_child_steiner - m * Modia3D.skew(rCM)' * Modia3D.skew(rCM)

        # Assign to obj_root
        obj_root.m    = m
        obj_root.r_CM = rCM
        obj_root.I_CM = I
        return nothing

        #= # other way to compute inertia tensor
        a1 = rCM_new - rCM_child_new
        a2 = rCM_root - rCM_new
        IchildNew =  I_child + m_child * Modia3D.skew(a1)' * Modia3D.skew(a1)
        IrootNew  =  I_root  + m_root  * Modia3D.skew(a2)' * Modia3D.skew(a2)
        InewNew = IchildNew + IrootNew
        =#
    else
        # ----------- substracting mass properties (parent - child) ----------------
        # new mass is the difference between parents and childs mass
        m = m_root - m_child

        # new center of mass is the difference between parents and childs rCM
        if m != 0.0
            rCM = (m_root * rCM_root - m_child * rCM_child_new)/m
        else
            rCM = Modia3D.ZeroVector3D(Float64)
        end

        # I: substract the sum of I_child_steiner and new mass multiplied with skew matrices of
        #    new center of mass from I_root_steiner
        I = I_root_steiner - I_child_steiner - m * Modia3D.skew(rCM)' * Modia3D.skew(rCM)

        # Assign to obj_root
        obj_root.m    = m
        obj_root.r_CM = rCM
        obj_root.I_CM = I
        return nothing
end; end
