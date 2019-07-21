# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.ContactDetectionMPR (Modia3D/contactDetection/ContactDetectionMPR/_module.jl)
#
using LinearAlgebra
EYE3() = Matrix(1.0I,3,3)

# returns true, if AABB's are touching
AABB_touching(aabb1::Basics.BoundingBox, aabb2::Basics.BoundingBox) = aabb1.x_max >= aabb2.x_min && aabb1.x_min <= aabb2.x_max &&
                                           aabb1.y_max >= aabb2.y_min && aabb1.y_min <= aabb2.y_max &&
                                           aabb1.z_max >= aabb2.z_min && aabb1.z_min <= aabb2.z_max



function Composition.initializeContactDetection!(world::Composition.Object3D, scene::Composition.Scene)
  ch = scene.options.contactDetection
  ch.contactPairs = Composition.ContactPairs(world,  scene.superObjs, scene.noCPairs, scene.AABB, scene.options.nz_max,
                                ch.visualizeContactPoints, ch.visualizeSupportPoints, ch.defaultContactSphereDiameter)
  if ch.contactPairs.nz == 0
     Composition.closeContactDetection!(ch)
     scene.collide = false
         @warn "... From Modia3D collision handler: Collision handling switched off, since no contacts can take place (nz=0).\n" *
               "... You might need to set canCollide=true at joints.\n"
     return
  end
  @assert(ch.contactPairs.ne > 0)
  @assert(ch.contactPairs.nz > 0)
end


function Composition.setComputationFlag(ch::Composition.ContactDetectionMPR_handler)
  ch.distanceComputed = false
end

# This function performs (a) a broad phase to determine which
# shapes are potentially in contact to each other, (b) computes
# the distances of these shapes in a narrow phase and (c) selects
# the nz shape pairs with the smallest distances and orders them
# according to their distances in O(n_n log(n_z)) operations.
# This function is called before every integrator step.
function Composition.selectContactPairsWithEvent!(sim::Union{ModiaMath.SimulationState,NOTHING}, ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D)
    empty!(ch.contactDict)
    empty!(ch.noContactDict)
    empty!(ch.distanceSet)
    selectContactPairs!(sim,ch,world,true)
    ch.contactPairs.nzContact = length(ch.contactDict)
end


# This function performs (a) a broad phase to determine which
# shapes are potentially in contact to each other, (b) computes
# the distances of these shapes in a narrow phase and (c) selects
# the nz shape pairs with the smallest distances and orders them
# according to their distances in O(n_n log(n_z)) operations.
# This function is called before every integrator step.
# It also checks if the contact pairs (contact == true), from a previous call of selectContactPairsWithEvent!(...)
# are still true.
function Composition.selectContactPairsNoEvent!(sim::Union{ModiaMath.SimulationState,NOTHING}, ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D)
    empty!(ch.noContactDict)
    empty!(ch.distanceSet)
    selectContactPairs!(sim,ch,world,false)
end


function selectContactPairs!(sim::Union{ModiaMath.SimulationState,NOTHING}, ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D, hasEvent::Bool)
  if !ch.distanceComputed
    computeDistances(ch, world, false, hasEvent)
  end

  # Store z in simulation engine
  if typeof(sim) != Nothing
    simh = sim.eventHandler
    i = 0

    for (key, pair) in ch.contactDict
      i = i + 1
      simh.z[i] = pair.distanceWithHysteresis
    end


    for (key, pair) in ch.noContactDict
      i = i + 1
      simh.z[i] = pair.distanceWithHysteresis
    end

    # Provide dummy value for not used z-entry
    while i < ch.contactPairs.nz
        i = i + 1
        simh.z[i] = 42.0
    end
  end

  ch.distanceComputed = true
end


# This function performs (a) a broad phase to determine which
# shapes are potentially in contact to each other, (b) computes
# the distances of these shapes in a narrow phase and (c)
# stores the distances of the contact pairs selected by the last
# call of function selectContactPairsWithEvent!(C) in z in O(n log(nz))
# operations. This function is called whenever the integrator
# requests a new zero-crossing function evaluation
function Composition.getDistances!(ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D)
  if !ch.distanceComputed
    computeDistances(ch, world, true, false)
    ch.distanceComputed = true
  end
end


function computeDistances(ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D, phase2::Bool, hasEvent::Bool=true)
  collSuperObjs = ch.contactPairs.collSuperObjs
  noCPairs = ch.contactPairs.noCPairs

  AABB = ch.contactPairs.AABB
  if length(collSuperObjs) > 1
    for i = 1:length(collSuperObjs)
      superObj = collSuperObjs[i]
      for j = 1:length(superObj)
        obj = superObj[j]
        AABB[i][j] = Solids.boundingBox!(obj.data.geo, AABB[i][j], obj.r_abs, obj.R_abs; tight=false, scaleFactor=0.01)
    end; end

    # counter
    # is: actual super - object
    # js: subsequent super - object
    # i: Object3D of is_th super - object
    # j: Object3D of js_th super - object
    n = 0
    for is = 1:length(collSuperObjs)
      actSuperObj = collSuperObjs[is]
      if !isempty(actSuperObj)
        actSuperAABB = AABB[is]
      for i = 1:length(actSuperObj)
        actObj = actSuperObj[i]       # determine contact from this Object3D with all Object3Ds that have larger indices
        actAABB = actSuperAABB[i]
        for js = is+1:length(collSuperObjs)
          if !(js in noCPairs[is])    # PairID is not in objects which cant collide
            nextSuperObj = collSuperObjs[js]
            nextSuperAABB = AABB[js]
            for j = 1:length(nextSuperObj)
              n += 1
              nextObj  = nextSuperObj[j]
              nextAABB = nextSuperAABB[j]
              pairID  = pack(is,i,js,j)
              storeDistancesForSolver!(world, pairID, ch, actObj, nextObj, actAABB, nextAABB, phase2, hasEvent)
  end; end; end; end; end; end; end
end


const zEps  = 1.e-8
const zEps2 = 2*zEps


function pushCollisionPair!(ch, contact, distanceWithHysteresis, pairID, contactPoint1,contactPoint2,contactNormal,actObj,nextObj,hasevent)::Nothing
    distanceKey = Composition.DistanceKey(contact, distanceWithHysteresis, pairID)
    if  hasevent && (length(ch.distanceSet) < ch.contactPairs.nz) ||
       !hasevent && (length(ch.distanceSet) + length(ch.contactDict) < ch.contactPairs.nz)
        push!(ch.distanceSet, distanceKey)
        if contact
            push!(ch.contactDict  , pairID => Modia3D.ContactPair(  contactPoint1,contactPoint2,contactNormal,actObj,nextObj,distanceWithHysteresis))
        else
            push!(ch.noContactDict, pairID => Modia3D.NoContactPair(contactPoint1,contactPoint2,contactNormal,actObj,nextObj,distanceWithHysteresis))
        end
    else
        lastKey = last(ch.distanceSet)
        if contact && lastKey.contact
            error("Number of max. collision pairs is too low (nz_max = ", ch.contactPairs.nz, "). Enlarge nz_max in sceneOptions(nz_max=xxx).")
        elseif distanceWithHysteresis < lastKey.distanceWithHysteresis
            delete!(ch.distanceSet  , lastKey)  # removes last entry of distanceSet
            delete!(ch.noContactDict, lastKey.pairID)
            push!(  ch.distanceSet  , distanceKey)
            if contact
                push!(ch.contactDict  , pairID => Modia3D.ContactPair(contactPoint1,contactPoint2,contactNormal,actObj,nextObj,distanceWithHysteresis))
            else
                push!(ch.noContactDict, pairID => Modia3D.NoContactPair(contactPoint1,contactPoint2,contactNormal,actObj,nextObj,distanceWithHysteresis))
            end
        end
    end
    return nothing
end


function storeDistancesForSolver!(world::Composition.Object3D, pairID::Composition.PairID, ch::Composition.ContactDetectionMPR_handler,
                                  actObj::Composition.Object3D, nextObj::Composition.Object3D,
                                  actAABB::Basics.BoundingBox, nextAABB::Basics.BoundingBox, phase2::Bool, hasEvent::Bool)
    # If object pairs are already in contact, always perform narrow phase!!
    hasContact = haskey(ch.contactDict, pairID)

    # Broad phase
    if hasContact || AABB_touching(actAABB, nextAABB) # AABB's are overlapping
        # narrow phase
        (distanceOrg, contactPoint1, contactPoint2, contactNormal,r1_a, r1_b, r2_a, r2_b, r3_a, r3_b) = mpr(ch, actObj, nextObj, actObj.data.geo, nextObj.data.geo)
        #println("distanceOrg = $distanceOrg") #, contactPoint1 = $contactPoint1, contactPoint2 = $contactPoint2, contactNormal = $contactNormal")
    else # AABB's are not overlapping
        (distanceOrg, contactPoint1, contactPoint2, contactNormal,r1_a, r1_b, r2_a, r2_b, r3_a, r3_b) = computeDistanceBetweenAABB(actAABB, nextAABB)
    end
    # println("... 1: ", ModiaMath.instanceName(actObj), " ", ModiaMath.instanceName(nextObj), "  ", distanceOrg, " ", contactPoint1, " ", contactPoint2, " ", contactNormal)


    contact                = hasEvent ? distanceOrg < -zEps : hasContact
    distanceWithHysteresis = contact  ? distanceOrg : distanceOrg + zEps2

    if hasEvent
        # At event instant
        pushCollisionPair!(ch, contact, distanceWithHysteresis, pairID, contactPoint1,contactPoint2,contactNormal,actObj,nextObj,hasEvent)

    elseif !phase2
        # Integrator requires zero crossing function
        if contact
            # Collision pair had contact since the last event
            Modia3D.updateContactPair!(ch.contactDict[pairID], contactPoint1,contactPoint2,contactNormal,actObj,nextObj,distanceWithHysteresis)
        else
            # Collision pair did not have contact since the last event
            pushCollisionPair!(ch, contact, distanceWithHysteresis, pairID, contactPoint1,contactPoint2,contactNormal,actObj,nextObj,hasEvent)
        end

    else
        # Other model evaluation (only getDistances!())
        if contact
            # Collision pair had contact since the last event
            Modia3D.updateContactPair!(ch.contactDict[pairID], contactPoint1,contactPoint2,contactNormal,actObj,nextObj,distanceWithHysteresis)
        elseif haskey(ch.noContactDict, pairID)
            # Collision par did not have contact since the last event
            Modia3D.updateNoContactPair!(ch.noContactDict[pairID], contactPoint1,contactPoint2,contactNormal,actObj,nextObj,distanceWithHysteresis)
        end
    end


#=
        if ch.visualizeContactPoints
          transparency = 0.0
          setVisualizationContactProperties!(world.contactVisuObj1[j_local], transparency, contactPoint1)
          setVisualizationContactProperties!(world.contactVisuObj2[j_local], transparency, contactPoint2)
        end
        if ch.visualizeSupportPoints
          transparency = 0.5
          setVisualizationContactProperties!(world.supportVisuObj1A[j_local], transparency, r1_a)
          setVisualizationContactProperties!(world.supportVisuObj1B[j_local], transparency, r2_a)
          setVisualizationContactProperties!(world.supportVisuObj1C[j_local], transparency, r3_a)
          setVisualizationContactProperties!(world.supportVisuObj2A[j_local], transparency, r1_b)
          setVisualizationContactProperties!(world.supportVisuObj2B[j_local], transparency, r2_b)
          setVisualizationContactProperties!(world.supportVisuObj2C[j_local], transparency, r3_b)
        end
=#
    return nothing
end


function computeDistanceBetweenAABB(actAABB::Basics.BoundingBox, nextAABB::Basics.BoundingBox)
  xd = computeDistanceOneAxisAABB(actAABB.x_min, actAABB.x_max, nextAABB.x_min, nextAABB.x_max)
  yd = computeDistanceOneAxisAABB(actAABB.y_min, actAABB.y_max, nextAABB.y_min, nextAABB.y_max)
  zd = computeDistanceOneAxisAABB(actAABB.z_min, actAABB.z_max, nextAABB.z_min, nextAABB.z_max)
  distance = sqrt(xd^2 + yd^2 + zd^2)
  return (distance, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing)
end


function computeDistanceOneAxisAABB(A_min, A_max, B_min, B_max)
  if A_max < B_min
      return B_min - A_max
  elseif A_min > B_max
      return A_min - B_max
  else
      return 0.0
  end
end


function setVisualizationContactProperties!(obj::Composition.Object3D, transparency, contactPoint)
  if contactPoint != nothing
    Modia3D.set_r_abs!(obj, contactPoint)
    obj.data.material.transparency = transparency
  else
    obj.data.material.transparency = 1.0
  end
  return nothing
end


function Composition.closeContactDetection!(ch::Composition.ContactDetectionMPR_handler)
  empty!(ch.lastContactDict)
  empty!(ch.contactDict)
  empty!(ch.noContactDict)
  empty!(ch.distanceSet)
  empty!(ch.contactPairs.collSuperObjs)
  empty!(ch.contactPairs.noCPairs)
end


function createCrossingAsString(sim::ModiaMath.SimulationState, obj1::Composition.Object3D, obj2::Composition.Object3D)::String
  if ModiaMath.isLogEvents(sim)
    name1 = typeof(obj1) == NOTHING ? "nothing" : ModiaMath.instanceName(obj1)
    name2 = typeof(obj2) == NOTHING ? "nothing" : ModiaMath.instanceName(obj2)
    crossingAsString   = "distance(" * string(name1) * "," * string(name2) * ")"
  else
    crossingAsString = "dummyDistance(nothing,nothing)"
  end
  return crossingAsString
end
