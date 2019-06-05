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
function Composition.selectContactPairs!(ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D)
  if !ch.distanceComputed
    computeDistances(ch, world, false)
  end
  if !isempty(ch.dict1)
    if !isempty(ch.dict2)
      empty!(ch.dict2)
    end
    tmp = collect(ch.dict1)
    for i=1:length(tmp)
      ch.contactPairs.z[i] = tmp[i][1].distance        # fill z vector with smallest distances
      ch.contactPairs.contactPoint1[i] = tmp[i][2][1]
      ch.contactPairs.contactPoint2[i] = tmp[i][2][2]
      ch.contactPairs.contactNormal[i] = tmp[i][2][3]
      ch.contactPairs.contactObj1[i]   = tmp[i][2][4]
      ch.contactPairs.contactObj2[i]   = tmp[i][2][5]
      ch.dict2[tmp[i][1].index] = [tmp[i][1].distance, i, tmp[i][1].contact]  # interchange key and value of dictionary dict1 + position in z vector + flag contact
    end
  else # No AABBs are overlapping, take old z!
    for i=1:length(ch.contactPairs.z)
      ch.contactPairs.z[i] = 42.0
    end
  end
  ch.distanceComputed = true
end


function selectContactPairsNoEvent!(ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D)
  println("bin in neuer funktion ")
  if !isempty(ch.dict1)
    tmp = collect(ch.dict1)
    for i=1:length(tmp)
      if tmp[i][1].contact
        dictHelp[tmp[i][1].index] = i
      else
        @goto AfterLoopsSelectContactPairsNoEvent
      end
    end
    @label AfterLoopsSelectContactPairsNoEvent
  end


  if !ch.distanceComputed
    computeDistances(ch, world, false)
  end
  if !isempty(ch.dict1)
    if !isempty(ch.dict2)
      empty!(ch.dict2)
    end
    tmp = collect(ch.dict1)
    for i=1:length(tmp)
      ch.contactPairs.z[i] = tmp[i][1].distance        # fill z vector with smallest distances
      ch.contactPairs.contactPoint1[i] = tmp[i][2][1]
      ch.contactPairs.contactPoint2[i] = tmp[i][2][2]
      ch.contactPairs.contactNormal[i] = tmp[i][2][3]
      ch.contactPairs.contactObj1[i]   = tmp[i][2][4]
      ch.contactPairs.contactObj2[i]   = tmp[i][2][5]
      ch.dict2[tmp[i][1].index] = [tmp[i][1].distance, i, tmp[i][1].contact]  # interchange key and value of dictionary dict1 + position in z vector + flag contact
    end
  else # No AABBs are overlapping, take old z!
    for i=1:length(ch.contactPairs.z)
      ch.contactPairs.z[i] = 42.0
    end
  end
  ch.distanceComputed = true
end


# This function performs (a) a broad phase to determine which
# shapes are potentially in contact to each other, (b) computes
# the distances of these shapes in a narrow phase and (c)
# stores the distances of the contact pairs selected by the last
# call of function selectContactPairs!(C) in z in O(n log(nz))
# operations. This function is called whenever the integrator
# requests a new zero-crossing function evaluation
function Composition.getDistances!(ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D)
  if !ch.distanceComputed
    computeDistances(ch, world, true)
    ch.distanceComputed = true
  end
end


function computeDistances(ch::Composition.ContactDetectionMPR_handler, world::Composition.Object3D, phase2::Bool)
  collSuperObjs = ch.contactPairs.collSuperObjs
  noCPairs = ch.contactPairs.noCPairs

  AABB = ch.contactPairs.AABB
  if length(collSuperObjs) > 1
    for i = 1:length(collSuperObjs)
      superObj = collSuperObjs[i]
      for j = 1:length(superObj)
        obj = superObj[j]
        AABB[i][j] = Solids.boundingBox!(obj.data.geo, AABB[i][j], obj.r_abs, obj.R_abs; tight=false, scaleFactor=0.01)
      end
    end

    # new computation of dictionary 1
    if !isempty(ch.dict1)
      empty!(ch.dict1)
    end

    # counter
    # is: actual super - object
    # js: subsequent super - object
    # i: Object3D of is_th super - object
    # j: Object3D of js_th super - object
    for is = 1:length(collSuperObjs)
      actSuperObj = collSuperObjs[is]
      if !isempty(actSuperObj)
        actSuperAABB = AABB[is]
      for i = 1:length(actSuperObj)
        actObj = actSuperObj[i]      # determine contact from this Object3D with all Object3Ds that have larger indices
        actAABB = actSuperAABB[i]
        for js = is+1:length(collSuperObjs)
          if !(js in noCPairs[is]) # index is not in objects which cant collide
            nextSuperObj = collSuperObjs[js]
            nextSuperAABB = AABB[js]
            for j = 1:length(nextSuperObj)
              nextObj = nextSuperObj[j]
              nextAABB = nextSuperAABB[j]
              index = pack(is,i,js,j)
              storeDistancesForSolver!(world, index, ch, actObj, nextObj, actAABB, nextAABB, phase2)
  end; end; end; end; end; end; end
  #println("length(ch.dict1) = ", length(ch.dict1))
  #println("\n")
  return nothing
end


getInitialBoolOfDistance(distance_org) = distance_org < 0.0 ? true : false
manipulateDistance(distance_org::Float64, contact::Bool) = contact ? distance_org - 100*eps() : distance_org + 100*eps()

function hysteresis(distance_org::Float64; initial::Bool=false, contact::Bool=false)
  if initial
    contact = getInitialBoolOfDistance(distance_org)
    distance = manipulateDistance(distance_org, contact)
  else
    distance = manipulateDistance(distance_org, contact)
  end
  return (distance, contact)
end

function storeDistancesForSolver!(world::Composition.Object3D, index::Integer, ch::Composition.ContactDetectionMPR_handler,
                                  actObj::Composition.Object3D, nextObj::Composition.Object3D,
                                  actAABB::Basics.BoundingBox, nextAABB::Basics.BoundingBox, phase2::Bool)
  # Broad Phase
  if AABB_touching(actAABB, nextAABB) # AABB's are overlapping
    # narrow phase
    (distance_org, contactPoint1, contactPoint2, contactNormal,r1_a, r1_b, r2_a, r2_b, r3_a, r3_b) = mpr(ch, actObj, nextObj, actObj.data.geo, nextObj.data.geo)
    #println("distance_org = ", distance_org)
  else # AABB's are not overlapping
    (distance_org, contactPoint1, contactPoint2, contactNormal,r1_a, r1_b, r2_a, r2_b, r3_a, r3_b) = computeDistanceBetweenAABB(actAABB, nextAABB)
    #println("distance_org = ", distance_org)
  end


  if length(ch.dict1) < ch.contactPairs.nz
    (distance, contact) = hysteresis(distance_org, initial=true)
    key1 = Composition.KeyDict1(contact, distance, index)
    push!(ch.dict1, key1=>(contactPoint1,contactPoint2,contactNormal,actObj,nextObj))
  else
    (k,v) = last(ch.dict1) # returns last sorted key -k, and its value -v
    if distance_org < k.distance && k.distance <= 0.0
      error("Number of max. collisions (n_max) is too low.")
    elseif distance_org < k.distance && k.distance >= 0.0
      delete!(ch.dict1,k) # removes last entry of sorted dict1
      (distance, contact) = hysteresis(distance_org, initial=true)
      key1 = Composition.KeyDict1(contact, distance, index)
      push!(ch.dict1, key1=>(contactPoint1,contactPoint2,contactNormal,actObj,nextObj)) # new distance_org is added, it is smaller than the biggest one in dict1
    end

  end

  if phase2
    # it's for integrators update, if index of contact pair is already in dict2,
    # its distance is stored in z vector at the same position as before
    if !isempty(ch.dict2)
      # token = find(ch.dict2,index)   # is deprecated
      token = findkey(ch.dict2,index)
      if status((ch.dict2,token)) == 1          # index of contact pair is in dict2
        tmp_val = deref_value((ch.dict2,token)) # unpacking its values
        j_local = Int(tmp_val[2])
        (distance, contact) = hysteresis(distance_org, initial=false, contact=Bool(tmp_val[3]))
        ch.contactPairs.z[j_local] = distance        # new distance is stored in z, at it's old position
        ch.contactPairs.contactPoint1[j_local] = contactPoint1
        ch.contactPairs.contactPoint2[j_local] = contactPoint2
        ch.contactPairs.contactNormal[j_local] = contactNormal
        ch.contactPairs.contactObj1[j_local]   = actObj
        ch.contactPairs.contactObj2[j_local]   = nextObj

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

      else
        if distance_org < 0.0
          error("\nNumber of max. collision pairs nz (= ", ch.contactPairs.nz, ") is too low.",
                "\nProvide a large nz_max with Modia3D.SceneOptions(nz_max=xxx).")
        end
      end
end; end
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
  Basics.emptyArray!(ch.dict1)
  Basics.emptyArray!(ch.dict2)
  Basics.emptyArray!(ch.contactPairs.collSuperObjs)
  Basics.emptyArray!(ch.contactPairs.noCPairs)
  Basics.emptyArray!(ch.contactPairs.z)
  Basics.emptyArray!(ch.contactPairs.contactPoint1)
  Basics.emptyArray!(ch.contactPairs.contactPoint2)
  Basics.emptyArray!(ch.contactPairs.contactNormal)
  Basics.emptyArray!(ch.contactPairs.contactObj1)
  Basics.emptyArray!(ch.contactPairs.contactObj2)
end
