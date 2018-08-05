# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.ContactDetectionMPR (Modia3D/contactDetection/ContactDetectionMPR/_module.jl)
#


AABB_collision(aabb1::Basics.BoundingBox, aabb2::Basics.BoundingBox) = aabb1.x_max > aabb2.x_min && aabb1.x_min < aabb2.x_max &&
                                           aabb1.y_max > aabb2.y_min && aabb1.y_min < aabb2.y_max &&
                                           aabb1.z_max > aabb2.z_min && aabb1.z_min < aabb2.z_max



function Composition.initializeContactDetection!(world::Composition.Object3D, scene::Composition.Scene)
  # ch::Composition.ContactDetectionMPR_handler, celements::Array{Array{Composition.Object3D}}, cantCollide::Array{Array{Int64,1}}, AABB::Array{Array{Basics.BoundingBox}})
  # scene.options.contactDetection, scene.celements, scene.cantCollide, scene.AABB
  ch = scene.options.contactDetection
  ch.contactPairs = Composition.ContactPairs(scene.celements, scene.cantCollide, scene.AABB, scene.options.nz_max)
  if ch.contactPairs.nz == 0
     Composition.closeContactDetection!(ch)
     scene.collide = false
     warn("\n... From Modia3D collision handler: Collision handling switched off, since no contacts can take place (nz=0).\n",
          "... You might need to set canCollide=true at joints.\n")
     return
  end
  @assert(ch.contactPairs.ne > 0)
  @assert(ch.contactPairs.nz > 0)

#=
  if ch.visualizeContactPoints
    ch.contactPointShape1 = []
    ch.contactPointShape2 = []
    for i in 1:ch.n_max
      obj = Shapes.Sphere(0.05; visible=true, collide=false, material=Shapes.Material(; color=[0,0,0]))
      Shapes.defineZeroShapePosition!(obj)
      obj.r_abs = SVector{3,Float64}(0.0,0.0,0.0)
      obj.R_abs = SMatrix{3,3,Float64,9}(eye(3))
      push!(ch.contactPointShape1, obj)

      obj2 = Shapes.Sphere(0.05; visible=true, collide=false, material=Shapes.Material(; color=[0,0,0]))
      Shapes.defineZeroShapePosition!(obj2)
      obj2.r_abs = SVector{3,Float64}(0.0,0.0,0.0)
      obj2.R_abs = SMatrix{3,3,Float64,9}(eye(3))
      push!(ch.contactPointShape2, obj2)
    end
  end
=#
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
function Composition.selectContactPairs!(ch::Composition.ContactDetectionMPR_handler) # ch::Composition.ContactDetectionMPR_handler)
  if !ch.distanceComputed
    computeDistances(ch,false)
  end
  if !isempty(ch.dict1)
    if !isempty(ch.dict2)
      empty!(ch.dict2)
    end
    tmp = collect(ch.dict1)
    for i=1:length(tmp)
      ch.contactPairs.z[i] = tmp[i][1][1]        # fill z vector with smallest distances
      ch.contactPairs.contactPoint1[i] = tmp[i][2][2]
      ch.contactPairs.contactPoint2[i] = tmp[i][2][3]
      ch.contactPairs.contactNormal[i] = tmp[i][2][4]
      ch.contactPairs.contactObj1[i]   = tmp[i][2][5]
      ch.contactPairs.contactObj2[i]   = tmp[i][2][6]
      ch.dict2[tmp[i][2][1][]] = [tmp[i][1], i]  # interchange key and value of dictionary dict1 + position in z vector
    end
  else # No AABBs are overlapping, take old z!
    for i=1:length(ch.contactPairs.z)
      ch.contactPairs.z[i] = 42.0
    end
  end
  #println("ch.contactPairs.z = ", ch.contactPairs.z)
  #println("\n")
  ch.distanceComputed = true
end

# This function performs (a) a broad phase to determine which
# shapes are potentially in contact to each other, (b) computes
# the distances of these shapes in a narrow phase and (c)
# stores the distances of the contact pairs selected by the last
# call of function selectContactPairs!(C) in z in O(n log(nz))
# operations. This function is called whenever the integrator
# requests a new zero-crossing function evaluation
function Composition.getDistances!(ch::Composition.ContactDetectionMPR_handler)
  if !ch.distanceComputed
    computeDistances(ch,true)
    #println("getDistances: ch.contactPairs.z = ", ch.contactPairs.z)
    #println("\n")
    ch.distanceComputed = true
  end
end


function computeDistances(ch::Composition.ContactDetectionMPR_handler, phase2::Bool)
  celements = ch.contactPairs.celements
  cantCollide = ch.contactPairs.cantCollide
  AABB = ch.contactPairs.AABB
  if length(celements) > 1
    for i = 1:length(celements)
      superObj = celements[i]
      for j = 1:length(superObj)
        obj = superObj[j]
        AABB[i][j] = Solids.boundingBox!(obj.data.geo, AABB[i][j], obj.r_abs, obj.R_abs; tight=true, scaleFactor=0.1)
      end
    end

    # new computation of dictionary 1
    if !isempty(ch.dict1)
      empty!(ch.dict1)
    end

    for i_superObj = 1:length(celements)
      superObj = celements[i_superObj]
      superAABB = AABB[i_superObj]
      for i_obj = 1:length(superObj)
        obj = superObj[i_obj]      # determine contact from this Object3D with all Object3Ds that have larger indices
        aabb = superAABB[i_obj]
        for i_next_superObj = i_superObj+1:length(celements)
          if !(i_next_superObj in cantCollide[i_superObj]) # index is not in objects which cant collide
            nextSuperObj = celements[i_next_superObj]
            nextSuperAABB = AABB[i_next_superObj]
            for i_nextObj = 1:length(nextSuperObj)
              nextObj = nextSuperObj[i_nextObj]
              nextAabb = nextSuperAABB[i_nextObj]
              if AABB_collision(aabb, nextAabb)
                index = pack(i_superObj,i_obj,i_next_superObj,i_nextObj)
                (distance, contactPoint1, contactPoint2, contactNormal,r1_a, r1_b, r2_a, r2_b, r3_a, r3_b) = collision(ch, obj, nextObj)
                if length(ch.dict1) < ch.contactPairs.nz
                  push!(ch.dict1, distance=>(index,contactPoint1,contactPoint2,contactNormal,obj,nextObj))
                else
                  (k,v) = last(ch.dict1)  # returns last sorted key -k, and its value - v
                  if distance < k && k <= 0.0
                    error("Number of max. collisions (n_max) is too low.")
                  elseif distance < k && k >= 0.0
                    delete!(ch.dict1,k)
                    push!(ch.dict1, distance=>(index,contactPoint1,contactPoint2,contactNormal,obj,nextObj))  # new distance is added, it is smaller than the biggest one in dict1
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
                      j = Int(tmp_val[2])
                      ch.contactPairs.z[j] = distance        # new distance is stored in z, at it's old position
                      ch.contactPairs.contactPoint1[j] = contactPoint1
                      ch.contactPairs.contactPoint2[j] = contactPoint2
                      ch.contactPairs.contactNormal[j] = contactNormal
                      ch.contactPairs.contactObj1[j]   = obj
                      ch.contactPairs.contactObj2[j]   = nextObj
                    else
                      if distance < 0.0
                        error("\nNumber of max. collision pairs nz (= ", ch.contactPairs.nz, ") is too low.",
                              "\nProvide a large nz_max with Modia3D.SceneOptions(nz_max=xxx).")
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end
    #visualizeContactPoints()
    #visualizeSupportPoints()
 end
 # println("\n")
 return nothing
end

#=
function Composition.visualizeContactPoints()
  if ch.visualizeContactPoints
    i = 0
    for obj in ch.contactPointShape1
      i += 1
      obj.diameter = 0.05  #Shapes.Sphere(0.05; visible=true, collide=false, material=Shapes.Material(; color=[0,0,0]))
      obj.r_abs = ch.contactPoint1[i] #point
    end
    i = 0
    for obj in ch.contactPointShape2
      i += 1
      obj.diameter = 0.05  #Shapes.Sphere(0.05; visible=true, collide=false, material=Shapes.Material(; color=[0,0,0]))
      obj.r_abs = ch.contactPoint2[i]
    end
  end
end
=#


#function Composition.visualizeSupportPoints()
  #if ch.visualizeSupportPoints
    #println("not supported yet!")
    #=
           if r1_a != nothing
             ch.SupportPoints[c] = SimVis.Sphere(0.05; material=SimVis.Material(; color=[255,0,0]))
             ch.SupportPoints[c].r_abs .= r1_a
             ch.SupportPoints[c+size] = SimVis.Sphere(0.05; material=SimVis.Material(; color=[255,0,0]))
             ch.SupportPoints[c+size].r_abs .= r1_b

             ch.SupportPoints[c+2*size] = SimVis.Sphere(0.05; material=SimVis.Material(; color=[0,255,0]))
             ch.SupportPoints[c+2*size].r_abs .= r2_a
             ch.SupportPoints[c+3*size] = SimVis.Sphere(0.05; material=SimVis.Material(; color=[0,255,0]))
             ch.SupportPoints[c+3*size].r_abs .= r2_b

             ch.SupportPoints[c+4*size] = SimVis.Sphere(0.05; material=SimVis.Material(; color=[0,0,255]))
             ch.SupportPoints[c+4*size].r_abs .= r3_a
             ch.SupportPoints[c+5*size] = SimVis.Sphere(0.05; material=SimVis.Material(; color=[0,0,255]))
             ch.SupportPoints[c+5*size].r_abs .= r3_b
           end
    =#
  #end
#end


function Composition.closeContactDetection!(ch::Composition.ContactDetectionMPR_handler)
  Basics.emptyArray!(ch.dict1)
  Basics.emptyArray!(ch.dict2)
  Basics.emptyArray!(ch.contactPairs.celements)
  Basics.emptyArray!(ch.contactPairs.cantCollide)
  Basics.emptyArray!(ch.contactPairs.z)
  Basics.emptyArray!(ch.contactPairs.contactPoint1)
  Basics.emptyArray!(ch.contactPairs.contactPoint2)
  Basics.emptyArray!(ch.contactPairs.contactNormal)
  Basics.emptyArray!(ch.contactPairs.contactObj1)
  Basics.emptyArray!(ch.contactPairs.contactObj2)
end
