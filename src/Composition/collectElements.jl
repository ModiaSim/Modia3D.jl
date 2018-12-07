
function fillStackOrBuffer!(scene::Scene, obj::Object3D, superObj::SuperObjsRow)
  for child in obj.children
    if isNotWorld(child)
      if isNotFixed(child)
        push!(scene.buffer, child)
        if !child.joint.canCollide    # !isFree(child) &&  !( typeof(child.joint) <: Modia3D.AbstractPrismatic )
          push!(superObj.noCPair, length(scene.buffer))
        end
      else
        push!(scene.stack, child)
      end
    end
  end
  return nothing
end



function checkCollision(scene::Scene, obj::Object3D, superObj::Array{Object3D,1})
  if canCollide(obj)
    push!(superObj, obj)
  end
  return superObj
end


function build_elements!(scene::Scene, world::Object3D)::NOTHING
  stack = scene.stack
  buffer = scene.buffer
  coll = scene.superObjs
  empty!(stack)
  empty!(buffer)

  push!(buffer, world)
  actPos = 1
  nPos   = 1

  while actPos <= nPos
    superObj         = SuperObjsRow()
    AABBrow          = Array{Basics.BoundingBox,1}()
    obj = buffer[actPos]

    if obj != world
      superObj = assignAll(scene,obj,superObj)
    end
    fillStackOrBuffer!(scene,obj,superObj)

    while length(stack) > 0
      objChild       = pop!(stack)
      assignAll(scene,objChild,superObj)
      fillStackOrBuffer!(scene,objChild,superObj)
    end

    if length(superObj) > 0
      AABBrow = [Basics.BoundingBox() for i = 1:length(superObj)]
      push!(scene.celements, superObj)
      push!(scene.AABB, AABBrow)
      if isempty(cantCollSuperObj)
        push!(scene.noCPairs, [0])
      else
        push!(scene.noCPairs, cantCollSuperObj)
      end
    end
    nPos = length(buffer)
    actPos += 1
  end

#=
  println("scene.noCPairs ", scene.noCPairs)
  for a in scene.celements
    println("[")
    for b in a
      println(b)
    end
    println("]")
  end
=#

  if length(scene.celements) > 1
    scene.collide = true
  else
    scene.collide = false
  end
  return nothing
end
