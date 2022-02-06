"""
    ElasticContactPairMaterial(; coefficientOfRestitution=0.7,
                                 slidingFrictionCoefficient=0.5,
                                 rotationalResistanceCoefficient=0.001,
                                 vsmall=0.01, wsmall=0.01)

Generates an `ElasticContactPairMaterial` object by providing the material properties of two objects that are in contact to each other. Those properties are needed for the [Contact Force Law](@ref).

# Arguments
- `coefficientOfRestitution`: Coefficient of restitution between two objects (=0: inelastic ... =1: fully elastic).
- `slidingFrictionCoefficient`: Kinetic/sliding friction force coefficent between two objects (`>= 0.0`).
- `rotationalResistanceCoefficient`: Rotational resistance torque coefficient between two objects (`>= 0.0`).
  Its effect is that the contact torque is computed to reduce the relative angular velocity between two objects.
  For a ball, `rotationalResistanceCoefficient` is the (standard) rolling resistance coefficient.
- `vsmall` in [m/s]: Used for regularization when computing the unit vector in direction of the relative tangential velocity to avoid a division by zero.
- `wsmall` in [rad/s]: Used for regularization when computing the unit vector in direction of the relative angular velocity to avoid a division by zero.
"""
mutable struct ElasticContactPairMaterial <: Modia3D.AbstractContactPairMaterial
    coefficientOfRestitution::Float64
    slidingFrictionCoefficient::Float64
    rotationalResistanceCoefficient::Float64
    vsmall::Float64
    wsmall::Float64

    function ElasticContactPairMaterial(; coefficientOfRestitution=0.0,
                                          slidingFrictionCoefficient=0.0,
                                          rotationalResistanceCoefficient=0.0,
                                          vsmall=0.01,
                                          wsmall=0.01)
        @assert(coefficientOfRestitution >= 0.0)
        @assert(coefficientOfRestitution <= 1.0)
        @assert(slidingFrictionCoefficient >= 0.0)
        @assert(rotationalResistanceCoefficient >= 0.0)
        new(coefficientOfRestitution, slidingFrictionCoefficient,
            rotationalResistanceCoefficient, vsmall, wsmall)
end; end

struct NoContactPairMaterial <: Modia3D.AbstractContactPairMaterial
end

mutable struct ObserverContactPairMaterial <: Modia3D.AbstractContactPairMaterial
    printAlarm::Bool

    function ObserverContactPairMaterial(; printAlarm=false)
        new(printAlarm)
end; end

mutable struct ImpulseContactPairMaterial <: Modia3D.AbstractContactPairMaterial
end

mutable struct WheelRailContactPairMaterial <: Modia3D.AbstractContactPairMaterial
end

@enum ResponseType ElasticResponse ObserverResponse ImpulseResponse WheelRailResponse NoResponse

function getResponseMaterial(responseType::String)
    if responseType == "ElasticResponse"
        return ElasticContactPairMaterial()
    end
    if responseType == "ObserverResponse"
        return ObserverContactPairMaterial()
    end
    if responseType == "ImpulseResponse"
        return ImpulseContactPairMaterial()
    end
    if responseType == "WheelRailResponse"
        return WheelRailContactPairMaterial()
    end
    if responseType == "NoResponse"
        return NoContactPairMaterial()
    end
end

"""
    material = combineContactPairMaterials(material1, material2)

Return a new `ElasticContactPairMaterial` object from the mean values of the ElasticContactPairMaterial
objects `material1` and `material2`.
"""
function combineContactPairMaterials(mat1::ElasticContactPairMaterial, mat2::ElasticContactPairMaterial)::ElasticContactPairMaterial
    mat = ElasticContactPairMaterial()
    mat.coefficientOfRestitution        = (mat1.coefficientOfRestitution + mat2.coefficientOfRestitution)/2
    mat.slidingFrictionCoefficient      = (mat1.slidingFrictionCoefficient + mat2.slidingFrictionCoefficient)/2
    mat.rotationalResistanceCoefficient = (mat1.rotationalResistanceCoefficient + mat2.rotationalResistanceCoefficient)/2
    mat.vsmall = (mat1.vsmall + mat2.vsmall)/2
    mat.wsmall = (mat1.wsmall + mat2.wsmall)/2
    return mat
end


struct TwoNamesKey
    name1::String
    name2::String
    function TwoNamesKey(name1::String, name2::String)
        if name1 < name2
            return new(name1, name2)
        else
            return new(name2, name1)
end; end; end

Base.:isequal(keyA::TwoNamesKey, keyB::TwoNamesKey) = keyA.name1 == keyB.name1 && keyA.name2 == keyB.name2
Base.:isless(keyA::TwoNamesKey, keyB::TwoNamesKey) = keyA.name1 < keyB.name1 || ( keyA.name1 == keyB.name1 && keyA.name2 < keyB.name2 )

"""
    readContactPairMaterialFromJSON(filename)

Read a JSON file consisting of a dictionary of ContactPairMaterial instances from `filename` and
return a `Dict{String, ContactPairMaterial}` dictionary.
"""
function readContactPairMaterialFromJSON(filename::AbstractString)
    dataJSON = JSON.parsefile(filename)
    palette = Dict{TwoNamesKey, Modia3D.AbstractContactPairMaterial}()
    for (namesPair,dataPair) in dataJSON
        # Split a key "name1,name2" in two parts
        i = findfirst(",", namesPair)
        if isnothing(i) || i.start < 2 || i.start >= length(namesPair)
            println("... Wrong key \"", namesPair,"\" not included in contactPairMaterialPalette.")
        else
            namesKey = TwoNamesKey( namesPair[1:i.start-1], namesPair[i.start+1:end] )
            responseMat = getResponseMaterial(pop!(dataPair, "responseType", NoResponse))
            for (propName,propData) in dataPair
                if isdefined(responseMat, Symbol(propName))
                    setfield!(responseMat, Symbol(propName), propData)
                else
                    error("... from key: ", namesKey, " . There might be spelling mistakes. Check ", propName, ".")
                end
            end
            palette[namesKey] = responseMat
    end; end
    return palette
end


"""
    contactPairMaterialPalette

Dictionary of contact pair material data, see [`ElasticContactPairMaterial`](@ref)
"""
contactPairMaterialPalette = readContactPairMaterialFromJSON( joinpath(Modia3D.path, "palettes", "contactPairMaterials.json") )

function rereadContactPairMaterialFromJSON()
    global contactPairMaterialPalette = readContactPairMaterialFromJSON( joinpath(Modia3D.path, "palettes", "contactPairMaterials.json") )
    return nothing
end


"""
    material = getContactPairMaterial(obj1, obj2)

Return a contact pair material object of the materials with the
names `name1` and `name2` from dictionary `Modia3D.contactPairMaterialPalette`.
"""
function getContactPairMaterial(obj1, obj2)::Modia3D.AbstractContactPairMaterial
    name1 = obj1.feature.contactMaterial
    name2 = obj2.feature.contactMaterial

    value = get(contactPairMaterialPalette, TwoNamesKey(name1,name2), NoContactPairMaterial())
    if typeof(value) == NoContactPairMaterial
        # Combination name1,name2 is not present
        value1 = get(contactPairMaterialPalette, TwoNamesKey(name1,name1), NoContactPairMaterial())
        value2 = get(contactPairMaterialPalette, TwoNamesKey(name2,name2), NoContactPairMaterial())
        if typeof(value1) != NoContactPairMaterial && typeof(value2) != NoContactPairMaterial
            return combineContactPairMaterials(value1, value2)
        else
            error("\nFor contact between \"", Modia3D.fullName(obj1), "\" and \"", Modia3D.fullName(obj2), "\" pairing of the\ncontact materials \"", name1, "\" and \"", name2, "\" is required.\nBut this combination is not available in ", joinpath(Modia3D.path, "palettes", "contactPairMaterials.json"), ".\n")
        end
    end
    return value
end


"""
    material = checkContactPairMaterialInit(obj1, obj2)

Return a contact pair material object of the materials with the
names `name1` and `name2` from dictionary `Modia3D.contactPairMaterialPalette`.
"""
function checkContactPairMaterialInit(obj1, obj2)::Nothing
    name1 = obj1.feature.contactMaterial
    name2 = obj2.feature.contactMaterial

    value = get(contactPairMaterialPalette, TwoNamesKey(name1,name2), NoContactPairMaterial())
    if typeof(value) == NoContactPairMaterial
        # Combination name1,name2 is not present
        value1 = get(contactPairMaterialPalette, TwoNamesKey(name1,name1), NoContactPairMaterial())
        value2 = get(contactPairMaterialPalette, TwoNamesKey(name2,name2), NoContactPairMaterial())
        if typeof(value1) != NoContactPairMaterial && typeof(value2) != NoContactPairMaterial
            return nothing
        else
            @warn("If a contact between $(Modia3D.fullName(obj1)) and $(Modia3D.fullName(obj2)) occurs during simulation a pairing of contact materials $name1 and $name2 will be required. But this combination is not available in $Modia3D.path/palettes/contactPairMaterials.json.")
        end
    end
    return nothing
end
