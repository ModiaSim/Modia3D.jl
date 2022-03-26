# License for this file: MIT (expat)
# Copyright 2022, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/src/Shapes/_module.jl)
#

"""
    loadPalettes!(; solidMaterialPalette       = nothing, 
                    contactPairMaterialPalette = nothing,
                    visualMaterialPalette      = nothing,
                    log = true
Read the palettes for which a file name is given and store them as global palettes in Modia3D.
If `log=true`, log the reading of the files.

When Modia3D is used the first time, it reads the palettes automatically from
`"Modia3D/palettes/*.json"`. The `loadPalettes` function overwrites this default setting.

# Examples

```
file1 = "$(Modia3D.path)/palettes/solidMaterials.json"
file2 = "$(Modia3D.path)/palettes/contactPairMaterials.json"
file3 = "$(Modia3D.path)/palettes/visualMaterials.json"

Modia3D.loadPalettes!(solidMaterialPalette       = file1, 
                      contactPairMaterialPalette = file2,
                      visualMaterialPalette      = file3)
```
"""
function loadPalettes!(; solidMaterialPalette       = nothing, 
                         contactPairMaterialPalette = nothing,
                         visualMaterialPalette      = nothing,
                         log = true)::Nothing
    if !isnothing(solidMaterialPalette)
        if log
            println("  Reading solidMaterialPalette from JSON file \"$solidMaterialPalette\".")
        end
        rereadSolidMaterialFromJSON(file = solidMaterialPalette)
    end
    
    if !isnothing(contactPairMaterialPalette)
        if log
            println("  Reading contactPairMaterialPalette from JSON file \"$contactPairMaterialPalette\".")
        end    
        rereadContactPairMaterialFromJSON(file = contactPairMaterialPalette)
    end
    
    if !isnothing(visualMaterialPalette)
        if log
            println("  Reading visualMaterialPalette from JSON file \"$visualMaterialPalette\".")
        end    
        rereadVisualMaterialFromJSON(file = visualMaterialPalette)
    end
    
    return nothing
end


"""
    showPalettes()
    
Show loaded palettes
"""
function showPalettes()::Nothing
    println("\nsolidMaterialPalette = ", Modia3D.solidMaterialPalette[1])
    println("\ncontactPairMaterialPalette = ", Modia3D.contactPairMaterialPalette[1])
    println("\nvisualMaterialPalette = ", Modia3D.visualMaterialPalette)
    return nothing
end
