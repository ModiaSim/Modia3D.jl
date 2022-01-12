# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#
#=
mutable struct SimVis_Renderer_WithShapes{F <: AbstractFloat}
    SimVis_Renderer
    velements::Vector{Composition.Object3D{F}}
    ids::Vector{Ptr{Nothing}}                # ids[i] is the SimVis id of velements[i]


end
=#

mutable struct SimVis_Renderer
    velements::AbstractVector{Composition.Object3D}  # Objects to be visualized
    ids::Vector{Ptr{Nothing}}                # ids[i] is the SimVis id of velements[i]
    isInitialized::Bool                      # = true, if SimVis is initialized (SimVis_init was called)
    host::String
    port::Int
    sync::Bool

    directory::String            # Directory Visualization/Extras
    dll_name::String             # Absolute path of SimVis DLL as string
    isProfessionalEdition::Bool  # = true, if DLL is professional SimVis edition otherwise community edition
    isNoRenderer::Bool           # = true, if SimVis DLL is not available and the NoRenderer renderer is used
    dll_handle::Ptr{Nothing}     # Handle for SimVis DLL

    # SimVis functions
    init::Ptr{Nothing}
    shutdown::Ptr{Nothing}
    getObjectID::Ptr{Nothing}
    freeObjectID::Ptr{Nothing}
    setTime::Ptr{Nothing}
    setBaseObject::Ptr{Nothing}
    setFileObject::Ptr{Nothing}
    setTextObject::Union{Ptr{Nothing}, Nothing}

    function SimVis_Renderer(info; host="127.0.0.1",port=11000,sync=false, professionalEdition=true)
        (directory, dll_name, isProfessionalEdition, isNoRenderer) = info

        dll_handle    = Libdl.dlopen(dll_name)
        init          = Libdl.dlsym(dll_handle, :SimVis_init)
        shutdown      = Libdl.dlsym(dll_handle, :SimVis_shutdown)
        getObjectID   = Libdl.dlsym(dll_handle, :SimVis_getObjectID)
        freeObjectID  = Libdl.dlsym(dll_handle, :SimVis_freeObjectID)
        setTime       = Libdl.dlsym(dll_handle, :SimVis_setTime)
        setBaseObject = Libdl.dlsym(dll_handle, :SimVis_setBaseObject)
        setFileObject = Libdl.dlsym(dll_handle, :SimVis_setFileObject)
        if professionalEdition
            setTextObject = Libdl.dlsym(dll_handle, :SimVis_setTextObject)
        else
            setTextObject = nothing
        end

        new(Any[], Ptr{Nothing}[], false, host, port, sync,
            directory, dll_name, isProfessionalEdition, isNoRenderer,
            dll_handle, init, shutdown, getObjectID, freeObjectID, setTime,
            setBaseObject, setFileObject, setTextObject)
    end
end


struct ProfessionalEdition <: Modia3D.AbstractDLR_VisualizationRenderer
    simVis::SimVis_Renderer

    ProfessionalEdition(info; host="127.0.0.1",port=11000,sync=false) =
        new( SimVis_Renderer(info, host=host,port=port,sync=sync) )
end


struct CommunityEdition <: Modia3D.AbstractDLR_VisualizationRenderer
    simVis::SimVis_Renderer

    CommunityEdition(info; host="127.0.0.1",port=11000,sync=false) =
        new( SimVis_Renderer(info, host=host,port=port,sync=sync,professionalEdition=false) )
end


const UndefinedDirectory = "???"

function getSimVisInfo()
    dll_name = ""
    isProfessionalEdition = false

    # Get directory of SimVis2.exe
    if haskey(ENV, "DLR_VISUALIZATION")
        directory = ENV["DLR_VISUALIZATION"]
    else
        directory = UndefinedDirectory
        println("\n... Info: ENV[\"DLR_VISUALIZATION\"] is not defined.\n" *
                "          Therefore, no online renderer is used in Modia3D.\n")
        return (directory,"???",false,true)
    end

    # Check for 64 bit
    if Base.Sys.WORD_SIZE != 64
        @warn "DLR Visualization library only supported for 64-bit system but not for $(Base.Sys.WORD_SIZE) bit.\n" *
              "\nNo Renderer is used in Modia3D (so, animation is switched off)."
        return (directory,"???",false,true)
    end

    # Check whether professional or community edition or on windows or on linux
    if ISWINDOWS()
        dll_name1 = joinpath(directory, "windows", "SimVisInterface_ProfessionalEdition.dll")
        if isfile( dll_name1 )
            dll_name = dll_name1
            isProfessionalEdition = true
        else
            dll_name2 = joinpath(directory, "windows", "SimVisInterface_CommunityEdition.dll")
            if isfile( dll_name2 )
                dll_name = dll_name2
                isProfessionalEdition = false
            else
                @warn "\nModia3D: DLL of DLR-Visualization library not found. Neither of these files\n" *
                      "   $dll_name1, \n" *
                      "   $dll_name2, \n" *
                      "exists. Check whether ENV[\"DLR_VISUALIZATION\"] is correct." *
                      "\nNo Renderer is used in Modia3D (so, animation is switched off)."
                return (directory,"???",false,true)
            end
        end

    elseif ISLINUX()
        dll_name1 = joinpath(directory, "linux", "SimVisInterface_ProfessionalEdition.so")
        if isfile( dll_name1 )
            dll_name = dll_name1
            isProfessionalEdition = true
        else
            dll_name2 = joinpath(directory, "linux", "SimVisInterface_CommunityEdition.so")
            if isfile( dll_name2 )
                dll_name = dll_name2
                isProfessionalEdition = false
            else
                @warn "\nModia3D: *.so of DLR-Visualization library not found. Neither of these files\n" *
                      "   $dll_name1, \n" *
                      "   $dll_name2, \n" *
                      "exists. Check whether ENV[\"DLR_VISUALIZATION\"] is correct." *
                      "\nNo Renderer is used in Modia3D (so, animation is switched off)."
                return (directory, "???", false, true)
            end
        end
    else
        @warn "\nModia3D: DLR Visualization library only supported for Windows or Linux.\n" *
              "\nNo Renderer is used in Modia3D (so, animation is switched off)."
        return (directory, "???", false, true)
    end

    # Try to open the found DLL/SO
    dll = Libdl.dlopen_e(dll_name)
    if dll != C_NULL
        Libdl.dlclose(dll)
    else
        @warn "\nModia3D: DLR Visualization interface library:" *
              "\n   $dll_name" *
              "\nexists, but could not be opened with Libdl.dlopen_e." *
              "\nNo Renderer is used in Modia3D (so, animation is switched off)."
        return (directory, dll_name, false, true)
    end

    # Print info message
    if isProfessionalEdition
        println("   Renderer: Professional edition of the DLR_Visualization library.\n",
                "             (", dll_name, ")")

    else
        println("   Renderer: Community edition of the DLR_Visualization library",
                "\n             (", dll_name, ",",
                "\n              -> the renderer supports only a subset of the Modia3D functionality).\n")
    end

    return (directory, dll_name, isProfessionalEdition, false)
end
