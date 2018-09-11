#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

using StaticArrays

@static if VERSION >= v"0.7.0-DEV.2005"
    using Libdl
    ISWINDOWS() = Sys.iswindows()
    ISLINUX()   = Sys.islinux()
else
    using Base.Libdl
    ISWINDOWS() = is_windows()
    ISLINUX()   = is_linux()
end


struct SimVisInfo
   directory::String               # Directory Visualization/Extras
   dll_name::String                # Absolute path of SimVis DLL as string 
   isCommercialEdition::Bool       # = true, if DLL is commercial SimVis edition otherwise community edition
   isNoRenderer::Bool              # = true, if SimVis DLL is not available and the NoRenderer renderer is used

   function SimVisInfo()
      dll_name = ""

      # Get directory of SimVis2.exe
      if haskey(ENV, "DLR_VISUALIZATION")
         directory = ENV["DLR_VISUALIZATION"]
      else
         directory = "???"
         @static if VERSION >= v"0.7.0-DEV.2005"
             @warn "\n\nEnvironment variable \"DLR_VISUALIZATION\" not defined.\n" *
                   "Include ENV[\"DLR_VISUALIZATION\"] = <path-to-Visualization/Extras/SimVis> into your HOME/.julia/config/startup.jl file.\n" *
                   "\nNo Renderer is used in Modia3D (so, animation is switched off)."
         else
             warn("\n\nEnvironment variable \"DLR_VISUALIZATION\" not defined.\n",
                  "Include ENV[\"DLR_VISUALIZATION\"] = <path-to-Visualization/Extras/SimVis> into your HOME/.juliarc.jl file.\n",
                  "\nNo Renderer is used in Modia3D (so, animation is switched off).")
         end
         return new(directory,"???",false,true)
      end

      # Check for 64 bit
      if Base.Sys.WORD_SIZE != 64
         @static if VERSION >= v"0.7.0-DEV.2005"
             @warn "DLR Visualization library only supported for 64-bit system but not for $(Base.Sys.WORD_SIZE) bit.\n" *
                   "\nNo Renderer is used in Modia3D (so, animation is switched off)."
         else
             warn("DLR Visualization library only supported for 64-bit system but not for ", Base.Sys.WORD_SIZE, "bit.\n",
                  "\nNo Renderer is used in Modia3D (so, animation is switched off).")
         end
         return new(directory,"???",false,true)
      end

      # Check whether commercial or community edition or on windows or on linux
      if ISWINDOWS()         
         dll_name1 = joinpath(directory, "windows", "SimVisInterface_ProfessionalEdition.dll")
         if isfile( dll_name1 )
            dll_name = dll_name1
            isCommercialEdition = true
         else
            dll_name2 = joinpath(directory, "windows", "SimVisInterface_CommunityEdition.dll")
            if isfile( dll_name2 )
               dll_name = dll_name2
               isCommercialEdition = false
            else
               @static if VERSION >= v"0.7.0-DEV.2005"
                   @warn "\n\nModia3D: DLL of DLR-Visualization library not found. Neither of these files\n" * 
                         "   $dll_name1, \n" *
                         "   $dll_name2, \n" *
                         "exists. Check whether ENV[\"DLR_VISUALIZATION\"] is correct." *
                         "\nNo Renderer is used in Modia3D (so, animation is switched off)."
               else
                   warn("\n\nModia3D: DLL of DLR-Visualization library not found. Neither of these files\n",
                        "   ", dll_name1, "\n",
                        "   ", dll_name2, "\n",
                        "exist. Check whether ENV[\"DLR_VISUALIZATION\"] is correct.",
                        "\nNo Renderer is used in Modia3D (so, animation is switched off).")
               end
               return new(directory,"???",false,true)
            end
         end

      elseif ISLINUX()
         dll_name1 = joinpath(directory, "linux", "SimVisInterface_ProfessionalEdition.so")
         if isfile( dll_name1 )
            dll_name = dll_name1
            isCommercialEdition = true
         else
            dll_name2 = joinpath(directory, "linux", "SimVisInterface_CommunityEdition.so")
            if isfile( dll_name2 )
               dll_name = dll_name2
               isCommercialEdition = false
            else
               @static if VERSION >= v"0.7.0-DEV.2005"
                   @warn "\n\nModia3D: *.so of DLR-Visualization library not found. Neither of these files\n" * 
                         "   $dll_name1, \n" *
                         "   $dll_name2, \n" *
                         "exists. Check whether ENV[\"DLR_VISUALIZATION\"] is correct." *
                         "\nNo Renderer is used in Modia3D (so, animation is switched off)."
               else
                   warn("\n\nModia3D: *.so of DLR-Visualization library not found. Neither of these files\n",
                        "   ", dll_name1, "\n",
                        "   ", dll_name2, "\n",
                        "exist. Check whether ENV[\"DLR_VISUALIZATION\"] is correct.",
                        "\nNo Renderer is used in Modia3D (so, animation is switched off).")
               end
               return new(directory,"???",false,true)
            end
         end
      else
         @static if VERSION >= v"0.7.0-DEV.2005"
             @warn "\n\nModia3D: DLR Visualization library only supported for Windows or Linux.\n" *
                   "\nNo Renderer is used in Modia3D (so, animation is switched off)."
         else
             warn("\n\nModia3D: DLR Visualization library only supported for Windows or Linux.\n",
                  "\nNo Renderer is used in Modia3D (so, animation is switched off).")
         end
         return new(directory,"???",false,true)
      end

      # Try to open the found DLL/SO
      dll = Libdl.dlopen_e(dll_name)
      if dll != C_NULL
         Libdl.dlclose(dll)
      else
         @static if VERSION >= v"0.7.0-DEV.2005"
             @warn "\n\nModia3D: DLR Visualization interface library:" *
                   "\n   $dll_name" * 
                   "\nexists, but could not be opened with Libdl.dlopen_e." *
                   "\nNo Renderer is used in Modia3D (so, animation is switched off)."
         else
             warn("\n\nModia3D: DLR Visualization interface library:",
                  "\n   ", dll_name, 
                  "\nexist, but could not be opened with Libdl.dlopen_e.",
                  "\nNo Renderer is used in Modia3D (so, animation is switched off).")
         end
         return new(directory,dll_name,false,true)
      end

      # Print info message
      if isCommercialEdition
         println("   Renderer: Commercial edition of the DLR_Visualization library.\n",
                 "             (", dll_name, ")")
         
      else
         println("   Renderer: Community edition of the DLR_Visualization library",
               "\n             (", dll_name, ",", 
               "\n              -> the renderer supports only a subset of the Modia3D functionalities).\n")
      end

      new(directory, dll_name, isCommercialEdition, false)   
   end 
end


const simVisInfo = SimVisInfo()

if simVisInfo.isNoRenderer
   # include nothing
elseif simVisInfo.isCommercialEdition
   include("simvis_commercialEdition.jl") 
   include("simvis_bothEditions.jl") 
else
   include("simvis_communityEdition.jl")
   include("simvis_bothEditions.jl") 
end
