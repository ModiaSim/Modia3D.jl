# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module 
#   Modia3D.Basics (Modia3D/Basics/_module.jl)
#


"""
    getAndCheckFullLibraryPath(dir,libname)
    
Return joinpath(dir,libname). The returned full path must be a DLL.
It is checked whether this DLL can be opened
"""
function getAndCheckFullLibraryPath(dir,libname)
   full_libname = joinpath(dir,libname)
   
   # Check that library can be opened (in order to get an early error message)
   dll = Base.Libdl.dlopen_e(full_libname)
   if  dll == C_NULL
      error("SimVis: Unable to load DLL ", full_libname)
   else
      # close dll
      Base.Libdl.dlclose(dll)
   end
   
   return full_libname
end


"""
    value = getEnvironmentVariable(name, description)
    
Returns the value of the environment variable `name`. 
If `name` does not exist, an error message is triggered:

    Environment variable <name> not defined
    (= <description>)
"""
function getEnvironmentVariable(name::String, description::String)::String
   if haskey(ENV,name)
      return ENV[name]
   else
      error("\n\nEnvironment variable $name not defined\n(= $description)\n")
   end
end