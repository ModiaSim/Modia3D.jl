@enum RunTask begin
    visualize
    simulate
    test
end

runTask = simulate


"""
    run(path; task::RunTask=simulate)

Load model and run a simulation task of file specified by `path`. This function sets the global
variable `Modia3D.runTask` as defined by `task` and executes `include(path)`. Afterwards
`Modia3D.runTask` is reset to `simulate`. Other supported simulation tasks are `visualize` for
visualization and `test` for simulation with result checks.
"""
function run(path; task::RunTask=simulate)
    global runTask = task
    include(path)
    global runTask = simulate
    return nothing
end
