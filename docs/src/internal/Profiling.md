# Profiling

This section summarizes some techniques to determine timing and memory allocation measures for Modia3D.
The various approaches are demonstrated with models from directory
`Modia3D/test/Profiling`.


## @time

The simplest technique is to put `@time` in front of the statement that should be measured, such as:

```julia
# File Modia3D/test/Profiling/BouncingSphere_time.jl

module BouncingSphereSimulation_time

@time begin BouncingSphere = Model3D(
    ...
    )
end

@time bouncingSphere = @instantiateModel(...)
@time simulate!(bouncingSphere,...)
@time plot(bouncingSphere, ...)
end
```

This gives the following output:

```julia
  0.000116 seconds (199 allocations: 12.344 KiB)

Instantiating model Main.BouncingSphereSimulation.BouncingSphere
  1.106054 seconds (2.40 M allocations: 150.386 MiB, 3.92% gc time, 62.47% compilation time)
  0.322050 seconds (579.29 k allocations: 32.774 MiB, 5.81% compilation time)
  0.077613 seconds (1.74 k allocations: 97.844 KiB)
```

The second line
```
  1.106054 seconds (2.40 M allocations: 150.386 MiB, 3.92% gc time, 62.47% compilation time)
```
means the following:

- The total time spent in this statement was 1.106054 seconds
  - 62.47% of this time was used to compile Julia code.
  - 3.92% of this time was used by the *garbage collector*, so was used to find and
    remove allocated memory that is no longer in use.

- There have been 2.5 million allocations of memory. In total 150.386 Mega-Bytes
  have been allocated.

If a @time output contains *gc time* and/or *compilation time*  it might be useful to
just re-run the file, in order that the total time better reflects the execution time.
In a second run *compilation time* should no longer be present.

Modia3D must allocate memory at the beginning of a `simulate!(..)` run for its internal
data structure. After initialization, no memory should be allocated. However, with the `@time`
macro it is not possible to figure out whether this is the case. Use instead option `logTiming=true`,
see next section.


## logTiming

Function `simulate!(..., logTiming=true, ...)` has keyword argument `logTiming`. When set to true,
the result of the `@timeit` macro of the [TimerOutputs](https://github.com/KristofferC/TimerOutputs.jl) package is shown. Function `simulate!(..)` and `Modia3D` are instrumented with this timer.
Example:

```julia
# File Modia3D/test/Profiling/Mobile.jl

module MobileWithLogTiming
...
const enableVisualization = false
const tolerance = 1e-4
...
@time mobile = @instantiateModel(..., logExecution=true, ...)
...
@time simulate!(mobile, ...)
@time simulate!(mobile, log=true, logTiming=true, ...)
end
```

The `@time` macro should be used for `@instantiateModel` and `simulate!` in order to get
information, whether allocation of storage is due to compilation and/or garbage collection.
If the second `simulate!(..)` shows such allocation, the profiling run should be repeated.

`@instantiateModel` should have flag `logExecution=true`, in order that the generated
function `getDerivatives!(..)` is called twice to force compilation of this function.

The `logTiming=true` flag generates the following output in this case:

```
Instantiating model Main.MobileWithLogTiming.Mobile

Execute getDerivatives
First executions of getDerivatives
  1.363270 seconds (548.47 k allocations: 45.652 MiB, 2.47% gc time, 99.56% compilation time)
  0.002773 seconds (18.82 k allocations: 1.157 MiB)
  2.540190 seconds (7.17 M allocations: 264.371 MiB, 2.62% gc time, 53.43% compilation time)
... first simulation:
  8.341341 seconds (37.34 M allocations: 2.236 GiB, 3.36% gc time)
... second simulation:
... Simulate model Mobile
      Initialization at time = 0.0 s
      Termination of Mobile at time = 5.0 s
        cpuTime         = 11.7 s
        allocated       = 2290.0 MiB
        algorithm       = CVODE_BDF
        FloatType       = Float64
        interval        = 0.01 s
        tolerance       = 0.0001 (relative tolerance)
        nStates         = 188
        nResults        = 501
        nGetDerivatives = 1968 (total number of getDerivatives! calls)
        nf              = 1465 (number of getDerivatives! calls from integrator)
        nZeroCrossings  = 0 (number of getDerivatives! calls for zero crossing detection)
        nJac            = 6 (number of Jacobian computations)
        nAcceptedSteps  = 281
        nRejectedSteps  = 3
        nErrTestFails   = 3
        nTimeEvents     = 0
        nStateEvents    = 0
        nRestartEvents  = 0

... Timings for simulation of Mobile:
 ──────────────────────────────────────────────────────────────────────────────────────────────
                                                       Time                   Allocations
                                               ──────────────────────   ───────────────────────
               Tot / % measured:                    11.8s / 99.0%           2.24GiB / 100%

 Section                               ncalls     time   %tot     avg     alloc   %tot      avg
 ──────────────────────────────────────────────────────────────────────────────────────────────
 simulate!                                  1    11.7s   100%   11.7s   2.24GiB  100%   2.24GiB
 solve                                      1    11.7s   100%   11.7s   2.23GiB  100%   2.23GiB
 getDerivatives!                        1.97k    11.7s   100%  5.93ms   2.24GiB  100%   1.16MiB
 LinearEquationsIteration               1.97k    11.6s  99.3%  5.91ms   2.22GiB  99.4%  1.16MiB
 luA ldiv!                              1.97k    4.65s  39.7%  2.36ms   1.68MiB  0.07%     896B
 Modia3D                                 189k    3.56s  30.4%  18.8μs    905KiB  0.04%    4.90B
 Modia3D_2                               185k    2.57s  22.0%  13.9μs   5.16KiB  0.00%    0.03B
 Modia3D_2 computeForcesAndResiduals     185k    638ms  5.45%  3.45μs     0.00B  0.00%    0.00B
 Modia3D_2 computeKinematics!            185k    566ms  4.83%  3.06μs     0.00B  0.00%    0.00B
 Modia3D_1                              1.97k   50.3ms  0.43%  25.6μs   5.16KiB  0.00%    2.68B
 Modia3D_1 computeKinematics!           1.97k   22.4ms  0.19%  11.4μs     0.00B  0.00%    0.00B
 init!                                      1   9.46ms  0.08%  9.46ms   2.03MiB  0.09%  2.03MiB
 Modia3D_1 computeForcesAndResiduals    1.97k   8.00ms  0.07%  4.07μs     0.00B  0.00%    0.00B
 Modia3D_3                              1.97k    584μs  0.00%   297ns     0.00B  0.00%    0.00B
 ODEProblem                                 1   38.2μs  0.00%  38.2μs   2.58KiB  0.00%  2.58KiB
 Modia3D_4 isTerminal                       1    400ns  0.00%   400ns     0.00B  0.00%    0.00B
 ──────────────────────────────────────────────────────────────────────────────────────────────
 11.999957 seconds (37.34 M allocations: 2.237 GiB, 2.67% gc time)
```

The meaning of the first lines

```
First executions of getDerivatives
  1.363270 seconds (548.47 k allocations: 45.652 MiB, 2.47% gc time, 99.56% compilation time)
  0.002773 seconds (18.82 k allocations: 1.157 MiB)
  2.540190 seconds (7.17 M allocations: 264.371 MiB, 2.62% gc time, 53.43% compilation time)
```

is the following:

1. `1.363270 seconds` is the time for the first evaluation of `getDerivatives!`.
   This time is nearly completely used for compilation of this function

2. `0.002773 seconds` is the time for the second evaluation of `getDerivatives!`.
   This time is nearly irrelevant for the timing of `@instantiateModel.`

3. `2.540190` seconds is the total time spent in `@instantiateModel`, including the
   two calls of `getDerivatives!`. This time, together with (1.) shows the following:
   - `0.47*2.5 = 1.1` seconds are used to process the model, generate `getDerivatives!` and
     process `getDerivatives!` twice.
   - `0.53*2.5 = 1.32` seconds are used to compile `getDerivatives!`.


The meaning of column `Section` is the following:

#### @timeit instrumentation of simulate!(..)


| Column Section               | Description |
|:-----------------------------|:------------|
| `simulate!`                  | Time of `simulate!(..)`, but without log outputs. |
| `init!`                      | Time of `init!(..)`, so model initialization.      |
| `ODEProblem`                 | Time of `DifferentialEquations.ODEProblem(..)`, so setup of problem. |
| `solve`                      | Time of `DifferentialEquations.solve(..)`, so after `init!(..)`. |
| `getDerivatives!`            | Time of `getDerivatives!(..)`. |
| `LinearEquationsIteration`   | Time to build and solve linear equation systems inside `getDerivatives!(..)` |
| `luA ldiv!`                  | Time to solve `A*x=b` with `luA` and `ldiv!` inside `LinearEquationsIteration` |


#### @timeit instrumentation of Modia3D

| Column Section                           | Description |
|:-----------------------------------------|:------------|
| `Modia3D`                                | Total time spent in Modia3D functions. |
| `Modia3D_0 initializeVisualization`      | Time to initialize visualization (during init!(..)). |
| `Modia3D_1`                              | Time of `leq_mode == 0` (= compute h(q,qd,gravity)). |
| `Modia3D_1 computeKinematics!`           | Time to compute position, velocity, acceleration with qdd=0. |
| `Modia3D_1 selectContactPairsWithEvent!` | Time to call `selectContactPairsWithEvent!` (collision handling). |
| `Modia3D_1 selectContactPairsNoEvent!`   | Time to call `selectContactPairsNoEvent!` (collision handling). |
| `Modia3D_1 getDistances!`                | Time to call `getDistances!` (collision handling). |
| `Modia3D_1 dealWithContacts!`            | Time to call `dealWithContacts!` (collision handling). |
| `Modia3D_1 computeForcesAndResiduals`    | Time to compute forces/torques/residuals (without collision). |
| `Modia3D_2`                              | Time of `leq_mode > 0` (= compute M(q)*qdd, for unit vectors of qdd). |
| `Modia3D_2 computeKinematics!`           | Time to compute accelerations with qdd = unit vector. |
| `Modia3D_2 computeForcesAndResiduals`    | Time to compute forces/torques/residuals for M(q)*qdd. |
| `Modia3D_3`                              | Time of `leq_mode == -1`. |
| `Modia3D_3 visualize!`                   | Time of `for obj in visualObject3Ds ... visualize(..)`. |
| `Modia3D_3 exportAnimation`              | Time of `for obj in visualObject3Ds ... push!(objectData, dat)`  |
| `Modia3D_4 isTerminal`                   | Time of `exportAnimation` and `closeVisualization` during termination. |


#### How to instrument with @timeit

Further functions in Modia3D can be instrumented in the following way:

The timer of [TimerOutputs](https://github.com/KristofferC/TimerOutputs.jl) is available as
`instantiatedModel.timer`. Therefore, instrumenting a function call is done with:

```julia
TimerOutputs.@timeit instantiatedModel.timer "Modia3D XXX" functionCall(...)
```

or

```julia
TimerOutputs.@timeit instantiatedModel.timer "Modia3D XXX" begin
   statements
end
```



