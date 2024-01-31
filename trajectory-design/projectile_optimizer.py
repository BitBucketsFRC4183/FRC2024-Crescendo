import casadi.casadi as cs
import opengen as og



x_0 = cs.MX.sym("x_0", NX)
x_ref = cs.MX.sym("x_ref", NX)
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(N)]

# Create the cost function
x_t = x_0
total_cost = 0

for t in range(0, N):
    total_cost += stage_cost(x_t, u_k[t], x_ref)  # update cost
    x_t = dynamics_dt(x_t, u_k[t])  # update state

total_cost += terminal_cost(x_t, x_ref)  # terminal cost

optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
optimization_parameters += [x_0]
optimization_parameters += [x_ref]

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

umin = [-2.0, -1.0] * N  # - cs.DM.ones(NU * N) * cs.inf
umax = [2.0, 1.0] * N  # cs.DM.ones(NU * N) * cs.inf

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_constraints(bounds)


build_config = og.config.BuildConfiguration() \
    .with_build_directory("my_optimizers") \
    .with_build_mode("release") \

meta = og.config.OptimizerMeta() \
    .with_optimizer_name("navigation") \
    .with_authors(["matt_lui"])

solver_config = og.config.SolverConfiguration() \
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config)
builder.build()
