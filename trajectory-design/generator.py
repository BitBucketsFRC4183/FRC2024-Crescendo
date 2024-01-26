import casadi.casadi as cs
import opengen as og


u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = cs.vertcat(1.5 * u[0] - u[1],
               cs.fmax(0.0, u[2] - u[3] + 0.1))

problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c)

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
