using DifferentialEquations
using LinearAlgebra
using Plots
using Dates
using ProgressMeter

function set_initial_conditions(Nx, Ny, Δx, Δy)
	temperature = zeros(Nx, Ny)
	center_x = (Nx / 2) * Δx
	center_y = (Ny / 2) * Δy
	radius = min(center_x, center_y) / 2
	for j in axes(temperature,2), i in axes(temperature,1)
		x = i * Δx
		y = j * Δy
		distance = hypot(x - center_x, y - center_y)
		temperature[i, j] = exp(-distance / radius) * 10
	end
	return temperature
end

function heat_equation_2d!(dT, T, parameters, time)
	(; Δx, Δy, α) = parameters
	apply_boundary_conditions!(T)
	@inbounds for j in axes(T,2), i in axes(T,1)
		(j == 1 || j == size(T, 2)) && continue
		(i == 1 || i == size(T, 1)) && continue
		dT[i, j] = α * ((T[i+1, j] - 2 * T[i, j] + T[i-1, j]) / Δx^2 +
							 (T[i, j+1] - 2 * T[i, j] + T[i, j-1]) / Δy^2)
	end
	nothing
end

function apply_boundary_conditions!(temperature)
	temperature[1, :] .= 0.0
	temperature[end, :] .= 0.0
	temperature[:, 1] .= 0.0
	temperature[:, end] .= 0.0
	nothing
end

function solve_heat_equation(
	; alpha = 1
	, length_x = 150
	, length_y = 150
	, final_time = 100
	, num_x_points = 150
	, num_y_points = 150
	, delta_x = length_x / (num_x_points)
	, delta_y = length_y / (num_y_points)
	, time_step = 0.1
)

	initial_temperature = set_initial_conditions(num_x_points, num_y_points, delta_x, delta_y)

	parameters = (;α=alpha, Δx=delta_x, Δy=delta_y)
	problem = ODEProblem(heat_equation_2d!, initial_temperature, (0.0, final_time), parameters)

	start_time = now()
	solution = solve(problem, Euler(), dt = time_step, saveat = time_step)
	end_time = now()

	println("Total execution time: $(end_time - start_time)")
	return solution
end

function save_animation(solution)
	min_val = 0
	max_val = 10

    progress = Progress(length(solution.t), 1)
	println("Start saving animation...")
    anim = @animate for i in 1:length(solution.t)
        next!(progress)
        heatmap(solution.u[i], xlabel = "x", ylabel = "y", title = "Temperature Distribution", color = :inferno, clim = (min_val, max_val), aspect_ratio = 1)
    end
	mp4(anim, "heat_equation_2d.mp4", fps = 10)
end

function main()
	solution = solve_heat_equation()
	save_animation(solution)
	return nothing
end

main()