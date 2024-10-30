using CSV, Plots, DataFrames, Printf

function run_all()
motor = "RS2205s_2300kV_5045prop"
fn = "./data/characterization_" *  motor * ".csv"

data = CSV.read(fn, DataFrame)

data[!, "Motor Speed (kRPM)"] = data[!, "Motor Electrical Speed (RPM)"]/1000
data[!, "Motor Speed (krad/s)"] = data[!, "Motor Speed (kRPM)"] * 2 * π / 60
data[!, "Motor Speed / Max (0..1)"] = data[!, "Motor Speed (krad/s)"] / maximum(data[!, "Motor Speed (krad/s)"])
data[!, "Thrust (N)"] = data[!, "Thrust (kgf)"] * 9.81
data[!, "Overall Efficiency (N/W)"] = data[!, "Thrust (N)"] ./ data[!, "Electrical Power (W)"]
data[!, "Actuator Output (0..1)"] = ((data[!, "ESC signal (µs)"] .- 1000.0) ./ 1000.0)

function fit_data(xs, ys,  c, degree)

  X = zeros(length(xs), degree)
  for i=1:length(xs)
    for d=1:degree
      X[i, d] = xs[i]^d
    end
  end

  Y = ys .- c

  coeffs = X \ Y
  
  println("fin")

  return coeffs

end

function fit_data_exact(xs, ys,  c, degree)

  X = zeros(length(xs), degree)
  for i=1:length(xs)
      X[i, degree] = xs[i]^degree
  end

  Y = ys .- c

  coeffs = X \ Y

  println("fin")

  return coeffs

end

inds = 1000 .<= data[!,  "ESC signal (µs)"] .<= 2000

plots = []


pairs = [
         ("Motor Speed (krad/s)", "ESC signal (µs)", 2, 1000),
         ("Motor Speed / Max (0..1)", "Actuator Output (0..1)", 2, 0),
         ("Motor Speed (krad/s)", "Thrust (N)", 2, 0),
         ("Motor Speed (krad/s)", "Current (A)", 3, 0),
         ("Motor Speed (krad/s)", "Electrical Power (W)", 3, 0),
         ("Motor Speed (krad/s)", "Overall Efficiency (N/W)", 4, 0),
         ("Thrust (N)", "Current (A)", 2, 0)
        ]

all_coeffs = []
for (i, p) in enumerate(pairs)
  xx = p[1]
  yy = p[2]
  d  = p[3]
  c  = p[4]
 
  if i == 3
    coef = fit_data_exact(data[inds, xx], data[inds, yy], c, d)
  else 
    coef = fit_data(data[inds, xx], data[inds, yy], c, d)
  end
  push!(all_coeffs, [c, coef...])
  

  print(xx * " - " * yy * " : " * string([c, coef...]) * "\n")

  plot(size=(600, 400))
  scatter(data[!, xx], data[!, yy], label="Data")
  xd = data[!, xx]
  label = "Fit: y = $(c == 0 ? "" : "$(@sprintf("%.3g", c)) + ")" * "$(@sprintf("%.3g",coef[1]))x" * prod(" + $(@sprintf("%.3g",coef[j]))x^$(j)" for j in 2:d)
  plot!(x -> c + sum(coef[i] * x^i for i=1:d), extrema(xd)..., label = label)
  #plot!(x -> c + coef[1] * x^d,  extrema(xd)..., label = "Fit")
  xlabel!(xx)
  pl = ylabel!(yy)

  push!(plots, pl)
end

N = length(plots)
l = grid(Int(N), 1)
plot(plots..., layout=l, size=(600, 400*N), left_margin = 24Plots.mm)
savefig(motor * ".pdf")


output_fn = motor * "_coeffs.csv"

file = open(output_fn, "w")
for i=1:length(all_coeffs)
  write(file, pairs[i][1] * ", " * pairs[i][2] * ", " * prod(string(c)*"," for c in all_coeffs[i]))
  write(file, "\n")
end
close(file)


## analyse the motor thrust constant

RPM_PER_RADS = 60.0/(2*π)

thrust_constant_N_kRPM2 = fit_data_exact(data[inds, "Motor Speed (kRPM)"], data[inds, "Thrust (N)"], 0, 2)[1]
thrust_constant_N_krads2 = thrust_constant_N_kRPM2 * (RPM_PER_RADS^2)

println("Thrust Constant is $(thrust_constant_N_kRPM2) N / kRPM^2")
println("Thrust Constant is $(thrust_constant_N_krads2) N / (krad/s)^2")

println("Max rpm is $(data[end, "Motor Speed (kRPM)"]) kRPM = $(data[end, "Motor Speed (kRPM)"] / RPM_PER_RADS) krad/s")

end


