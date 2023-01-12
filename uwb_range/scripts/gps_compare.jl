using Printf
using Plots
using Dates
using Geodesy
using Distances
using CSV, DataFrames

BASE = LLA(50.10419809, 14.38930613, 268.524)
GPS = CSV.read("#3.txt", DataFrame)
UWB = CSV.read("test.csv", DataFrame)

START_TIME = UWB."field.stamp"[1]*1e-9

UWB_epoch = UWB."field.stamp"*1e-9
UWB_datetime = []

for value in UWB_epoch
    push!(UWB_datetime, value - START_TIME)
end

distances = []
GPS_datetime = []
for row in eachrow( GPS )
    x = LLA(row.latitude, row.longitude, row."altitude(m)")
    dist = euclidean_distance(BASE, x)
    # dist = haversine([row.latitude, row.longitude], [50.10419809, 14.38930613])
    stamp = DateTime(row."date time", dateformat"yyyy-mm-dd HH:MM:SS.sss")
    datetime2unix(stamp)
    push!(distances, dist)
    push!(GPS_datetime, datetime2unix(stamp) - START_TIME)
end


scatter(GPS_datetime, distances, fmt = :PDF, lims = :round, title="UWB compared to GPS", 
fontfamily="Computer Modern",
titlefontsize=8,
xguide = "time [sec]",
yguide = "Distance [m]",
minorgrid=true,
xlims=(0, UWB_datetime[end]),
guidefontsize=6,
label="GPS",
ms=1, ma=0.5,
color_palette=:lighttest);
scatter!(UWB_datetime, UWB."field.range", label="UWB", mc=:red, ms=1, ma=0.5)
savefig("GPS.pdf")

function main()
    

end

main()