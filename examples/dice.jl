using ConstrainedDynamics
using ConstrainedDynamicsVis


# Parameters
joint_axis = [1.0;0.0;0.0]

length1 = 0.5
width, depth = 0.5, 0.5
box1 = Box(width, depth, length1, 1., color = RGBA(1., 1., 0.))
b1 = Box(0.1, 0.1, .1, .1, color = RGBA(1., 0., 0.))

#Corner Vector v
v = [length1 / 2; length1 / 2; -length1 / 2]

# Initial orientation
ϕ1 = 0;
q1 = UnitQuaternion(RotX(ϕ1))

# Links
origin = Origin{Float64}()
link1 = Body(box1)

# Constraints

joint0to1 = EqualityConstraint(Floating(origin, link1))
impact = InequalityConstraint(Impact(link1,[0;0;1.0]; p = v))

eqcs = [joint0to1]
ineqcs = [impact]
shapes = [box1]


mech = Mechanism(origin, [link1], eqcs, ineqcs, shapes = shapes)

setPosition!(link1,x = [0.;-2;1.5])

ωtemp = (rand(3) .- 0.5) * 100
setVelocity!(link1,v = [0;3;7.],ω = ωtemp)

storage = simulate!(mech, 10., record = true)
visualize(mech, storage, shapes)
