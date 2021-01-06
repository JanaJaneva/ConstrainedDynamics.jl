using ConstrainedDynamics
using ConstrainedDynamicsVis


# Parameters
joint_axis = [1.0;0.0;0.0]

h = 1.
r = .05
box = Cylinder(r, h, h, color = RGBA(1., 0., 0.))

p2 = [0.0;0.0;h / 2] # joint connection point

#Corner vector v
v = [0;r/2;0]

# Initial orientation
ϕ1 = π/2
ϕ2 = 4
ϕ3 = π/2.2
q1 = UnitQuaternion(RotX(ϕ1))
q2 = UnitQuaternion(RotX(ϕ2))
q3 = UnitQuaternion(RotX(ϕ3))

# Links
origin = Origin{Float64}()
link1 = Body(box)
link2 = Body(box)
link3 = Body(box)
link4 = Body(box)

# Constraints
joint0to1 = EqualityConstraint(Floating(origin,link1))
joint1to2 = EqualityConstraint(Fixed(link1,link2;qoffset = q1))
joint2to3 = EqualityConstraint(Fixed(link2,link3;qoffset = q2))
eqcs = [joint0to1;joint1to2;joint2to3;[EqualityConstraint(Fixed(link3,link4; qoffset = q3))]]

impact1 = InequalityConstraint(Impact(link1,[0;0;1.0]; p = v))
impact2 = InequalityConstraint(Impact(link2,[0;0;1.0]; p = v))
impact3 = InequalityConstraint(Impact(link3,[0;0;1.0]; p = v))
impact4 = InequalityConstraint(Impact(link4,[0;0;1.0]; p = v))



links = [link1;link2;link3;link4]
#ineqcs = [impact1;impact2;impact3;impact4]
ineqcs = [impact1]
shapes = [box]


mech = Mechanism(origin, links, eqcs, ineqcs, shapes = shapes)
setPosition!(origin,link2,Δq = q1)

setVelocity!(link1,v=[0;3.0;0])

storage = simulate!(mech, 10., record = true)
visualize(mech, storage, shapes)