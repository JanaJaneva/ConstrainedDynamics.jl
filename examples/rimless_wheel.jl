using ConstrainedDynamics
using ConstrainedDynamicsVis


# Parameters
joint_axis = [1.0;0.0;0.0]

h = 1.
r = .05
box = Cylinder(r, h, h, color = RGBA(1., 0., 0.))

p2 = [0.0;0.0;h / 2] # joint connection point

#Corner vector v
v = [0;0;h/2]

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
joint3to4 = EqualityConstraint(Fixed(link3,link4;qoffset = q3))

impact11 = InequalityConstraint(Impact(link1,[0;0;1.0]; p = v)) # Comment for friction
impact21 = InequalityConstraint(Impact(link2,[0;0;1.0]; p = v)) # Comment for friction
impact31 = InequalityConstraint(Impact(link3,[0;0;1.0]; p = v)) # Comment for friction
impact41 = InequalityConstraint(Impact(link4,[0;0;1.0]; p = v)) # Comment for friction
impact12 = InequalityConstraint(Impact(link1,[0;0;1.0]; p = -v)) # Comment for friction
impact22 = InequalityConstraint(Impact(link2,[0;0;1.0]; p = -v)) # Comment for friction
impact32 = InequalityConstraint(Impact(link3,[0;0;1.0]; p = -v)) # Comment for friction
impact42 = InequalityConstraint(Impact(link4,[0;0;1.0]; p = -v)) # Comment for friction

# impact11 = InequalityConstraint(Friction(link1,[0;0.2;1.0],0.4; p = v)) # Uncomment for friction
# impact21 = InequalityConstraint(Friction(link2,[0;0.2;1.0],0.4; p = v)) # Uncomment for friction
# impact31 = InequalityConstraint(Friction(link3,[0;0.2;1.0],0.4; p = v)) # Uncomment for friction
# impact41 = InequalityConstraint(Friction(link4,[0;0.2;1.0],0.4; p = v)) # Uncomment for friction
# impact12 = InequalityConstraint(Friction(link1,[0;0.2;1.0],0.4; p = -v)) # Uncomment for friction
# impact22 = InequalityConstraint(Friction(link2,[0;0.2;1.0],0.4; p = -v)) # Uncomment for friction
# impact32 = InequalityConstraint(Friction(link3,[0;0.2;1.0],0.4; p = -v)) # Uncomment for friction
# impact42 = InequalityConstraint(Friction(link4,[0;0.2;1.0],0.4; p = -v)) # Uncomment for friction



links = [link1;link2;link3;link4]
eqcs = [joint0to1;joint1to2;joint2to3;joint3to4]
ineqcs = [impact11;impact21;impact31;impact41;impact12;impact22;impact32;impact42]
shapes = [box]


mech = Mechanism(origin, links, eqcs, ineqcs, shapes = shapes)

setPosition!(link1,x = [0;0;1.0])
setPosition!(link1,link2,Δq = q1)
setPosition!(link2,link3,Δq = q2)
setPosition!(link3,link4,Δq = q3)

setVelocity!(link1,ω=[-10.0;0.0;0],v=[0;5;0]) # Comment for friction
# setVelocity!(link1,ω=[-1.0;0.0;0],v=[0;0;0]) # Uncomment for friction
setVelocity!(link1,link2)
setVelocity!(link2,link3)
setVelocity!(link3,link4)

storage = simulate!(mech, 10., record = true)
visualize(mech, storage, shapes)