function setPosition!(body::Body; x::AbstractVector = SA[0;0;0], q::UnitQuaternion = one(UnitQuaternion))
    body.state.xc = x
    body.state.qc = q
    return
end

function setPosition!(body1::Body, body2::Body;
        p1::AbstractVector = SA[0;0;0], p2::AbstractVector = SA[0;0;0], 
        Δx::AbstractVector = SA[0;0;0], Δq::UnitQuaternion = one(UnitQuaternion)
    )

    q1 = body1.state.qc
    q2 = body1.state.qc * Δq
    x2 = body1.state.xc + vrotate(p1 + Δx, q1) - vrotate(p2, q2)
    setPosition!(body2;x = x2,q = q2)
    return
end

function setPosition!(body1::Origin, body2::Body;
        p1::AbstractVector = SA[0;0;0], p2::AbstractVector = SA[0;0;0],
        Δx::AbstractVector = SA[0;0;0], Δq::UnitQuaternion = one(UnitQuaternion)
    )

    q2 = Δq
    x2 = p1 + Δx - vrotate(p2, q2)
    setPosition!(body2;x = x2,q = q2)
    return
end


function setVelocity!(body::Body; v::AbstractVector = SA[0;0;0], ω::AbstractVector = SA[0;0;0])
    body.state.vc = v
    body.state.ωc = ω
    return
end

function setVelocity!(body1::Body, body2::Body;
        p1::AbstractVector = SA[0;0;0], p2::AbstractVector = SA[0;0;0],
        Δv::AbstractVector = SA[0;0;0], Δω::AbstractVector = SA[0;0;0]
    )

    q1 = body1.state.qc
    q2 = body2.state.qc
    
    v1 = body1.state.vc
    ω1 = body1.state.ωc # in local coordinates
    
    vp1 = v1 + vrotate(cross(ω1,p1),q1)
    ωp1 = vrotate(ω1,q1) # in world coordinates

    vp2 = vp1 + vrotate(Δv,q1)
    ωp2 = ωp1 + vrotate(Δω,q2) # in world coordinates

    v2 = vp2 + vrotate(cross(vrotate(ωp2,inv(q2)),-p2),q2)
    ω2 = vrotate(ωp2,inv(q2)) # in local coordinates

    setVelocity!(body2;v = v2,ω = ω2)
    return
end

function setVelocity!(body1::Origin, body2::Body;
        p1::AbstractVector = SA[0;0;0], p2::AbstractVector = SA[0;0;0],
        Δv::AbstractVector = SA[0;0;0], Δω::AbstractVector = SA[0;0;0]
    )
    
    q2 = body2.state.qc

    vp2 = Δv
    ωp2 = vrotate(Δω,q2) # in world coordinates

    v2 = vp2 + cross(ωp2,-p2)
    ω2 = vrotate(ωp2,inv(q2)) # in local coordinates

    setVelocity!(body2;v = v2,ω = ω2)
    return 
end

function setForce!(body::Body; 
        F::AbstractVector = SA[0;0;0], τ::AbstractVector = SA[0;0;0], p::AbstractVector = SA[0;0;0]
    )
    
    τ += vrotate(torqueFromForce(F, p),inv(body.state.qc)) # in local coordinates
    setForce!(body.state, F, τ)
    return 
end

@inline torqueFromForce(F::AbstractVector, p::AbstractVector) = cross(p, F)