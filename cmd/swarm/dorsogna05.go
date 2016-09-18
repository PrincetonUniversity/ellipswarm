package main

import (
	"math"

	"github.com/PrincetonUniversity/ellipswarm"
)

// UpdateDOrsogna05 updates the state of a particle based on its neighborhood
// according to the model published in D'Orsogna et al. 2005 in PRL.
func UpdateDOrsogna05(m, α, β, Cr, lr, Ca, la float64) func(*ellipswarm.Particle, *ellipswarm.Simulation) {
	// gradMorsePotential computes the gradient of the Morse potential around p.
	gradMorsePotential := func(p *ellipswarm.Particle, s *ellipswarm.Simulation) ellipswarm.Vec2 {
		var gradU ellipswarm.Vec2
		for _, q := range s.Swarm {
			// skip self or particles that are too distant
			d := s.Env.Dist(p.Pos, q.Memory.State.Pos)
			if d == 0 || s.Env.Indistinct(0, d, math.Inf(1)) {
				continue
			}

			θ1 := math.Atan2(p.Vel.Y, p.Vel.X)
			θ2 := math.Atan2(q.Memory.State.Vel.Y, q.Memory.State.Vel.X)
			φ := math.Abs(diffAngle(θ1, θ2))

			U := (Cr*math.Exp(-d/lr)/lr - Ca*math.Exp(-d/la)/la) / d
			U += 0.5 * math.Tan(φ/2-math.Pi/4) / d
			gradU.X += U * (q.Memory.State.Pos.X - p.Pos.X)
			gradU.Y += U * (q.Memory.State.Pos.Y - p.Pos.Y)
		}
		return gradU
	}

	// integrateRK4 performs one step of time integration of f.
	integrateRK4 := func(f func(ellipswarm.Vec2) ellipswarm.Vec2, x ellipswarm.Vec2, dt float64) ellipswarm.Vec2 {
		k1 := f(x)
		k2 := f(ellipswarm.Vec2{x.X + dt/2*k1.X, x.Y + dt/2*k1.Y})
		k3 := f(ellipswarm.Vec2{x.X + dt/2*k2.X, x.Y + dt/2*k2.Y})
		k4 := f(ellipswarm.Vec2{x.X + dt*k3.X, x.Y + dt*k3.Y})
		return ellipswarm.Vec2{x.X + dt*(k1.X+2*k2.X+2*k3.X+k4.X)/6, x.Y + dt*(k1.Y+2*k2.Y+2*k3.Y+k4.Y)/6}
	}

	// Return update function.
	return func(p *ellipswarm.Particle, s *ellipswarm.Simulation) {
		gradU := gradMorsePotential(p, s)
		f := func(v ellipswarm.Vec2) ellipswarm.Vec2 {
			n := math.Hypot(v.X, v.Y)
			k := α - β*n*n
			return ellipswarm.Vec2{(k*v.X - gradU.X) / m, (k*v.Y - gradU.Y) / m}
		}
		p.Pos = ellipswarm.Vec2{p.Pos.X + s.Env.Dt*p.Vel.X, p.Pos.Y + s.Env.Dt*p.Vel.Y}
		p.Vel = integrateRK4(f, p.Vel, s.Env.Dt)
		p.State = s.Env.Move(p.Memory.State, p.State)
	}
}
