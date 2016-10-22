package main

import (
	"math"

	"github.com/PrincetonUniversity/ellipswarm"
)

// UpdateDOrsogna05 updates the state of a particle based on its neighborhood
// based on the model published in D'Orsogna et al. 2005 in PRL.
// WARNING: particles are pre-filtered based on contrast.
func UpdateDOrsogna05(m, α, β, Cr, lr, Ca, la, Co, lo, Cs, ls float64, active bool) func(*ellipswarm.Particle, *ellipswarm.Simulation) {
	// gradMorsePotential computes the gradient of the Morse potential around p.
	gradMorsePotential := func(p *ellipswarm.Particle, s *ellipswarm.Simulation) ellipswarm.Vec2 {
		var gradU ellipswarm.Vec2
		done := make([]bool, len(s.Swarm))
		for i, fov := range p.FOV {
			id := p.ID[i]
			if id < 0 {
				// merged segment
				m := ellipswarm.Vec2{0.5 * (fov[0].X + fov[1].X), 0.5 * (fov[0].Y + fov[1].Y)}
				u, v := s.Env.Vec(p.Pos, fov[0]), s.Env.Vec(p.Pos, fov[1])
				w := s.Env.Vec(p.Pos, m)
				r := math.Hypot(w.X, w.Y)
				θ := math.Atan2(w.Y, w.X)
				dx, dy := w.X/r, w.Y/r
				φ := math.Abs(diffAngle(math.Atan2(u.Y, u.X), math.Atan2(v.Y, v.X)))
				var U float64
				if active && s.Behavior.Attractivity(φ, r, θ) < 0 {
					U = Cs * math.Exp(-r/ls) / ls
				} else {
					U = Cr*math.Exp(-r/lr)/lr - Ca*math.Exp(-r/la)/la
				}
				gradU.X += U * dx
				gradU.Y += U * dy
			} else {
				// particle
				if done[id] {
					continue
				}
				done[id] = true
				q := s.Swarm[id].Memory.State
				d := s.Env.Dist(p.Pos, q.Pos)
				dv := s.Env.Vec(p.Pos, q.Pos)
				dx, dy := dv.X/d, dv.Y/d
				U := Cr*math.Exp(-d/lr)/lr - Ca*math.Exp(-d/la)/la
				gradU.X += U * dx
				gradU.Y += U * dy

				// explicit alignment
				U = -Co * math.Exp(-d/lo) / lo
				θ := math.Atan2(q.Vel.Y, q.Vel.X)
				gradU.X += U * math.Cos(θ)
				gradU.Y += U * math.Sin(θ)
			}
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
