package main

import (
	"math"
	"math/rand"

	"github.com/PrincetonUniversity/ellipswarm"
)

// UpdateCouzin02 updates the state of a particle based on its neighborhood
// based on the model published in Couzin et al. 2002.
func UpdateCouzin02(speed, zor, zoo, zoa, blind, maxTurn, σ float64) func(*ellipswarm.Particle, *ellipswarm.Simulation) {
	return func(p *ellipswarm.Particle, s *ellipswarm.Simulation) {
		var nr, no, na float64
		var vr, vo, va, vnew ellipswarm.Vec2
		for _, q := range s.Swarm {
			d := s.Env.Dist(p.Pos, q.Pos)
			if d == 0 {
				continue
			}
			dv := s.Env.Vec(p.Pos, q.Pos)
			dx, dy := dv.X/d, dv.Y/d

			// skip particles inside blind zone
			α := diffAngle(math.Atan2(-p.Vel.Y, -p.Vel.X), math.Atan2(dy, dx))
			if math.Abs(α) < blind/2 {
				continue
			}

			// handle remaining particles
			switch {
			case d < zor:
				nr += 1
				vr = ellipswarm.Vec2{vr.X - dx, vr.Y - dy}
			case d < zoo:
				no += 1
				vo = ellipswarm.Vec2{vo.X + q.Vel.X/speed, vo.Y + q.Vel.Y/speed}
			case d < zoa:
				na += 1
				va = ellipswarm.Vec2{vr.X + dx, vr.Y + dy}
			}
		}
		if nr > 0 {
			vnew = ellipswarm.Vec2{vr.X / nr, vr.Y / nr}
		} else {
			switch {
			case no == 0 && na == 0:
				vnew = p.Vel
			case no == 0:
				vnew = ellipswarm.Vec2{va.X / na, va.Y / na}
			case na == 0:
				vnew = ellipswarm.Vec2{vo.X / no, vo.Y / no}
			default:
				vnew = ellipswarm.Vec2{0.5*vo.X/no + 0.5*va.X/na, 0.5*vo.Y/no + 0.5*va.Y/na}
			}
		}

		// normalize
		n := math.Hypot(vnew.X, vnew.Y)
		if n < 1e-6 {
			vnew = p.Vel
		} else {
			vnew = ellipswarm.Vec2{speed * vnew.X / n, speed * vnew.Y / n}
		}

		// add noise
		noise := σ * rand.NormFloat64()
		vnew = ellipswarm.Vec2{
			vnew.X*math.Cos(noise) - vnew.Y*math.Sin(noise),
			vnew.X*math.Sin(noise) + vnew.Y*math.Cos(noise),
		}

		// limit turns
		θ := diffAngle(math.Atan2(vnew.Y, vnew.X), math.Atan2(p.Vel.Y, p.Vel.X))
		φ := s.Env.Dt * maxTurn
		if math.Abs(θ) > φ {
			vnew = ellipswarm.Vec2{
				p.Vel.X*math.Cos(φ*θ/math.Abs(θ)) - p.Vel.Y*math.Sin(φ*θ/math.Abs(θ)),
				p.Vel.X*math.Sin(φ*θ/math.Abs(θ)) + p.Vel.Y*math.Cos(φ*θ/math.Abs(θ)),
			}
		}

		p.Pos = ellipswarm.Vec2{p.Pos.X + s.Env.Dt*p.Vel.X, p.Pos.Y + s.Env.Dt*p.Vel.Y}
		p.Vel = vnew
		p.State = s.Env.Move(p.Memory.State, p.State)
	}
}
