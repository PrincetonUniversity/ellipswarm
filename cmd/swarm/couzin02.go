package main

import (
	"math"
	"math/rand"

	"github.com/PrincetonUniversity/ellipswarm"
)

// UpdateCouzin02 updates the state of a particle based on its neighborhood
// based on the model published in Couzin et al. 2002.
func UpdateCouzin02(speed, zor, zoo, zoa, blind, maxTurn, σ float64, control, active bool) func(*ellipswarm.Particle, *ellipswarm.Simulation) {
	return func(p *ellipswarm.Particle, s *ellipswarm.Simulation) {
		var nr, no, na float64
		var vr, vo, va, vnew ellipswarm.Vec2
		for _, q := range s.Swarm {
			d := s.Env.Dist(p.Pos, q.Pos)
			if d == 0 || (!control && s.Env.Indistinct(0, d, math.Inf(1))) {
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
				nr++
				vr = ellipswarm.Vec2{vr.X - dx, vr.Y - dy}
			case d < zoo:
				no++
				vo = ellipswarm.Vec2{vo.X + q.Vel.X/speed, vo.Y + q.Vel.Y/speed}
			case d < zoa:
				na++
				va = ellipswarm.Vec2{vr.X + dx, vr.Y + dy}
			}
		}

		// handle scary blobs
		var ns float64
		var vs ellipswarm.Vec2
		if active {
			for _, fov := range p.FOV {
				m := ellipswarm.Vec2{0.5 * (fov[0].X + fov[1].X), 0.5 * (fov[0].Y + fov[1].Y)}
				u, v := s.Env.Vec(p.Pos, fov[0]), s.Env.Vec(p.Pos, fov[1])
				w := s.Env.Vec(p.Pos, m)
				r := math.Hypot(w.X, w.Y)
				θ := math.Atan2(w.Y, w.X)
				dx, dy := w.X/r, w.Y/r
				φ := math.Abs(diffAngle(math.Atan2(u.Y, u.X), math.Atan2(v.Y, v.X)))
				if s.Behavior.Attractivity(φ, r, θ) < 0 {
					ns++
					vs = ellipswarm.Vec2{vs.X - dx, vs.Y - dy}
				}
			}
		}

		if ns > 0 {
			vnew = ellipswarm.Vec2{vs.X / ns, vs.Y / ns}
		} else if nr > 0 {
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
