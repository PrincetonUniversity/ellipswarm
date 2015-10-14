package ellipswarm

import (
	"math"
	"math/rand"
	"sort"
)

// A Point is a simple 2D vector.
type Point struct {
	X float64
	Y float64
}

// A Segment is a piece of the visual footprint of a particle to another.
type Segment [2]Point

// An Ellipse contains the parameters of an ellipsoidal particle shape.
type Ellipse struct {
	Width  float64 // width of ellipse (length is 1)
	Offset float64 // offset of center of mass along front radius as a fraction
}

// State contains the full state of a particle.
type State struct {
	Pos Point   // position in body length units
	Dir float64 // direction in radians
}

// Parameters contains the parameters of a particle.
type Parameters struct {
	Speed      float64 // speed in body length per unit time
	Body       Ellipse // parameters of ellipsoidal body shape
	MaxTurn    float64 // maximum turning rate in radians per unit time
	SigmaNoise float64 // standard deviation of random noise per unit time
}

// Memory contains the memory of a particle.
type Memory struct {
	FOV          []Segment // field of view as list of visible segments
	Attractivity []float64 // attractivity of visible segments
	State        []State   // state of neighbors
}

// Style contains the display properties of a particle.
type Style struct {
	Color [4]float32 // RGBA body color
}

// A Particle has a state and a style.
type Particle struct {
	State
	Parameters
	Style
	Memory
}

// Detect computes the visual footprint of the neighbors of a particle.
func (p *Particle) Detect(s *Simulation) {
	p.FOV = make([]Segment, 0, len(p.FOV))
	// p.Memory.State = make([]State, 0, len(p.Memory.State))
	for _, q := range s.Swarm {
		// skip self or particles that are too distant
		d := s.Env.Dist(p.Pos, q.Pos)
		if d == 0 || s.Env.Indistinct(0, d, math.Inf(1)) {
			continue
		}

		// save state of close neighbors
		// if d < zoa {
		// 	p.Memory.State = append(p.Memory.State, q.State)
		// }

		// compute center of ellipse
		φ, b := q.Dir, q.Body.Offset/2
		sinφ, cosφ := math.Sincos(φ)
		c := Point{X: q.Pos.X - b*cosφ, Y: q.Pos.Y - b*sinφ}

		// compute relative position to center in polar coordinates
		r := math.Hypot(c.X-p.Pos.X, c.Y-p.Pos.Y)
		θ := math.Atan2(c.Y-p.Pos.Y, c.X-p.Pos.X)
		sinθ, cosθ := math.Sincos(θ)

		// compute position of extremal visible points relative to the center
		// of the ellipse in polar coordinates
		l, w := 0.5, 0.5*p.Body.Width
		l2, w2 := l*l, w*w
		α := math.Sqrt(-2*l2*w2+(l2+w2)*r*r+(w2-l2)*r*r*math.Cos(2*(θ-φ))) / math.Sqrt2
		β := w*r*math.Cos(θ-φ) - l*w
		sinψ1, cosψ1 := math.Sincos(+2 * math.Atan((α-l*r*math.Sin(θ-φ))/β))
		sinψ2, cosψ2 := math.Sincos(-2 * math.Atan((α+l*r*math.Sin(θ-φ))/β))

		// skip if NaN (point of view might be inside ellipse)
		if math.IsNaN(sinψ1) || math.IsNaN(sinψ2) {
			continue
		}

		// compute absolute position of extremal visible points
		var u Segment
		u[0].X = p.Pos.X + r*cosθ + l*cosφ*cosψ1 + w*sinφ*sinψ1
		u[0].Y = p.Pos.Y + r*sinθ + l*sinφ*cosψ1 - w*cosφ*sinψ1
		u[1].X = p.Pos.X + r*cosθ + l*cosφ*cosψ2 + w*sinφ*sinψ2
		u[1].Y = p.Pos.Y + r*sinθ + l*sinφ*cosψ2 - w*cosφ*sinψ2

		// add visible chunks of segment to the list
		chunks := Chunks{Chunk{Start: 0, Stop: 1}}
		if len(p.FOV) == 0 {
			p.FOV = []Segment{u}
		} else {
			ns := make([]Segment, 0, len(p.FOV)+1)
			for _, v := range p.FOV {
				// add chunks of v that are not occluded by u
				for _, c := range p.visibleChunks(u, v) {
					ns = append(ns, Segment{v.Point(c.Start), v.Point(c.Stop)})
				}

				// add chunks of u that are not occluded by v
				chunks = chunks.Intersect(p.visibleChunks(v, u))
			}
			for _, c := range chunks {
				ns = append(ns, Segment{u.Point(c.Start), u.Point(c.Stop)})
			}
			p.FOV = ns
		}
	}
}

// A Chunk is a reference to a portion of a Segment.
// Start and stop are numbers between 0 and 1 denoting
// positions along the oriented segment.
type Chunk struct {
	Start float64
	Stop  float64
}

// Chunks is a slice of Chunk.
type Chunks []Chunk

// Intersect returns the intersection of two Chunks as Chunks.
func (a Chunks) Intersect(b Chunks) Chunks {
	var out Chunks
	for _, u := range a {
		for _, v := range b {
			switch {
			case u.Start <= v.Start && u.Stop <= v.Stop && u.Stop >= v.Start:
				out = append(out, Chunk{Start: v.Start, Stop: u.Stop})
			case u.Start <= v.Start && u.Stop >= v.Stop:
				out = append(out, v)
			case u.Start >= v.Start && u.Stop >= v.Stop && u.Start <= v.Stop:
				out = append(out, Chunk{Start: u.Start, Stop: v.Stop})
			case u.Start >= v.Start && u.Stop <= v.Stop:
				out = append(out, u)
			}
		}
	}
	return out
}

// visibleChunks returns chunks of segment v that are not occluded by u.
func (p *Particle) visibleChunks(u, v Segment) Chunks {
	// direct intersection between u and v
	x0, y0 := u.Intersect(v)

	// project first point of v onto u
	x1, y1 := u.Intersect(Segment{p.Pos, v[0]})

	// project second point of v onto u
	x2, y2 := u.Intersect(Segment{p.Pos, v[1]})

	// project first point of u onto v
	x3, y3 := v.Intersect(Segment{p.Pos, u[0]})

	// project second point of u onto v
	x4, y4 := v.Intersect(Segment{p.Pos, u[1]})

	// conditions - variable names are meaningful, for instance:
	// v0c means that v[0] is inside the cone formed by p and u
	// v0b means that v[0] is behind u (as seen from p)
	cross := x0 > 0 && x0 < 1 && y0 > 0 && y0 < 1
	v0c, v0b := x1 > 0 && x1 < 1 && y1 > 0, y1 > 0 && y1 < 1
	v1c, v1b := x2 > 0 && x2 < 1 && y2 > 0, y2 > 0 && y2 < 1
	u0c, u0b := x3 > 0 && x3 < 1 && y3 > 0, y3 > 0 && y3 < 1
	u1c, u1b := x4 > 0 && x4 < 1 && y4 > 0, y4 > 0 && y4 < 1

	switch {
	// u occludes v totally
	case v0c && v0b && v1c && v1b:
		return Chunks{}

	// simple intersection
	case cross && v1b && v1c:
		return Chunks{Chunk{0, y0}}
	case cross && v0b && v0c:
		return Chunks{Chunk{y0, 1}}

	// complex intersection
	case cross && v1b && !v1c && u1b:
		return Chunks{Chunk{0, y0}, Chunk{x3, 1}}
	case cross && v1b && !v1c && u0b:
		return Chunks{Chunk{0, y0}, Chunk{x4, 1}}
	case cross && v0b && !v0c && u1b:
		return Chunks{Chunk{0, x3}, Chunk{y0, 1}}
	case cross && v0b && !v0c && u0b:
		return Chunks{Chunk{0, x4}, Chunk{y0, 1}}

	// u splits v
	case u0c && !u0b && u1c && !u1b && x3 < x4:
		return Chunks{Chunk{0, x3}, Chunk{x4, 1}}
	case u0c && !u0b && u1c && !u1b && x3 >= x4:
		return Chunks{Chunk{0, x4}, Chunk{x3, 1}}

	// one edge of v occluded
	case v1c && v1b && !v0c && u0c:
		return Chunks{Chunk{0, x3}}
	case v0c && v0b && !v1c && u0c:
		return Chunks{Chunk{x3, 1}}
	case v1c && v1b && !v0c && u1c:
		return Chunks{Chunk{0, x4}}
	case v0c && v0b && !v1c && u1c:
		return Chunks{Chunk{x4, 1}}

	// no occlusion
	default:
		return Chunks{Chunk{Start: 0, Stop: 1}}
	}
}

// Intersect returns the solution of the equation x*s=y*t
// where s and t are treated as vectors. Might contain NaNs or Infs.
func (s Segment) Intersect(t Segment) (x, y float64) {
	u := Point{s[1].X - s[0].X, s[1].Y - s[0].Y}
	v := Point{t[1].X - t[0].X, t[1].Y - t[0].Y}
	det := u.Y*v.X - u.X*v.Y
	x = ((t[0].Y-s[0].Y)*v.X - (t[0].X-s[0].X)*v.Y) / det
	y = (u.X*(t[0].Y-s[0].Y) - u.Y*(t[0].X-s[0].X)) / det
	return x, y
}

// Point returns a point at position (1-x) * s[0] + x * s[1].
func (s Segment) Point(x float64) Point {
	return Point{
		X: (1-x)*s[0].X + x*s[1].X,
		Y: (1-x)*s[0].Y + x*s[1].Y,
	}
}

// ByAngle is a wrapper to sort segments by angle.
type ByAngle struct {
	s *[]Segment
	p Point
}

// Len returns the length of the slice.
func (b ByAngle) Len() int {
	return len(*b.s)
}

// Less compares the order of two segments.
func (b ByAngle) Less(i, j int) bool {
	u, v := (*b.s)[i][0], (*b.s)[j][0]
	θ := math.Atan2(u.Y-b.p.Y, u.X-b.p.X)
	φ := math.Atan2(v.Y-b.p.Y, v.X-b.p.X)
	return diffAngle(θ, φ) >= 0
}

// Swap swaps two segments.
func (b ByAngle) Swap(i, j int) {
	(*b.s)[i], (*b.s)[j] = (*b.s)[j], (*b.s)[i]
}

// Merge fuses visible segments if the contrast between them is below a threshold.
// λ is the attenuation length.
func (p *Particle) Merge(s *Simulation) {
	// orient segments left-to-right
	for i, u := range p.FOV {
		θ := math.Atan2(u[0].Y-p.Pos.Y, u[0].X-p.Pos.X)
		φ := math.Atan2(u[1].Y-p.Pos.Y, u[1].X-p.Pos.X)
		if diffAngle(θ, φ) < 0 {
			p.FOV[i][0], p.FOV[i][1] = u[1], u[0]
		}
	}

	// sort by angle
	sort.Sort(ByAngle{s: &p.FOV, p: p.Pos})

	// merge
	if len(p.FOV) > 0 {
		ns := make([]Segment, 0, len(p.FOV))
		u := p.FOV[0]
		for _, v := range p.FOV[1:] {
			θ := math.Atan2(u[1].Y-p.Pos.Y, u[1].X-p.Pos.X)
			φ := math.Atan2(v[0].Y-p.Pos.Y, v[0].X-p.Pos.X)
			ψ := math.Abs(diffAngle(θ, φ))
			r1 := math.Hypot(u[1].X-p.Pos.X, u[1].Y-p.Pos.Y)
			r2 := math.Hypot(v[0].X-p.Pos.X, v[0].Y-p.Pos.Y)
			if s.Env.Indistinct(ψ, r1, r2) {
				u[1] = v[1]
				continue
			}
			ns = append(ns, u)
			u = v
		}
		p.FOV = append(ns, u)
	}

	// compute attractivity
	p.Attractivity = make([]float64, len(p.FOV))
	for i, v := range p.FOV {
		m := Point{(v[0].X + v[1].X) / 2, (v[0].Y + v[1].Y) / 2}
		ψ1 := math.Atan2(v[0].Y-p.Pos.Y, v[0].X-p.Pos.X)
		ψ2 := math.Atan2(v[1].Y-p.Pos.Y, v[1].X-p.Pos.X)

		r := math.Hypot(m.X-p.Pos.X, m.Y-p.Pos.Y)         // relative distance
		θ := math.Atan2(m.Y-p.Pos.Y, m.X-p.Pos.X) - p.Dir // relative direction
		φ := math.Abs(diffAngle(ψ1, ψ2))                  // subtended angle

		p.Attractivity[i] = s.Behavior.Attractivity(φ, r, θ)
	}
}

// Update updates the state of a particle based on its neighborhood.
func (p *Particle) Update(s *Simulation) {
	// choose new dir based on neighbors
	var acc Point
	for i, s := range p.FOV {
		m := Point{(s[0].X + s[1].X) / 2, (s[0].Y + s[1].Y) / 2}
		θ := math.Atan2(m.Y-p.Pos.Y, m.X-p.Pos.X) // relative direction

		// accumulate attraction or repulsion
		x := p.Attractivity[i]
		sin, cos := math.Sincos(θ)
		acc.X += x * cos
		acc.Y += x * sin
	}

	// align with close neighbors
	// for _, s := range p.Memory.State {
	// 	sin, cos := math.Sincos(s.Dir)
	// 	const strength = 0.1
	// 	acc.X += strength * cos
	// 	acc.Y += strength * sin
	// }

	// desired direction
	dir := math.Atan2(acc.Y, acc.X)
	if (acc.X == 0 && acc.Y == 0) || math.IsNaN(dir) {
		dir = p.Dir
	}

	// add noise
	dir += s.Env.Dt * p.SigmaNoise * rand.NormFloat64()

	// limit turn
	a, max := diffAngle(dir, p.Dir), s.Env.Dt*p.MaxTurn
	switch {
	case a > max:
		dir = p.Dir + max
	case a < -max:
		dir = p.Dir - max
	}

	// move
	sin, cos := math.Sincos(dir)
	old := p.State
	p.Pos.X += s.Env.Dt * p.Speed * cos
	p.Pos.Y += s.Env.Dt * p.Speed * sin
	p.Dir = dir
	p.State = s.Env.Move(old, p.State)
}

// diffAngle returns the difference between two angles in radians.
// θ and φ must be between -pi and pi. The result is between -pi and pi.
func diffAngle(θ, φ float64) float64 {
	return math.Mod(θ-φ+3*math.Pi, 2*math.Pi) - math.Pi
}
