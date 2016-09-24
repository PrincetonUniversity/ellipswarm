// Command swarm runs ellipswarm: vision-based swarming simulations.
//
// Usage
//
// The swarm command takes one optional argument:
//  swarm [config_file]
// It is the path to a TOML config file.
// If no config file is specified, an interactive simulation
// with default parameters will run in an OpenGL window.
//
// Config file
//
// The config file is written in TOML. If you are not familiar with TOML, fear not!
// It's basically a modern version of INI. Very very simple.
// See https://github.com/toml-lang/toml for the full language spec.
//
// Interactive mode
//
// In interactive mode, the simulation can be paused/resumed with space.
// While in pause, pressing right arrow will perform a single step.
// Tab and shift tab allow to cycle through focal individuals.
// Pressing Esc or closing the window will quit.
//
// Periodic boundary condition
//
// When using periodic boundary conditions, the attenuation length has to
// be less than -DomainSize / (2 * log(2 * MaxContrast / (1 + MaxContrast)))
// so that the range of vison is less than half the domain size.
// Otherwise, a particle could potentially see the same particle
// multiple times (or even itself) from different angles and the
// computational cost would explode.
package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"runtime"
	"time"

	"github.com/PrincetonUniversity/ellipswarm"
	"github.com/PrincetonUniversity/ellipswarm/hdf5"
	"github.com/PrincetonUniversity/ellipswarm/opengl"
)

const usage = `Usage: swarm [config_file]

The first argument is optional and is the path to a TOML config file.
If no config file is specified, an interactive simulation
with default parameters will run in an OpenGL window.
`

func init() {
	// Most OpenGL functions have to run from the main thread.
	// This is needed to arrange that main() runs on main thread.
	// See https://github.com/golang/go/wiki/LockOSThread for more info.
	runtime.LockOSThread()

	// We are using the global PRNG so we must seed it here.
	rand.Seed(time.Now().UnixNano())
}

func main() {
	var conf *Config
	var err error
	switch len(os.Args) {
	case 1:
		conf = DefaultConf
	case 2:
		conf, err = ParseConfig(os.Args[1])
	default:
		err = fmt.Errorf("%d arguments provided (0 required, 1 optional)\n\n%s", len(os.Args)-1, usage)
	}
	if err != nil {
		Fatal(err)
	}

	// setup simulation
	s := setup(conf)

	// run interactively or not depending on config
	if conf.Output == "" {
		err = opengl.Run(s, &opengl.Config{
			MaxSwarmSize: conf.SwarmSize,
			Step:         s.Step,
			Xmin:         0,
			Ymin:         0,
			Xmax:         conf.DomainSize,
			Ymax:         conf.DomainSize,
		})
	} else {
		err = hdf5.Run(s, &hdf5.Config{
			Output:       conf.Output,
			Steps:        conf.Steps,
			Step:         s.Step,
			MaxSwarmSize: conf.SwarmSize,
			Datasets: []*hdf5.Dataset{
				{
					Name: "particles",
					Val:  ellipswarm.State{},
					Dims: []int{conf.SwarmSize},
					Data: getStates,
				},
				{
					Name: "groups",
					Val:  0,
					Dims: []int{conf.SwarmSize},
					Data: getGroups(conf.MaxGroupDist),
				},
			},
		})
	}
	if err != nil {
		Fatal(err)
	}
}

// Fatal prints an error on the standard output and exits with a non-zero status.
func Fatal(err error) {
	fmt.Fprintf(os.Stderr, "Error: %s\n", err)
	os.Exit(1)
}

// setup initializes the state and parameters of all particles.
func setup(conf *Config) *ellipswarm.Simulation {
	λmax := -conf.DomainSize / (2 * math.Log(2*conf.MaxContrast/(1+conf.MaxContrast)))
	if conf.AttenuationLength > λmax {
		Fatal(fmt.Errorf("The attenuation length must be smaller than %f", λmax))
	}

	s := &ellipswarm.Simulation{
		Swarm: make([]ellipswarm.Particle, conf.SwarmSize),
		Env: ellipswarm.Environment{
			Dt: conf.Dt,
		},
		Behavior: ellipswarm.Behavior{
			Attractivity: attractivity(conf.AttenuationLength, conf.MaxContrast, conf.BodyWidth),
		},
	}

	switch conf.Model {
	case "Couzin 2002":
		const radPerDeg = math.Pi / 180
		s.Behavior.Update = UpdateCouzin02(conf.Speed, conf.Zor, conf.Zoo, conf.Zoa, conf.BlindAngle*radPerDeg, conf.MaxTurn*radPerDeg, conf.SDError*radPerDeg, conf.Mode == "control", conf.Mode == "active")
	case "D'Orsogna 2005":
		s.Behavior.Update = UpdateDOrsogna05(conf.Mass, conf.Alpha, conf.Beta, conf.Cr, conf.Lr, conf.Ca, conf.La)
	default:
		Fatal(fmt.Errorf("invalid model %q", conf.Model))
	}

	switch conf.ContrastType {
	case "michelson":
		s.Env.Indistinct = indistinct(conf.AttenuationLength, conf.MaxAngle, conf.MaxContrast)
	case "perfect":
		s.Env.Indistinct = alwaysDistinct
	default:
		Fatal(fmt.Errorf("bad contrast type %q", conf.ContrastType))
	}

	switch conf.DomainType {
	case "infinite":
		s.Env.Move = move
		s.Env.Dist = dist
		s.Env.Vec = vec
	case "finite":
		s.Env.Move = reflectiveMove(conf.DomainSize)
		s.Env.Dist = dist
		s.Env.Vec = vec
	case "periodic":
		s.Env.Move = periodicMove(conf.DomainSize)
		s.Env.Dist = periodicDist(conf.DomainSize)
		s.Env.Vec = periodicVec(conf.DomainSize)
	default:
		Fatal(fmt.Errorf("bad domain type %q", conf.DomainType))
	}

	R := 2 * math.Sqrt(float64(conf.SwarmSize)) // estimated radius of swarm
	for i := range s.Swarm {
		r := R * math.Sqrt(rand.Float64())
		sin, cos := math.Sincos(2 * math.Pi * rand.Float64())
		s.Swarm[i].Pos.X = 0.5*conf.DomainSize + r*cos
		s.Swarm[i].Pos.Y = 0.5*conf.DomainSize + r*sin
		// s.Swarm[i].Vel.X = rand.NormFloat64()
		// s.Swarm[i].Vel.Y = rand.NormFloat64()
		θ := 2 * math.Pi * rand.Float64()
		s.Swarm[i].Vel.X = conf.Speed * math.Cos(θ)
		s.Swarm[i].Vel.Y = conf.Speed * math.Sin(θ)
		s.Swarm[i].Body.Width = conf.BodyWidth
		s.Swarm[i].Body.Offset = conf.BodyOffset
		s.Swarm[i].Color = [4]float32{1, 1, 0, 1}
	}

	return s
}

// move is the trivial move function a.k.a. identity.
// It corresponds to an infinite word with no boundaries.
func move(old, new ellipswarm.State) ellipswarm.State {
	return new
}

// reflectiveMove is a move function that works in a square with reflective boundary conditions.
func reflectiveMove(size float64) func(old, new ellipswarm.State) ellipswarm.State {
	return func(old, new ellipswarm.State) ellipswarm.State {
		switch {
		case new.Pos.X < 0:
			new.Pos.X += 2 * -new.Pos.X
			new.Vel.X = -new.Vel.X
		case new.Pos.X > size:
			new.Pos.X -= 2 * (new.Pos.X - size)
			new.Vel.X = -new.Vel.X
		}
		switch {
		case new.Pos.Y < 0:
			new.Pos.Y += 2 * -new.Pos.Y
			new.Vel.Y = -new.Vel.Y
		case new.Pos.Y > size:
			new.Pos.Y -= 2 * (new.Pos.Y - size)
			new.Vel.Y = -new.Vel.Y
		}
		// HACK to limit turn at low speeds
		θ1 := math.Atan2(old.Vel.Y, old.Vel.X)
		θ2 := math.Atan2(new.Vel.Y, new.Vel.X)
		s := math.Hypot(new.Vel.X, new.Vel.Y)
		const MaxTurn = 1.5 // unit: rad/time
		const Dt = 0.1      // unit: time
		δθ, Δθ := diffAngle(θ2, θ1), Dt*MaxTurn*s
		switch {
		case δθ > Δθ:
			θ2 = θ1 + Δθ
		case δθ < -Δθ:
			θ2 = θ1 - Δθ
		}
		y, x := math.Sincos(θ2)
		new.Vel = ellipswarm.Vec2{X: s * x, Y: s * y}
		// END HACK
		return new
	}
}

// diffAngle returns the difference between two angles in radians.
// θ and φ must be between -pi and pi. The result is between -pi and pi.
func diffAngle(θ, φ float64) float64 {
	return math.Mod(θ-φ+3*math.Pi, 2*math.Pi) - math.Pi
}

// periodicMove is a move function that works in a square with periodic boundary conditions.
func periodicMove(size float64) func(old, new ellipswarm.State) ellipswarm.State {
	return func(old, new ellipswarm.State) ellipswarm.State {
		new.Pos.X = math.Mod(new.Pos.X, size)
		if new.Pos.X < 0 {
			new.Pos.X += size
		}
		new.Pos.Y = math.Mod(new.Pos.Y, size)
		if new.Pos.Y < 0 {
			new.Pos.Y += size
		}
		return new
	}
}

// dist is the trivial distance function.
func dist(a, b ellipswarm.Vec2) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y)
}

// periodicDist is a distance that works in a square with periodic boundary conditions.
func periodicDist(size float64) func(a, b ellipswarm.Vec2) float64 {
	return func(a, b ellipswarm.Vec2) float64 {
		x, y := b.X-a.X, b.Y-a.Y
		if 2*x <= -size {
			x += size
		} else if 2*x > size {
			x -= size
		}
		if 2*y <= -size {
			y += size
		} else if 2*y > size {
			y -= size
		}
		return math.Hypot(x, y)
	}
}

// vec is the trivial vector pointing from u to v.
func vec(u, v ellipswarm.Vec2) ellipswarm.Vec2 {
	return ellipswarm.Vec2{v.X - u.X, v.Y - u.Y}
}

// periodicVec returns a vector pointing from u to v for periodic boundary conditions.
func periodicVec(size float64) func(ellipswarm.Vec2, ellipswarm.Vec2) ellipswarm.Vec2 {
	return func(u, v ellipswarm.Vec2) ellipswarm.Vec2 {
		x, y := v.X-u.X, v.Y-u.Y
		if 2*x <= -size {
			x += size
		} else if 2*x > size {
			x -= size
		}
		if 2*y <= -size {
			y += size
		} else if 2*y > size {
			y -= size
		}
		return ellipswarm.Vec2{x, y}
	}
}

// indistinct determines if two objects are indistinguishable given the
// current visibility (attenuation length) and using two thresholds:
// one for the maximum angle between the edges of the two objects and
// one for the maximum Michelson contrast at those edges.
func indistinct(λ, maxAngle, maxContrast float64) func(θ, r1, r2 float64) bool {
	return func(θ, r1, r2 float64) bool {
		v1 := 1 - math.Exp(-r1/λ)
		v2 := 1 - math.Exp(-r2/λ)
		c := math.Abs(v1-v2) / (v1 + v2)
		return math.Abs(θ) < maxAngle && c < maxContrast
	}
}

// alwaysDistinct acts as if two nearby objects were always distinguishable.
func alwaysDistinct(θ, r1, r2 float64) bool {
	return false
}

// attractivity returns a reasonable attractivity function.
func attractivity(λ, maxContrast, bodyWidth float64) func(φ, r, θ float64) float64 {
	return func(φ, r, θ float64) float64 {
		// f returns the expected subtended angle of an ellipse
		// of width bodyWidth at distance r (found with Mathematica)
		// ψ is the relative angle so ψ = π/2 represents the worst case
		f := func(r, ψ float64) float64 {
			r2, w2 := r*r, bodyWidth*bodyWidth
			return math.Acos((r2 - w2 - 1) / math.Sqrt(1+r2*r2+w2*w2-2*w2+2*r2*(w2-1)*math.Cos(2*ψ)))
		}
		switch {
		case r < 1: // short-range repulsion
			return r - 1
		case φ > f(r, math.Pi/2): // scared by large blobs
			return -1
		case r > 4: // long-distance attraction
			return 1
		}
		return 0
	}
}

func getStates(s *ellipswarm.Simulation) interface{} {
	// FIXME: handle varying swarm size
	p := make([]ellipswarm.State, len(s.Swarm))
	for i, v := range s.Swarm {
		p[i] = v.State
	}
	return p
}

func getGroups(maxGroupDist float64) func(s *ellipswarm.Simulation) interface{} {
	return func(s *ellipswarm.Simulation) interface{} {
		id, _ := groupIDs(s, maxGroupDist)
		return id
	}
}

// groupIDs returns for each particle the ID of the group that contains it
// and the total number of groups. Solitary individuals are assigned to group 0.
// Group IDs are always sequential and start at 1.
func groupIDs(s *ellipswarm.Simulation, maxGroupDist float64) ([]int, int) {
	id := make([]int, len(s.Swarm))
	nid := 1
	for i, p := range s.Swarm {
		for j := i + 1; j < len(s.Swarm); j++ {
			if s.Env.Dist(p.State.Pos, s.Swarm[j].State.Pos) <= maxGroupDist {
				if id[i] > 0 && id[j] > 0 && id[i] != id[j] {
					// merge i's group and j's group
					min, max := id[i], id[j]
					if min > max {
						min, max = max, min
					}
					for k, v := range id {
						if v == max {
							id[k] = min
						}
						if v > max {
							id[k]--
						}
					}
					nid--
				} else if id[i] > 0 {
					// attach j to i's group
					id[j] = id[i]
				} else if id[j] > 0 {
					// attach i to j's group
					id[i] = id[j]
				} else {
					// form a new group
					id[i] = nid
					id[j] = nid
					nid++
				}
			}
		}
	}
	return id, nid - 1
}
