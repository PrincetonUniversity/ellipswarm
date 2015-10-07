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
// Known bugs
//
// The periodic boundary condition is not implemented correctly.
// As it is now, it only affect the motion of particles, but not their vision.
// It is a tricky problem as in a periodic world, you can potentially see
// the same particle multiple times from different angles.
//
// The HDF5 outputs currently only contains positions.
package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"runtime"
	"time"

	"github.com/PrincetonUniversity/ellipswarm"
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
	sim := setup(conf)

	// run interactively or not depending on config
	if conf.Output == "" {
		err = RunOpenGL(conf, sim)
	} else {
		err = RunHDF5(conf, sim)
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
	s := &ellipswarm.Simulation{
		Swarm: make([]ellipswarm.Particle, conf.SwarmSize),
		Env: ellipswarm.Environment{
			Dt: conf.Dt,
		},
		Behavior: ellipswarm.Behavior{
			Attractivity: attractivity(conf.AttenuationLength, conf.MaxContrast, conf.BodyWidth),
		},
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
	case "finite":
		s.Env.Move = reflectiveMove(conf.DomainSize)
		s.Env.Dist = dist
	case "periodic":
		s.Env.Move = periodicMove(conf.DomainSize)
		s.Env.Dist = periodicDist(conf.DomainSize)
	default:
		Fatal(fmt.Errorf("bad domain type %q", conf.DomainType))
	}

	R := 2 * math.Sqrt(float64(conf.SwarmSize)) // estimated radius of swarm
	for i := range s.Swarm {
		r := R * math.Sqrt(rand.Float64())
		sin, cos := math.Sincos(2 * math.Pi * rand.Float64())
		s.Swarm[i].Pos.X = 0.5*conf.DomainSize + r*cos
		s.Swarm[i].Pos.Y = 0.5*conf.DomainSize + r*sin
		s.Swarm[i].Vel.X = rand.NormFloat64()
		s.Swarm[i].Vel.Y = rand.NormFloat64()
		s.Swarm[i].Body.Width = conf.BodyWidth
		s.Swarm[i].Body.Offset = conf.BodyOffset
		s.Swarm[i].Mass = conf.Mass
		s.Swarm[i].Alpha = conf.Alpha
		s.Swarm[i].Beta = conf.Beta
		s.Swarm[i].Cr = conf.Cr
		s.Swarm[i].Lr = conf.Lr
		s.Swarm[i].Ca = conf.Ca
		s.Swarm[i].La = conf.La
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
		return new
	}
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
		v := math.Exp(-r / λ)
		c := v / (2 - v)
		switch {
		case c < maxContrast: // max detection distance
			return 0
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
