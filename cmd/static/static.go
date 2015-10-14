// Command static runs static ellipswarm simulations.
//
// Usage
//
// The static command takes one optional argument:
//  static [config_file]
// It is the path to a TOML config file.
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
			Dist: dist,
		},
		Behavior: ellipswarm.Behavior{
			Attractivity: attractivity,
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

	R := 0.4 * conf.DomainSize
	const golden = 0.618
	for i := range s.Swarm {
		r := R * math.Sqrt(rand.Float64())
		sin, cos := math.Sincos(2 * math.Pi * rand.Float64())
		s.Swarm[i].Pos.X = 0.5*conf.DomainSize + r*cos
		s.Swarm[i].Pos.Y = 0.5*conf.DomainSize + r*sin*golden
		s.Swarm[i].Dir = 0.5 * rand.NormFloat64()
		s.Swarm[i].Body.Width = conf.BodyWidth
		s.Swarm[i].Body.Offset = conf.BodyOffset
		s.Swarm[i].Color = [4]float32{1, 1, 0, 1}
	}

	for i := range s.Swarm {
		s.Swarm[i].Detect(s)
	}

	pi := personalInfo(s)
	for i := range s.Swarm {
		// multiply by 2 because pi is rarely greater than 0.5
		s.Swarm[i].Color[1] = float32(pi[i]) * 2
	}

	return s
}

// diffAngle returns the difference between two angles in radians.
// θ and φ must be between -pi and pi. The result is between -pi and pi.
func diffAngle(θ, φ float64) float64 {
	return math.Mod(θ-φ+3*math.Pi, 2*math.Pi) - math.Pi
}

// personalInfo returns for each particle the fraction
// of its field of view reaching outside the swarm.
func personalInfo(s *ellipswarm.Simulation) []float64 {
	pi := make([]float64, len(s.Swarm))
	for i, p := range s.Swarm {
		var c float64
		for _, v := range s.Swarm[i].FOV {
			θ := math.Atan2(v[0].Y-p.Pos.Y, v[0].X-p.Pos.X)
			φ := math.Atan2(v[1].Y-p.Pos.Y, v[1].X-p.Pos.X)
			c += math.Abs(diffAngle(θ, φ))
		}
		c /= 2 * math.Pi
		pi[i] = 1 - c
	}
	return pi
}

// dist is the trivial distance function.
func dist(a, b ellipswarm.Point) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y)
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

// attractivity is a stub attractivity function.
func attractivity(φ, r, θ float64) float64 {
	return 0
}
