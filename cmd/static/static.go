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
	"sort"
	"time"

	"github.com/PrincetonUniversity/ellipswarm"
	"github.com/PrincetonUniversity/ellipswarm/hdf5"
	"github.com/PrincetonUniversity/ellipswarm/opengl"
)

const usage = `Usage: static [config_file]

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
			Step:         func() { reset(s, conf) },
			ForcePause:   true,
			Xmin:         -(conf.SchoolMajorRadius + 1),
			Ymin:         -(conf.SchoolMajorRadius + 1),
			Xmax:         conf.SchoolMajorRadius + 1,
			Ymax:         conf.SchoolMajorRadius + 1,
		})
	} else {
		err = hdf5.Run(s, &hdf5.Config{
			Output:       conf.Output,
			Steps:        conf.Replicates,
			Step:         func() { reset(s, conf) },
			MaxSwarmSize: conf.SwarmSize,
			Datasets: []*hdf5.Dataset{
				{
					Name: "particles",
					Val:  ellipswarm.State{},
					Dims: []int{conf.SwarmSize},
					Data: getStates,
				},
				{
					Name: "personal",
					Val:  0.0,
					Dims: []int{conf.SwarmSize},
					Data: func(s *ellipswarm.Simulation) interface{} { return personalInfo(s) },
				},
				{
					Name: "social",
					Val:  0.0,
					Dims: []int{conf.SwarmSize, conf.SwarmSize},
					Data: func(s *ellipswarm.Simulation) interface{} { return socialInfo(s) },
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

func getStates(s *ellipswarm.Simulation) interface{} {
	// FIXME: handle varying swarm size
	p := make([]ellipswarm.State, len(s.Swarm))
	for i, v := range s.Swarm {
		p[i] = v.State
	}
	return p
}

var loader *hdf5.Loader

// setup initializes the state and parameters of all particles.
func setup(conf *Config) *ellipswarm.Simulation {
	s := &ellipswarm.Simulation{
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

	switch conf.SchoolType {
	case "random":
		setupRandom(s, conf)
	case "lattice":
		setupLattice(s, conf)
	case "data":
		setupData(s, conf, loader)
	default:
		Fatal(fmt.Errorf("bad school type %q", conf.SchoolType))
	}

	for i := range s.Swarm {
		s.Swarm[i].Body.Width = conf.BodyWidth
		s.Swarm[i].Body.Offset = conf.BodyOffset
		s.Swarm[i].Color = [4]float32{1, 0, 0, 1}
	}

	for i := range s.Swarm {
		s.Swarm[i].Detect(s)
	}

	// pi := personalInfo(s)
	// for i := range s.Swarm {
	// 	// multiply by 2 because pi is rarely greater than 0.5
	// 	s.Swarm[i].Color[1] = float32(pi[i]) * 2
	// }

	// si := socialInfo(s)
	// for i, p := range s.Swarm {
	// 	s.Swarm[i].Attractivity = make([]float64, len(p.FOV))
	// 	for j, k := range p.ID {
	// 		s.Swarm[i].Attractivity[j] = si[k+i*conf.SwarmSize]
	// 	}
	// }

	return s
}

// reset reinitializes the state of all particles.
func reset(s *ellipswarm.Simulation, conf *Config) {
	switch conf.SchoolType {
	case "random":
		resetRandom(s, conf)
	case "lattice":
		resetLattice(s, conf)
	case "data":
		resetData(s, conf, loader)
	default:
		Fatal(fmt.Errorf("bad school type %q", conf.SchoolType))
	}
	for i := range s.Swarm {
		s.Swarm[i].Detect(s)
	}
}

// setupRandom initializes the swarm with resetRandom.
func setupRandom(s *ellipswarm.Simulation, conf *Config) {
	s.Swarm = make([]ellipswarm.Particle, conf.SwarmSize)
	resetRandom(s, conf)
}

// resetRandom places particles randomly within an ellipse.
// The distribution of positions is uniform but anisotropic
// (isotropic on a disk that is reshaped to an ellipse).
func resetRandom(s *ellipswarm.Simulation, conf *Config) {
	for i := range s.Swarm {
		r := math.Sqrt(rand.Float64()) * conf.SchoolScale
		sin, cos := math.Sincos(2 * math.Pi * rand.Float64())
		s.Swarm[i].Pos.X = r * cos * conf.SchoolMajorRadius
		s.Swarm[i].Pos.Y = r * sin * conf.SchoolMinorRadius
		vy, vx := math.Sincos(2 * math.Pi * rand.Float64())
		s.Swarm[i].Vel.X = vx
		s.Swarm[i].Vel.Y = vy
	}
}

// setupLattice places particles on a lattice within an ellipse.
// The lattice is hexagonal with harcoded spacing.
// The distribution of orientations is normal (σ = 0.1 rad).
// The swarm size is defined by the geometry only.
func setupLattice(s *ellipswarm.Simulation, conf *Config) {
	const (
		dx = 2.4
		dy = 1.2
		σ  = 0.1
	)
	s.Swarm = make([]ellipswarm.Particle, 0, conf.SwarmSize)
	a, b := conf.SchoolMajorRadius, conf.SchoolMinorRadius
	i, odd := 0, true
	for y := -b; y <= b; y += dy {
		x0 := -a
		if odd {
			x0 += dx / 2
		}
		for x := x0; x <= a; x += dx {
			if (x/a)*(x/a)+(y/b)*(y/b) <= 1 {
				s.Swarm = append(s.Swarm, ellipswarm.Particle{})
				s.Swarm[i].Pos.X = x * conf.SchoolScale
				s.Swarm[i].Pos.Y = y * conf.SchoolScale
				vy, vx := math.Sincos(σ * rand.NormFloat64())
				s.Swarm[i].Vel.X = vx
				s.Swarm[i].Vel.Y = vy
				i++
			}
		}
		odd = !odd
	}
	conf.SwarmSize = i
}

// resetLattice simply randomizes the orientations.
func resetLattice(s *ellipswarm.Simulation, conf *Config) {
	const σ = 0.1
	for i := range s.Swarm {
		vy, vx := math.Sincos(σ * rand.NormFloat64())
		s.Swarm[i].Vel.X = vx
		s.Swarm[i].Vel.Y = vy
	}
}

func setupData(s *ellipswarm.Simulation, conf *Config, loader *hdf5.Loader) {
	var err error
	loader, err = hdf5.NewLoader(conf.SchoolDataPath, "particles")
	if err != nil {
		panic(err)
	}
	s.Swarm = make([]ellipswarm.Particle, conf.SwarmSize)
	resetData(s, conf, loader)
}

func resetData(s *ellipswarm.Simulation, conf *Config, loader *hdf5.Loader) {
	if err := loader.Load(&s.Swarm); err != nil {
		panic(err)
	}
	for i := range s.Swarm {
		s.Swarm[i].Pos.X *= conf.SchoolScale
		s.Swarm[i].Pos.Y *= conf.SchoolScale
		s.Swarm[i].Body.Width = conf.BodyWidth
		s.Swarm[i].Body.Offset = conf.BodyOffset
		s.Swarm[i].Color = [4]float32{1, 0, 0, 1}
	}
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

// socialInfo returns the probability of response matrix.
// It's a square matrix stored in row major order where cell (i,j)
// contains the probability that an event would propagate from j to i.
func socialInfo(s *ellipswarm.Simulation) []float64 {
	// Values taken from Rosenthal et al. (2015) PNAS.
	// In the paper, the scale is in cm with 5cm average body length.
	// Here the scale is in body length so we adjust β1 accordingly.
	// In the paper, β1' = 0.302. Here, β1 = β1' + β2⋅log(5)
	const (
		β1 = -1.9850112735688565
		β2 = -1.421
		β3 = -0.126
	)
	n := len(s.Swarm)
	N := cap(s.Swarm)
	si := make([]float64, N*N)
	for i, p := range s.Swarm {
		id := make([]int, n)
		for k := range id {
			id[k] = k
		}
		angle := make([]float64, n)
		for k, v := range p.FOV {
			θ := math.Atan2(v[0].Y-p.Pos.Y, v[0].X-p.Pos.X)
			φ := math.Atan2(v[1].Y-p.Pos.Y, v[1].X-p.Pos.X)
			angle[p.ID[k]] += math.Abs(diffAngle(θ, φ))
		}
		sort.Sort(IDsByAngle{ID: id, Angle: angle})
		for j, q := range s.Swarm {
			// j startles
			if i == j {
				si[j+N*i] = 0
				continue
			}
			// how far away is j from i
			LMD := math.Log(s.Env.Dist(p.Pos, q.Pos))
			// what's the rank of j in terms of angular area on the retina of i
			AR := 1.0
			for k := 0; id[k] != j; k++ {
				AR++
			}
			// P(i|j) = proba i reacts when j startled
			si[j+N*i] = 1 / (1 + math.Exp(-β1-β2*LMD-β3*AR))
		}
	}
	for i := 0; i < N; i++ {
		j0 := n
		if i >= n {
			j0 = 0
		}
		for j := j0; j < N; j++ {
			si[j+N*i] = 0
		}
	}
	return si
}

// IDsByAngle is a wrapper to sort IDs by decreasing subtended angle.
type IDsByAngle struct {
	ID    []int
	Angle []float64
}

// Len returns the length of the slice.
func (b IDsByAngle) Len() int {
	return len(b.ID)
}

// Less compares the magnitude of two angles.
func (b IDsByAngle) Less(i, j int) bool {
	return b.Angle[i] > b.Angle[j]
}

// Swap swaps two elements.
func (b IDsByAngle) Swap(i, j int) {
	b.ID[i], b.ID[j] = b.ID[j], b.ID[i]
	b.Angle[i], b.Angle[j] = b.Angle[j], b.Angle[i]
}

// dist is the trivial distance function.
func dist(a, b ellipswarm.Vec2) float64 {
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
