// Command detect computes the detectability around an ellipswarm.
//
// Usage
//
// The detect command takes one optional argument:
//  detect [config_file]
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
	"github.com/PrincetonUniversity/ellipswarm/hdf5"
	"github.com/PrincetonUniversity/ellipswarm/opengl"
)

const usage = `Usage: detect [config_file]

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
					Data: getStates(conf.SwarmSize),
				},
				{
					Name: "detections",
					Val:  BitSet{},
					Dims: []int{conf.GridYcount, conf.GridXcount},
					Data: func(s *ellipswarm.Simulation) interface{} {
						d := detect(s, conf)
						return &d
					},
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

func getStates(size int) func(s *ellipswarm.Simulation) interface{} {
	return func(s *ellipswarm.Simulation) interface{} {
		p := make([]ellipswarm.State, size)
		p = p[:len(s.Swarm)]
		for i, v := range s.Swarm {
			p[i] = v.State
		}
		return &p
	}
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
		setupData(s, conf, &loader)
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

func setupData(s *ellipswarm.Simulation, conf *Config, loader **hdf5.Loader) {
	var err error
	*loader, conf.SwarmSize, err = hdf5.NewLoader(conf.SchoolDataPath, "particles")
	if err != nil {
		panic(err)
	}
	s.Swarm = make([]ellipswarm.Particle, conf.SwarmSize)
	resetData(s, conf, *loader)
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

// A BitSet stores up to 256 boolean values.
type BitSet [4]uint64

// Set sets a bit in a BitSet.
func (b *BitSet) Set(n uint8) {
	b[(n&0xc0)>>6] |= 1 << (n & 0x3f)
}

// detect returns for each point in a grid the number
// of particles which can detect it.
func detect(s *ellipswarm.Simulation, conf *Config) []BitSet {
	d := make([]BitSet, conf.GridXcount*conf.GridYcount)
	for i, y := range linspace(conf.GridYmin, conf.GridYmax, conf.GridYcount) {
		for j, x := range linspace(conf.GridXmin, conf.GridXmax, conf.GridXcount) {
			for k, p := range s.Swarm {
				q := ellipswarm.Vec2{X: x, Y: y}
				if visible(q, p) {
					d[i*conf.GridXcount+j].Set(uint8(k))
				}
			}
		}
	}
	return d
}

// visible returns true if the point is in the field of view of the particle.
func visible(q ellipswarm.Vec2, p ellipswarm.Particle) bool {
	for _, s := range p.FOV {
		// project q onto s
		x, y := s.Intersect(ellipswarm.Segment{p.Pos, q})
		if x > 0 && x < 1 && y > 0 {
			// q in cone formed by p and s
			if y > 0 && y < 1 {
				// q behind s as seen from p
				return false
			}
			return true
		}
	}
	return true
}

// linspace generates count equally spaced points between min and max.
func linspace(min, max float64, count int) []float64 {
	s := make([]float64, count)
	for i := 0; i < count; i++ {
		s[i] = min + (float64(i)/float64(count-1))*(max-min)
	}
	return s
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
