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
	"github.com/sbinet/go-hdf5"
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
		data.setup(s, conf)
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

	si := socialInfo(s)
	for i, p := range s.Swarm {
		s.Swarm[i].Attractivity = make([]float64, len(p.FOV))
		for j, k := range p.ID {
			s.Swarm[i].Attractivity[j] = si[k+i*conf.SwarmSize]
		}
	}

	return s
}

// setupRandom places particles randomly within an ellipse.
// The distribution of positions is uniform but anisotropic
// (isotropic on a disk that is reshaped to an ellipse).
func setupRandom(s *ellipswarm.Simulation, conf *Config) {
	s.Swarm = make([]ellipswarm.Particle, conf.SwarmSize)
	for i := range s.Swarm {
		r := math.Sqrt(rand.Float64())
		sin, cos := math.Sincos(2 * math.Pi * rand.Float64())
		s.Swarm[i].Pos.X = r * cos * conf.SchoolMajorRadius
		s.Swarm[i].Pos.Y = r * sin * conf.SchoolMinorRadius
		s.Swarm[i].Dir = 2*math.Pi*rand.Float64() - math.Pi
	}
}

// setupLattice places particles on a lattice within an ellipse.
// The lattice is hexagonal with harcoded spacing.
// The distribution of orientations is normal (σ = 0.1 rad).
// The swarm size is defined by the geometry only.
func setupLattice(s *ellipswarm.Simulation, conf *Config) {
	const (
		dx = 1.6 * 1.5
		dy = 0.8 * 1.5
		σ  = 0.1
	)
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
				s.Swarm[i].Pos.X = x
				s.Swarm[i].Pos.Y = y
				s.Swarm[i].Dir = σ * rand.NormFloat64()
				i++
			}
		}
		odd = !odd
	}
	conf.SwarmSize = i
}

type dataReader struct {
	file  *hdf5.File
	px    Dataset
	py    Dataset
	dir   Dataset
	index uint
}

var data *dataReader

func (d *dataReader) setup(s *ellipswarm.Simulation, conf *Config) {
	if d == nil {
		d = new(dataReader)
		var err error
		d.file, err = hdf5.OpenFile(conf.SchoolDataPath, hdf5.F_ACC_RDONLY)
		if err != nil {
			panic(err)
		}
		d.px.Dataset, err = d.file.OpenDataset("px")
		if err != nil {
			panic(err)
		}
		d.px.DataSpace = d.px.Dataset.Space()
		d.py.Dataset, err = d.file.OpenDataset("py")
		if err != nil {
			panic(err)
		}
		d.py.DataSpace = d.py.Dataset.Space()
		d.dir.Dataset, err = d.file.OpenDataset("dir")
		if err != nil {
			panic(err)
		}
		d.dir.DataSpace = d.dir.Dataset.Space()
		dims, _, err := d.dir.DataSpace.SimpleExtentDims()
		if err != nil {
			panic(err)
		}
		start := make([]uint, len(dims))
		count := make([]uint, len(dims))
		copy(count, dims)
		count[0] = 1
		d.px.MemSpace, err = hdf5.CreateSimpleDataspace(count[1:], nil)
		if err != nil {
			panic(err)
		}
		d.py.MemSpace, err = hdf5.CreateSimpleDataspace(count[1:], nil)
		if err != nil {
			panic(err)
		}
		d.dir.MemSpace, err = hdf5.CreateSimpleDataspace(count[1:], nil)
		if err != nil {
			panic(err)
		}
		if err := d.px.DataSpace.SelectHyperslab(start, nil, count, nil); err != nil {
			panic(err)
		}
		if err := d.py.DataSpace.SelectHyperslab(start, nil, count, nil); err != nil {
			panic(err)
		}
		if err := d.dir.DataSpace.SelectHyperslab(start, nil, count, nil); err != nil {
			panic(err)
		}
		//
		r := int(dims[0])
		if r < conf.Replicates {
			conf.Replicates = r
		}
		conf.SwarmSize = int(dims[1])
		s.Swarm = make([]ellipswarm.Particle, conf.SwarmSize)
	}
	// select at index
	if err := d.px.DataSpace.SetOffset([]uint{d.index, 0}); err != nil {
		panic(err)
	}
	if err := d.py.DataSpace.SetOffset([]uint{d.index, 0}); err != nil {
		panic(err)
	}
	if err := d.dir.DataSpace.SetOffset([]uint{d.index, 0}); err != nil {
		panic(err)
	}
	// read data
	pxd := make([]float64, conf.SwarmSize)
	if err := d.px.Dataset.ReadSubset(&pxd, d.px.MemSpace, d.px.DataSpace); err != nil {
		panic(err)
	}
	pyd := make([]float64, conf.SwarmSize)
	if err := d.py.Dataset.ReadSubset(&pyd, d.py.MemSpace, d.py.DataSpace); err != nil {
		panic(err)
	}
	dird := make([]float64, conf.SwarmSize)
	if err := d.dir.Dataset.ReadSubset(&dird, d.dir.MemSpace, d.dir.DataSpace); err != nil {
		panic(err)
	}
	// hack
	dims, _, err := d.dir.DataSpace.SimpleExtentDims()
	if err != nil {
		panic(err)
	}
	conf.SwarmSize = int(dims[0])
	for i := 0; i < conf.SwarmSize; i++ {
		if pxd[i] == 0.0 || pyd[i] == 0.0 {
			conf.SwarmSize = i
			break
		}
	}
	s.Swarm = make([]ellipswarm.Particle, conf.SwarmSize)
	// initialize swarm
	for i := range s.Swarm {
		s.Swarm[i].Pos.X = pxd[i]
		s.Swarm[i].Pos.Y = pyd[i]
		s.Swarm[i].Dir = dird[i]
	}
	d.index++
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
	si := make([]float64, n*n)
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
				si[j+n*i] = 1
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
			si[j+n*i] = 1 / (1 + math.Exp(-β1-β2*LMD-β3*AR))
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
