package main

import (
	"github.com/BurntSushi/toml"
)

// Config holds the various parameters required for running a simulation.
type Config struct {
	// Output is either a filename (path) for the HDF5 output file,
	// or the empty string for an interactive OpenGL simulation.
	Output string

	SwarmSize int     // number of particles
	Steps     int     // number of time steps (hdf5 only)
	Dt        float64 // duration of time steps

	// Particles parameters
	BodyWidth  float64 // unit: body length
	BodyOffset float64 // unit: 1

	// Swarming model
	Model string // either: "Couzin 2002"" or "D'Orsogna 2005"

	// Simulation mode
	Mode string // possible values: control, passive, active

	// Parameters for "Couzin 2002" model
	Speed      float64 // unit: body length / time
	Zor        float64 // unit: body length
	Zoo        float64 // unit: body length
	Zoa        float64 // unit: body length
	BlindAngle float64 // unit: deg
	MaxTurn    float64 // unit: deg / time
	SDError    float64 // unit: deg

	// Parameters for "D'Orsogna 2005" model
	Mass  float64 // unit: mass
	Alpha float64 // unit: mass / time
	Beta  float64 // unit: mass * time^3 / (body length)^2
	Cr    float64 // unit: 1
	Lr    float64 // unit: body length
	Ca    float64 // unit: 1
	La    float64 // unit: body length
	Co    float64 // unit: 1
	Lo    float64 // unit: body length
	Cs    float64 // unit: 1
	Ls    float64 // unit: body length

	// Visibility and merging of look-alike objects parameters.
	// When using periodic boundary conditions, AttenuationLength
	// must be smaller than:
	//  -DomainSize / (2 * math.Log(2 * MaxContrast / (1 + MaxContrast)))
	AttenuationLength float64 // unit: body length
	ContrastType      string  // possible values: michelson, perfect
	MaxAngle          float64 // unit: rad
	MaxContrast       float64 // unit: 1

	// Boundary conditions parameters
	DomainType string  // possible values: infinite, finite, periodic
	DomainSize float64 // unit: body length

	// Extra computations parameters
	MaxGroupDist float64 // unit: body length
}

// DefaultConf are the default parameters.
var DefaultConf = &Config{
	Output:            "",
	SwarmSize:         75,
	Steps:             10000,
	Dt:                0.1,
	BodyWidth:         0.125,
	BodyOffset:        0.8,
	Model:             "D'Orsogna 2005",
	Mode:              "passive",
	Mass:              1,
	Alpha:             1.2,
	Beta:              0.37,
	Cr:                2,
	Lr:                1,
	Ca:                1,
	La:                4,
	Co:                1,
	Lo:                2,
	Cs:                4,
	Ls:                4,
	AttenuationLength: 3.0,
	ContrastType:      "michelson",
	MaxAngle:          0.01,
	MaxContrast:       0.1,
	DomainType:        "periodic",
	DomainSize:        50,
	MaxGroupDist:      4,
}

// ParseConfig parses the TOML config file whose path is provided.
func ParseConfig(path string) (*Config, error) {
	// config file overwrites default parameters
	conf := DefaultConf
	_, err := toml.DecodeFile(path, conf)
	return conf, err
}
