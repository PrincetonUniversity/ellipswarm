package main

import (
	"math"

	"github.com/BurntSushi/toml"
)

// Config holds the various parameters required for running a simulation.
type Config struct {
	// Output is either a filename (path) for the HDF5 output file,
	// or the empty string for an interactive OpenGL simulation.
	Output string

	SwarmSize  int // number of particles
	Replicates int // number of replicates

	// Particles parameters
	BodyWidth  float64 // unit: body length
	BodyOffset float64 // unit: 1

	// Visibility and merging of look-alike objects parameters
	AttenuationLength float64 // unit: body length
	ContrastType      string  // possible values: michelson, perfect
	MaxAngle          float64 // unit: rad
	MaxContrast       float64 // unit: 1

	// Boundary conditions parameters
	DomainSize float64 // unit: body length
}

// DefaultConfig are the default parameters.
var DefaultConf = &Config{
	Output:            "",
	SwarmSize:         300,
	Replicates:        100,
	BodyWidth:         0.125,
	BodyOffset:        0.8,
	AttenuationLength: math.Inf(1),
	ContrastType:      "michelson",
	MaxAngle:          0.01,
	MaxContrast:       0.1,
	DomainSize:        30,
}

// ParseConfig parses the TOML config file whose path is provided.
func ParseConfig(path string) (*Config, error) {
	// config file overwrites default parameters
	conf := DefaultConf
	_, err := toml.DecodeFile(path, conf)
	return conf, err
}
