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
	Mass       float64 // unit: mass
	Alpha      float64 // unit: mass / time
	Beta       float64 // unit: mass * time^3 / (body length)^2
	Cr         float64 // unit: 1
	Lr         float64 // unit: body length
	Ca         float64 // unit: 1
	La         float64 // unit: body length

	// Environment parameters (cf. D'Orsogna 2005)

	// Visibility and merging of look-alike objects parameters
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

// DefaultConfig are the default parameters.
var DefaultConf = &Config{
	Output:            "",
	SwarmSize:         75,
	Steps:             10000,
	Dt:                0.1,
	BodyWidth:         0.125,
	BodyOffset:        0.8,
	Mass:              1,
	Alpha:             1.2,
	Beta:              0.37,
	Cr:                2,
	Lr:                1,
	Ca:                1,
	La:                4,
	AttenuationLength: 3.0,
	ContrastType:      "michelson",
	MaxAngle:          0.01,
	MaxContrast:       0.1,
	DomainType:        "finite",
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
