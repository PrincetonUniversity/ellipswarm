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
	Speed      float64 // unit: body length/time
	BodyWidth  float64 // unit: body length
	BodyOffset float64 // unit: 1
	MaxTurn    float64 // unit: rad/time
	SigmaNoise float64 // unit: rad/time

	// Visibility and merging of look-alike objects parameters
	AttenuationLength float64 // unit: body length
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
	Speed:             1.0,
	BodyWidth:         0.125,
	BodyOffset:        0.8,
	MaxTurn:           1.5,
	SigmaNoise:        0.25,
	AttenuationLength: 2.0,
	MaxAngle:          0.01,
	MaxContrast:       0.1,
	DomainType:        "infinite",
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
