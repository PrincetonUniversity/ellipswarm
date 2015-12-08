// +build nogl

package opengl

import (
	"fmt"
	"os"

	"github.com/PrincetonUniversity/ellipswarm"
)

// Config holds the parameters of the OpenGL driver.
type Config struct {
	// Maximum swarm size.
	MaxSwarmSize int

	// Bounds of default viewport.
	Xmin float64
	Ymin float64
	Xmax float64
	Ymax float64
}

// Run returns an error explaining that OpenGL support is disabled.
func Run(s *ellipswarm.Simulation, conf *Config) error {
	return fmt.Errorf("%s was built without OpenGL support", os.Args[0])
}
