// +build nogl

package main

import (
	"fmt"
	"os"

	"github.com/PrincetonUniversity/ellipswarm"
)

// RunOpenGL returns an error.
func RunOpenGL(conf *Config, s *ellipswarm.Simulation) error {
	return fmt.Errorf("%s was built without OpenGL support\n"+
		"You must specify an output file ('output' key in the config file).", os.Args[0])
}
