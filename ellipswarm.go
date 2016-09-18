// Package ellipswarm runs vision-based swarming simulations of ellipsoidal particles.
//
// A fixed number of ellipsoidal particles interact in a 2D world.
// Particles can see each other but cannot see through each other.
// They follow simple individual laws.
package ellipswarm

// An Environment contains all the parameters relative to the environment.
type Environment struct {
	// Dt is the time step of the simulation
	Dt float64

	// Move validates and canonicalize a move by returning
	// the actual new state given a requested change in state.
	// Coupled with Dist and Vec, it can be use to enforce complex geometries,
	// boundary conditions, and even physical laws of motion.
	Move func(old, new State) State

	// Dist returns the distance between two points. Coupled with Move and Vec,
	// it can be used to create periodic boundary conditions.
	Dist func(a, b Vec2) float64

	// Vec returns the vector pointing from u to v. Coupled with Move and Dist,
	// it can be used to create periodic boundary conditions.
	Vec func(u, v Vec2) Vec2

	// Indistinct returns wether two objects separated by an angle θ and
	// at respective distance r1 and r2 from the observer are indistinguishable
	// given the current visibility conditions.
	Indistinct func(θ, r1, r2 float64) bool
}

// Behavior contains all the parameters relative to the rules followed by particles.
type Behavior struct {
	// Attractivity returns the attractivity of an object of apparent size φ
	// at relative position (r, θ) from the focal particle.
	// A negative value means repulsion. The scale is arbitrary.
	Attractivity func(φ, r, θ float64) float64

	// Update updates the position and velocity of particle p.
	Update func(p *Particle, s *Simulation)
}

// A Simulation contains all the state and parameters of a simulation.
type Simulation struct {
	Swarm    []Particle
	Env      Environment
	Behavior Behavior
}

// Step runs a single simulation step.
func (s *Simulation) Step() {
	for i := range s.Swarm {
		s.Behavior.Update(&s.Swarm[i], s)
	}
	for i := range s.Swarm {
		s.Swarm[i].Detect(s)
		s.Swarm[i].Merge(s)
	}
}
