// +build !nogl

package opengl

import (
	"fmt"
	"unsafe"

	"github.com/PrincetonUniversity/ellipswarm"
	"github.com/go-gl/gl/v3.3-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
)

// Config holds the parameters of the OpenGL driver.
type Config struct {
	MaxSwarmSize int    // maximum swarm size
	Step         func() // go to next step
	ForcePause   bool   // step manually only?

	// bounds of default viewport
	Xmin float64
	Ymin float64
	Xmax float64
	Ymax float64
}

// Run runs an interactive simulation in an OpenGL window.
func Run(s *ellipswarm.Simulation, conf *Config) error {
	// init GLFW and OpenGL
	if err := glfw.Init(); err != nil {
		return err
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.Samples, 4)
	glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 3)
	glfw.WindowHint(glfw.ContextVersionMinor, 3)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)

	if err := gl.Init(); err != nil {
		return err
	}

	// create OpenGL window
	const (
		title  = "Ellipswarm"
		width  = 800
		height = 800
	)
	w, err := glfw.CreateWindow(width, height, title, nil, nil)
	if err != nil {
		return err
	}
	w.MakeContextCurrent()

	// set background color and enable alpha blending
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
	gl.ClearColor(0, 0, 0, 1)
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	w.SwapBuffers()

	// initialize OpenGL objects
	d, err := newDisplay(conf.MaxSwarmSize)
	if err != nil {
		return err
	}

	// handle scrolling zoom
	vp := viewport{{float32(conf.Xmin), float32(conf.Ymin)}, {float32(conf.Xmax), float32(conf.Ymax)}}
	var focal int // index of the particle whose field of view is displayed
	w.SetScrollCallback(func(w *glfw.Window, xo, yo float64) {
		xc, yc := w.GetCursorPos()
		xs, ys := w.GetSize()
		x, y := float32(xc)/float32(xs), (float32(ys)-float32(yc))/float32(ys)
		dx, dy := vp[1].X-vp[0].X, vp[1].Y-vp[0].Y
		z := 0.05 * float32(yo)
		vp[0].X += z * -(x * dx)
		vp[0].Y += z * -(y * dy)
		vp[1].X += z * (1 - x) * dx
		vp[1].Y += z * (1 - y) * dy
		d.updateViewport(vp)
		d.draw(s, focal, vp)
		w.SwapBuffers()
	})

	var quit, step bool
	pause := conf.ForcePause
	w.SetKeyCallback(func(w *glfw.Window, key glfw.Key, _ int, action glfw.Action, mod glfw.ModifierKey) {
		if key == glfw.KeyEscape && action == glfw.Press {
			quit = true
		}
		if key == glfw.KeySpace && action == glfw.Press && !conf.ForcePause {
			pause = !pause
		}
		if key == glfw.KeyRight && (action == glfw.Press || action == glfw.Repeat) {
			if pause {
				pause = false
				step = true
			}
		}
		if key == glfw.KeyTab && action == glfw.Press {
			// cycle through particles, then disable (focal = -1)
			if mod == glfw.ModShift {
				focal--
			} else {
				focal++
			}
			// FIXME: problematic when len(s.Swarm) < conf.MaxSwarmSize
			focal = (conf.MaxSwarmSize+focal+2)%(conf.MaxSwarmSize+1) - 1
		}
		if key == glfw.KeyR && action == glfw.Press {
			vp = viewport{{float32(conf.Xmin), float32(conf.Ymin)}, {float32(conf.Xmax), float32(conf.Ymax)}}
			d.updateViewport(vp)
			d.draw(s, focal, vp)
			w.SwapBuffers()
		}
	})

	for !(quit || w.ShouldClose()) {
		if step {
			pause = true
			step = false
			conf.Step()
		}
		if !pause {
			conf.Step()
		}
		d.draw(s, focal, vp)
		w.SwapBuffers()
		glfw.PollEvents()
	}
	return nil
}

// A viewport is a rectangle delimiting the area of simulation space shown on screen.
// The first point is the bottom left corner, the second point is the top right corner.
type viewport [2]struct{ X, Y float32 }

// display contains all the OpenGL objects required to display the simulation.
type display struct {
	vao  uint32 // vertex array object
	prog struct {
		ellipse uint32
		fov     uint32
	}
	attr struct {
		pos     uint32
		vel     uint32
		width   uint32
		offset  uint32
		color   uint32
		fpos    uint32
		attract uint32
	}
	buf struct {
		state   uint32 // state
		fov     uint32 // field of view (visible segments)
		attract uint32 // attractivity
	}
	uni struct {
		vp  int32 // viewport
		pos int32 // position of focal particle
	}
}

// draw updates the OpenGL buffers and draws the particles on screen.
func (d *display) draw(s *ellipswarm.Simulation, focal int, vp viewport) {
	d.updateViewport(vp)
	d.updateParticles(s.Swarm)
	if focal >= 0 {
		d.updateFOV(s.Swarm[focal])
		d.updateAttractivity(s.Swarm[focal])
	}

	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	if focal >= 0 {
		d.drawFOV(s.Swarm[focal])
	}
	d.drawParticles(s.Swarm)
}

// updateViewport sends the new viewport to OpenGL.
func (d *display) updateViewport(vp viewport) {
	gl.UseProgram(d.prog.ellipse)
	gl.Uniform2fv(d.uni.vp, 2, &vp[0].X)
	gl.UseProgram(d.prog.fov)
	gl.Uniform2fv(d.uni.vp, 2, &vp[0].X)
}

// updateParticles updates the OpenGL buffer containing particle states.
func (d *display) updateParticles(p []ellipswarm.Particle) {
	gl.BindBuffer(gl.ARRAY_BUFFER, d.buf.state)
	const n = unsafe.Sizeof(ellipswarm.Particle{})
	q := (uintptr)(gl.MapBuffer(gl.ARRAY_BUFFER, gl.WRITE_ONLY))
	if q != 0 {
		for i, v := range p {
			*(*ellipswarm.Particle)(unsafe.Pointer(q + uintptr(i)*n)) = v
		}
		gl.UnmapBuffer(gl.ARRAY_BUFFER)
	}
}

// updateFOV updates the OpenGL buffer containing the field of view
// of the focal particle.
func (d *display) updateFOV(p ellipswarm.Particle) {
	// update focal particle position
	gl.UseProgram(d.prog.fov)
	gl.Uniform2f(d.uni.pos, float32(p.Pos.X), float32(p.Pos.Y))

	// update pos attribute
	gl.BindBuffer(gl.ARRAY_BUFFER, d.buf.fov)
	const n = unsafe.Sizeof(ellipswarm.Segment{})
	ps := (uintptr)(gl.MapBuffer(gl.ARRAY_BUFFER, gl.WRITE_ONLY))
	if ps != 0 {
		for i, v := range p.FOV {
			*(*ellipswarm.Segment)(unsafe.Pointer(ps + uintptr(i)*n)) = v
		}
		gl.UnmapBuffer(gl.ARRAY_BUFFER)
	}
}

// updateAttractivity updates the OpenGL buffer containing the attractivity
// of segments as seen from the focal particle.
func (d *display) updateAttractivity(p ellipswarm.Particle) {
	gl.BindBuffer(gl.ARRAY_BUFFER, d.buf.attract)
	const n = unsafe.Sizeof(float64(0))
	q := (uintptr)(gl.MapBuffer(gl.ARRAY_BUFFER, gl.WRITE_ONLY))
	if q != 0 {
		for i, v := range p.Attractivity {
			*(*float64)(unsafe.Pointer(q + uintptr(2*i)*n)) = v
			*(*float64)(unsafe.Pointer(q + uintptr(2*i+1)*n)) = v
		}
		gl.UnmapBuffer(gl.ARRAY_BUFFER)
	}
}

// drawFOV draws the field of view of the focal particle.
func (d *display) drawFOV(p ellipswarm.Particle) {
	gl.UseProgram(d.prog.fov)
	gl.DrawArrays(gl.LINES, 0, int32(2*len(p.FOV)))
}

// drawParticles draws the ellipsoidal particles.
func (d *display) drawParticles(p []ellipswarm.Particle) {
	gl.UseProgram(d.prog.ellipse)
	gl.DrawArrays(gl.POINTS, 0, int32(len(p)))
}

// newDisplay compiles shaders and initializes a display.
func newDisplay(maxSwarmSize int) (*display, error) {
	d := new(display)

	// compile and link shaders
	var err error
	d.prog.ellipse, err = makeProg([]shader{
		{"Vertex", "ellipse.vert", gl.CreateShader(gl.VERTEX_SHADER)},
		{"Geometry", "ellipse.geom", gl.CreateShader(gl.GEOMETRY_SHADER)},
		{"Fragment", "ellipse.frag", gl.CreateShader(gl.FRAGMENT_SHADER)},
	})
	if err != nil {
		return nil, err
	}
	d.prog.fov, err = makeProg([]shader{
		{"Vertex", "fov.vert", gl.CreateShader(gl.VERTEX_SHADER)},
		{"Geometry", "fov.geom", gl.CreateShader(gl.GEOMETRY_SHADER)},
		{"Fragment", "fov.frag", gl.CreateShader(gl.FRAGMENT_SHADER)},
	})
	if err != nil {
		return nil, err
	}

	// uniform location cannot be specified in the shaders in OpenGL 3.3 core
	// FIXME: d.uni.vp should not be shared by both programs
	// since they are not guaranteed to coincide
	d.uni.vp = gl.GetUniformLocation(d.prog.ellipse, gl.Str("vp\x00"))
	d.uni.pos = gl.GetUniformLocation(d.prog.fov, gl.Str("pos\x00"))

	// attribute locations are specified in the shaders with layout(location=n)
	d.attr = struct{ pos, vel, width, offset, color, fpos, attract uint32 }{0, 1, 2, 3, 4, 5, 6}

	// generate a single VAO for both programs as their interfaces match
	gl.GenVertexArrays(1, &d.vao)
	gl.BindVertexArray(d.vao)

	gl.GenBuffers(1, &d.buf.state)
	gl.BindBuffer(gl.ARRAY_BUFFER, d.buf.state)

	gl.BufferData(gl.ARRAY_BUFFER, maxSwarmSize*int(unsafe.Sizeof(ellipswarm.Particle{})), nil, gl.STREAM_DRAW)

	const n = int32(unsafe.Sizeof(ellipswarm.Particle{}))

	gl.EnableVertexAttribArray(d.attr.pos)
	gl.VertexAttribPointer(d.attr.pos, 2, gl.DOUBLE, false, n, unsafe.Pointer(unsafe.Offsetof(ellipswarm.Particle{}.Pos)))

	gl.EnableVertexAttribArray(d.attr.vel)
	gl.VertexAttribPointer(d.attr.vel, 2, gl.DOUBLE, false, n, unsafe.Pointer(unsafe.Offsetof(ellipswarm.Particle{}.Vel)))

	gl.EnableVertexAttribArray(d.attr.width)
	gl.VertexAttribPointer(d.attr.width, 1, gl.DOUBLE, false, n, unsafe.Pointer(unsafe.Offsetof(ellipswarm.Particle{}.Body)+unsafe.Offsetof(ellipswarm.Ellipse{}.Width)))

	gl.EnableVertexAttribArray(d.attr.offset)
	gl.VertexAttribPointer(d.attr.offset, 1, gl.DOUBLE, false, n, unsafe.Pointer(unsafe.Offsetof(ellipswarm.Particle{}.Body)+unsafe.Offsetof(ellipswarm.Ellipse{}.Offset)))

	gl.EnableVertexAttribArray(d.attr.color)
	gl.VertexAttribPointer(d.attr.color, 4, gl.FLOAT, false, n, unsafe.Pointer(unsafe.Offsetof(ellipswarm.Particle{}.Color)))

	gl.GenBuffers(1, &d.buf.fov)
	gl.BindBuffer(gl.ARRAY_BUFFER, d.buf.fov)
	gl.BufferData(gl.ARRAY_BUFFER, 2*maxSwarmSize*int(unsafe.Sizeof(ellipswarm.Segment{})), nil, gl.STREAM_DRAW)

	gl.EnableVertexAttribArray(d.attr.fpos)
	gl.VertexAttribPointer(d.attr.fpos, 2, gl.DOUBLE, false, int32(unsafe.Sizeof(ellipswarm.Vec2{})), nil)

	gl.GenBuffers(1, &d.buf.attract)
	gl.BindBuffer(gl.ARRAY_BUFFER, d.buf.attract)
	gl.BufferData(gl.ARRAY_BUFFER, 2*maxSwarmSize*int(unsafe.Sizeof(0.0)), nil, gl.STREAM_DRAW)

	gl.EnableVertexAttribArray(d.attr.attract)
	gl.VertexAttribPointer(d.attr.attract, 1, gl.DOUBLE, false, 0, nil)

	gl.BindBuffer(gl.ARRAY_BUFFER, 0)

	return d, nil
}

//go:generate go get github.com/simleb/bindata
//go:generate bindata -s -o shaders.go -r shaders shaders

// A shader wraps an OpenGL shader.
type shader struct {
	name   string
	path   string
	shader uint32
}

// makeProg builds OpenGL programs.
func makeProg(shaders []shader) (uint32, error) {
	var fail bool
	for _, s := range shaders {
		src := bindata[s.path] + "\x00"
		str, free := gl.Strs(src)
		gl.ShaderSource(s.shader, 1, str, nil)
		free()
		gl.CompileShader(s.shader)
		var status int32
		gl.GetShaderiv(s.shader, gl.COMPILE_STATUS, &status)
		if status != gl.TRUE {
			var n int32
			gl.GetShaderiv(s.shader, gl.INFO_LOG_LENGTH, &n)
			log := make([]uint8, n)
			gl.GetShaderInfoLog(s.shader, n, &n, &log[0])
			fmt.Printf("### %s shader compilation error: %s ###\n\n%s\n\n", s.name, s.path, gl.GoStr(&log[0]))
			fail = true
			gl.DeleteShader(s.shader)
		}
	}
	if fail {
		return 0, fmt.Errorf("ellipswarm: GLSL errors")
	}
	prog := gl.CreateProgram()
	for _, s := range shaders {
		gl.AttachShader(prog, s.shader)
	}
	gl.LinkProgram(prog)

	return prog, nil
}
