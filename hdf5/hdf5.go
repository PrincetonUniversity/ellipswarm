package hdf5

import (
	"fmt"
	"io"
	"os"
	"path/filepath"
	"time"

	"github.com/PrincetonUniversity/ellipswarm"
	"github.com/sbinet/go-hdf5"
)

// A Dataset stipulates how to generate data and where to store them in the HDF5 file.
type Dataset struct {
	// Name the name of the dataset in the HDF5 file.
	Name string

	// Val is a value of the same concrete type as the underlying type of the data.
	Val interface{}

	// Dims are the dimensions of the data for a single step.
	Dims []int

	// Data is a function that produces the data
	// as a slice of row-major concrete values.
	Data func(s *ellipswarm.Simulation) interface{}

	dset   *hdf5.Dataset
	fspace *hdf5.Dataspace
	mspace *hdf5.Dataspace
}

// Config holds the parameters of the HDF5 driver.
type Config struct {
	Output       string     // path of output file
	Steps        int        // total number of steps
	Step         func()     // go to next step
	MaxSwarmSize int        // maximum swarm size
	Datasets     []*Dataset // list of datasets
}

// Run runs a simulation and saves data to an HDF5 file.
func Run(s *ellipswarm.Simulation, conf *Config) (err error) {
	if err := os.MkdirAll(filepath.Dir(conf.Output), 0755); err != nil {
		return err
	}

	file, err := hdf5.CreateFile(conf.Output, hdf5.F_ACC_TRUNC)
	if err != nil {
		return err
	}
	defer checkClose(&err, file)

	if err := saveConfig(file, conf); err != nil {
		return err
	}

	for _, d := range conf.Datasets {
		if err := d.init(file, conf); err != nil {
			return err
		}
		defer checkClose(&err, d)
	}

	for k := uint(0); k < uint(conf.Steps); k++ {
		// show progress as percentage
		fmt.Printf("\r% 3d%%", 100*k/uint(conf.Steps))

		for _, d := range conf.Datasets {
			start := make([]uint, len(d.Dims)+1)
			start[0] = k
			if err := d.fspace.SetOffset(start); err != nil {
				return err
			}
			if err := d.dset.WriteSubset(d.Data(s), d.mspace, d.fspace); err != nil {
				return err
			}
		}

		conf.Step()
	}
	fmt.Printf("\r100%%\n")
	return nil
}

// A dataPoint is what is recorded in the HDF5 file for each particle at each step.
// This structure is mapped to a compound datatype in HDF5 so member names are important.
type dataPoint struct {
	Pos   ellipswarm.Vec2 // position
	Vel   ellipswarm.Vec2 // velocity
	Group int             // index of the group that contains the particle
}

// saveConfig creates a "config" dataset with a null dataspace whose attributes
// reflect the whole configuration plus some other appropriate metadata.
func saveConfig(file *hdf5.File, conf *Config) (err error) {
	null, err := hdf5.CreateDataspace(hdf5.S_NULL)
	if err != nil {
		return err
	}

	anytype, err := hdf5.NewDatatypeFromValue(0)
	if err != nil {
		return err
	}
	defer checkClose(&err, anytype)

	dset, err := file.CreateDataset("config", anytype, null)
	if err != nil {
		return err
	}
	defer checkClose(&err, dset)

	dtype, err := hdf5.NewDatatypeFromValue("")
	if err != nil {
		return err
	}
	defer checkClose(&err, dtype)

	scalar, err := hdf5.CreateDataspace(hdf5.S_SCALAR)
	if err != nil {
		return err
	}

	attr, err := dset.CreateAttribute("Time", dtype, scalar)
	if err != nil {
		return err
	}
	defer checkClose(&err, attr)

	now := time.Now().String()
	if err := attr.Write(&now, dtype); err != nil {
		return err
	}

	return nil
}

// newDataset creates a dataset.
func (d *Dataset) init(file *hdf5.File, conf *Config) error {
	dtype, err := hdf5.NewDatatypeFromValue(d.Val)
	if err != nil {
		return err
	}
	defer checkClose(&err, dtype)

	udims := make([]uint, len(d.Dims)+1)
	udims[0] = uint(conf.Steps)
	for i, n := range d.Dims {
		udims[i+1] = uint(n)
	}

	d.fspace, err = hdf5.CreateSimpleDataspace(udims, nil)
	if err != nil {
		return err
	}

	start := make([]uint, len(udims))
	count := make([]uint, len(udims))
	copy(count, udims)
	count[0] = 1

	if err := d.fspace.SelectHyperslab(start, nil, count, nil); err != nil {
		checkClose(&err, d.fspace)
		return err
	}

	if len(d.Dims) == 0 {
		d.mspace, err = hdf5.CreateDataspace(hdf5.S_SCALAR)
	} else {
		d.mspace, err = hdf5.CreateSimpleDataspace(udims[1:], nil)
	}
	if err != nil {
		checkClose(&err, d.fspace)
		return err
	}

	d.dset, err = file.CreateDataset(d.Name, dtype, d.fspace)
	if err != nil {
		checkClose(&err, d.fspace)
		checkClose(&err, d.mspace)
	}

	return err
}

// Close closes the HDF5 dataset and Dataspaces.
func (d *Dataset) Close() error {
	if err := d.dset.Close(); err != nil {
		return err
	}
	if err := d.mspace.Close(); err != nil {
		return err
	}
	if err := d.fspace.Close(); err != nil {
		return err
	}
	return nil
}

// checkClose checks for errors in deferred calls.
func checkClose(err *error, c io.Closer) {
	if cerr := c.Close(); *err == nil {
		*err = cerr
	}
}
