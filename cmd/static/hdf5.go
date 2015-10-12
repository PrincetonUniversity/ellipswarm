package main

import (
	"io"
	"os"
	"path/filepath"
	"reflect"
	"time"

	"github.com/PrincetonUniversity/ellipswarm"
	"github.com/sbinet/go-hdf5"
)

// A DataPoint is what is recorded in the HDF5 file for each particle at each step.
// This structure is mapped to a compound datatype in HDF5 so member names are important.
type DataPoint struct {
	Pos ellipswarm.Point // position
	Dir float64          // direction (angle in rad)
}

// RunHDF5 runs a simulation and saves data to an HDF5 file.
func RunHDF5(conf *Config, sim *ellipswarm.Simulation) (err error) {
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

	par, err := NewDataset(file, "particles", DataPoint{}, []int{conf.SwarmSize})
	if err != nil {
		return err
	}
	defer checkClose(&err, par)

	p := make([]DataPoint, len(sim.Swarm))
	for i, v := range sim.Swarm {
		p[i].Pos = v.Pos
		p[i].Dir = v.Dir
	}

	if err := par.Dataset.WriteSubset(&p, par.MemSpace, par.DataSpace); err != nil {
		return err
	}

	return nil
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

	v := reflect.ValueOf(conf).Elem()
	for i := 0; i < v.NumField(); i++ {
		err := func() error {
			dtype, err := hdf5.NewDatatypeFromValue(v.Field(i).Interface())
			if err != nil {
				return err
			}
			defer checkClose(&err, dtype)

			attr, err := dset.CreateAttribute(v.Type().Field(i).Name, dtype, scalar)
			if err != nil {
				return err
			}
			defer checkClose(&err, attr)

			if err := attr.Write(v.Field(i).Addr().Interface(), dtype); err != nil {
				return err
			}
			return nil
		}()
		if err != nil {
			return err
		}
	}

	return nil
}

// A Dataset is a wrapper for an HDF5 Dataset and its associated Dataspaces.
type Dataset struct {
	Dataset   *hdf5.Dataset
	DataSpace *hdf5.Dataspace
	MemSpace  *hdf5.Dataspace
}

// NewDataset creates a Dataset.
func NewDataset(file *hdf5.File, name string, valOfType interface{}, dims []int) (*Dataset, error) {
	var d = new(Dataset)
	dtype, err := hdf5.NewDatatypeFromValue(valOfType)
	if err != nil {
		return nil, err
	}
	defer checkClose(&err, dtype)

	udims := make([]uint, len(dims))
	for i, d := range dims {
		udims[i] = uint(d)
	}

	d.DataSpace, err = hdf5.CreateSimpleDataspace(udims, nil)
	if err != nil {
		return nil, err
	}

	start := make([]uint, len(dims))
	count := make([]uint, len(dims))
	copy(count, udims)
	count[0] = 1

	if err := d.DataSpace.SelectHyperslab(start, nil, count, nil); err != nil {
		checkClose(&err, d.DataSpace)
		return nil, err
	}

	if len(dims) == 1 {
		d.MemSpace, err = hdf5.CreateDataspace(hdf5.S_SCALAR)
	} else {
		d.MemSpace, err = hdf5.CreateSimpleDataspace(count[1:], nil)
	}
	if err != nil {
		checkClose(&err, d.DataSpace)
		return nil, err
	}

	d.Dataset, err = file.CreateDataset(name, dtype, d.DataSpace)
	if err != nil {
		checkClose(&err, d.DataSpace)
		checkClose(&err, d.MemSpace)
		return nil, err
	}

	return d, err
}

// Close closes the HDF5 Dataset and Dataspaces.
func (d *Dataset) Close() error {
	if err := d.Dataset.Close(); err != nil {
		return err
	}
	if err := d.MemSpace.Close(); err != nil {
		return err
	}
	if err := d.DataSpace.Close(); err != nil {
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
