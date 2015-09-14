package main

import (
	"os"
	"path/filepath"
	"reflect"
	"time"

	"github.com/PrincetonUniversity/ellipswarm"
	"github.com/sbinet/go-hdf5"
)

// RunHDF5 runs a simulation and saves data to an HDF5 file.
func RunHDF5(conf *Config, sim *ellipswarm.Simulation) error {
	if err := os.MkdirAll(filepath.Dir(conf.Output), 0755); err != nil {
		return err
	}

	file, err := hdf5.CreateFile(conf.Output, hdf5.F_ACC_TRUNC)
	if err != nil {
		return err
	}
	defer file.Close()

	if err := saveConfig(file, conf); err != nil {
		return err
	}

	dtype, err := hdf5.NewDatatypeFromValue(ellipswarm.Point{})
	if err != nil {
		return err
	}
	defer dtype.Close()

	N := uint(conf.SwarmSize)
	T := uint(conf.Steps)

	dspace, err := hdf5.CreateSimpleDataspace([]uint{T, N}, nil)
	if err != nil {
		return err
	}
	defer dspace.Close()

	if err := dspace.SelectHyperslab([]uint{0, 0}, nil, []uint{1, N}, nil); err != nil {
		return err
	}

	memspace, err := hdf5.CreateSimpleDataspace([]uint{N}, nil)
	if err != nil {
		return err
	}
	defer memspace.Close()

	dset, err := file.CreateDataset("particles", dtype, dspace)
	if err != nil {
		return err
	}
	defer dset.Close()

	for k := uint(0); k < T; k++ {
		if err := dspace.SetOffset([]uint{k, 0}); err != nil {
			return err
		}

		// make a dense array with just particle positions for now
		p := make([]ellipswarm.Point, len(sim.Swarm))
		for i, v := range sim.Swarm {
			p[i] = v.Pos
		}

		if err := dset.WriteSubset(&p, memspace, dspace); err != nil {
			return err
		}

		sim.Step()
	}
	return nil
}

// saveConfig creates a "config" dataset with a null dataspace whose attributes
// reflect the whole configuration plus some other appropriate metadata.
func saveConfig(file *hdf5.File, conf *Config) error {
	null, err := hdf5.CreateDataspace(hdf5.S_NULL)
	if err != nil {
		return err
	}

	anytype, err := hdf5.NewDatatypeFromValue(0)
	if err != nil {
		return err
	}
	defer anytype.Close()

	dset, err := file.CreateDataset("config", anytype, null)
	if err != nil {
		return err
	}
	defer dset.Close()

	dtype, err := hdf5.NewDatatypeFromValue("")
	if err != nil {
		return err
	}
	defer dtype.Close()

	scalar, err := hdf5.CreateDataspace(hdf5.S_SCALAR)
	if err != nil {
		return err
	}

	attr, err := dset.CreateAttribute("Time", dtype, scalar)
	if err != nil {
		return err
	}
	defer attr.Close()

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
			defer dtype.Close()

			attr, err := dset.CreateAttribute(v.Type().Field(i).Name, dtype, scalar)
			if err != nil {
				return err
			}
			defer attr.Close()

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
