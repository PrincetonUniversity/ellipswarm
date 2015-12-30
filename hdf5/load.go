package hdf5

import (
	"fmt"

	"github.com/PrincetonUniversity/ellipswarm"
	"github.com/sbinet/go-hdf5"
)

// A Loader sequentially loads data from an HDF5 dataset.
type Loader struct {
	i uint // index of current slice
	n uint // total number of slices

	data []ellipswarm.State // data buffer

	file   *hdf5.File
	dset   *hdf5.Dataset
	fspace *hdf5.Dataspace
	mspace *hdf5.Dataspace
}

// NewLoader opens a dataset in an HDF5 file and returns an initialized loader.
func NewLoader(filepath, dataset string) (*Loader, error) {
	l := new(Loader)
	var err error
	l.file, err = hdf5.OpenFile(filepath, hdf5.F_ACC_RDONLY)
	if err != nil {
		return nil, err
	}
	l.dset, err = l.file.OpenDataset(dataset)
	if err != nil {
		checkClose(&err, l.file)
		return nil, err
	}
	l.fspace = l.dset.Space()
	dims, _, err := l.fspace.SimpleExtentDims()
	if err != nil {
		checkClose(&err, l.dset)
		checkClose(&err, l.file)
		return nil, err
	}
	if len(dims) != 2 {
		return nil, fmt.Errorf("loader: expected 2 dimensions, got %d", len(dims))
	}
	l.n = dims[0]

	l.mspace, err = hdf5.CreateSimpleDataspace(dims[1:], nil)
	if err != nil {
		checkClose(&err, l.fspace)
		checkClose(&err, l.dset)
		checkClose(&err, l.file)
		return nil, err
	}

	start := []uint{0, 0}
	count := []uint{1, dims[1]}
	if err := l.fspace.SelectHyperslab(start, nil, count, nil); err != nil {
		checkClose(&err, l.mspace)
		checkClose(&err, l.fspace)
		checkClose(&err, l.dset)
		checkClose(&err, l.file)
		return nil, err
	}

	l.data = make([]ellipswarm.State, dims[1])

	return l, nil
}

// Load loads the next batch of data available
// and cycles when everything has already been loaded.
func (l *Loader) Load(s *[]ellipswarm.Particle) error {
	start := []uint{l.i, 0}
	if err := l.fspace.SetOffset(start); err != nil {
		return err
	}
	l.i = (l.i + 1) % l.n

	if err := l.dset.ReadSubset(&l.data, l.mspace, l.fspace); err != nil {
		return err
	}

	// resize s (data valid until first particle at 0,0)
	*s = (*s)[:0]
	for _, p := range l.data {
		if p.Pos.X == 0 && p.Pos.Y == 0 {
			break
		}
		*s = append(*s, ellipswarm.Particle{State: p})
	}

	return nil
}
