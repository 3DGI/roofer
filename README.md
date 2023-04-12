# roofer
Automatic 3D building reconstruction

## Design
* Core functions acessible through documented API
  * I/O functions (optional, eg GDAL, LASlib, proj)
  * Geometry processing functions
  * Validation and quantification functions (eg integration with val3dity)
  * Basic reconstruction pipeline excl I/O.
* Visual debugger built-in (polyscope?), can be disabled using compile flag
* main C++ application built on top of Core API
  * implements complete building reconstruction pipeline including I/O
  * able to scale to large inputs
  * outputs CityJSON (features)

## Goals:
* easy to compile and install on all major platforms. Minimum number of preferably lightweight dependencies. Non essential dependencies should be optional.
* easy to adapt and extend (to support and encourage the exploration of new reconstruction algorithms)
* simple to use, both API and app

## CLI 

```
$ roofer reconstruct --bid x
```

## Related sofware
1. tyler -- for tiling of roofer output and conversion to other formats (obj, 3dtiles, gltf)
2. wallparty -- computation of party walls area + geometry
3. footprint extractor -- derive building footprints directly from lidar point cloud
