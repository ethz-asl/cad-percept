# Geomesh Tools

## filter_center_node
Simple node that loads a mesh, cuts it according to a ROI and stores it again in the given GeoTF file.
Note, meshes are expected to be stored in pseudo-mercator / EPSG:3857 https://epsg.io/3857.

See convert.launch as an example.

In order to obtain such meshes from a digital elevation model / geoTIFF, the following tools 
and steps can be executed.

Tools:
- gdal_warp: Can convert DEMs from one frame to another
- Tin-Terrain: Converts DEM to Mesh https://github.com/heremaps/tin-terrain
- meshconv: Neat tool to convert between obj/off files without loosing too much precision (looking at you, meshlab)

Steps:
- Obtain DEM (e.g. pix4d, geodata4edu, nasa)
- Convert to pseudo mercator: 
  - gdalwarp -t_srs EPSG:3857 input.tif dem_3857.tif
- Convert to mesh with max deviation 0.05 m
  - tin-terrain dem2tin --input dem_3857.tif --output dem_3857.obj --max-error 0.05
- Convert to off file for cgal
  - meshconv dem_3857.obj -c off 
 
 Result
 Note ENU Frame location (rendered axes)
 ![example mesh](https://github.com/ethz-asl/cad-percept/raw/feature/geodetic_meshes/modules/geodetic/cpt_geomesh_tools/mesh_enu.png)