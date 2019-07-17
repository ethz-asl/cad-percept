# CPT Selective ICP

Package replacing ethzasl_icp_mapper by a selective ICP algorithm taking a reference map (mesh) and a list of references as input for better alignment.

## Instructions

Use service to set reference facets, otherwise whole model is used:
```
rosservice call /set_ref "data:
- 14
- 1"
```