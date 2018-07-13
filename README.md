 [![Dependency Status][Ontologenius-Dependency-Image]][Ontologenius-Dependency-Url]
 [![Dependency Status][Description-Dependency-Image]][Description-Dependency-Url]

# route_verbalization

The route_verbalization is a clean combination of [route_description](https://github.com/LAAS-HRI/route_description) and [semantic_map_drawer](https://github.com/sarthou/semantic_map_drawer) packages to provide a smarter route verbalization.

### Usage

To only get the route verbalization, run the node `verbalize_route` :
```bash
$ rosrun route_verbalization verbalize_route
```

It will provide to services :
 - route_verbalization/verbalizeRegion
 - route_verbalization/verbalizePlace
 > The place verbalization is in progress

### Test

To test the verbalization by automatically querying the semantic_route_description, launch the file `test.launch` :
```
$ roslaunch route_verbalization test.launch
```
Then call the service `testRegion` or `testPlace` :
```
$ rosservice call /route_verbalization/testRegion "from_: 'gina'
to: 'gf_toilets'
persona: 'lambda'
signpost: false"

```

### Verification

To check the consistency of your semantic description of the environment, you can execute the `draw_route` node which will generate images representing each corridor and open space with the elements along them:
```
$ rosrun route_verbalization draw_route
```

[Ontologenius-Dependency-Image]: https://img.shields.io/badge/dependencies-ontoloGenius-1eb0fc.svg
[Ontologenius-Dependency-Url]: https://github.com/sarthou/ontologenius
[Description-Dependency-Image]: https://img.shields.io/badge/dependencies-semantic_route_description-1eb0fc.svg
[Description-Dependency-Url]: https://github.com/LAAS-HRI/semantic_route_description
