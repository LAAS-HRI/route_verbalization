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

### Expected results

**Route :** robot_infodesk -- os_exp_1 -- zizzi
```
you will find it there
OR
you see there zizzi
```

**Route :** robot_infodesk -- os_exp_1 -- gf_ww1_os1_intersection -- gf_walkway_1 -- gina
 ```
 go straight on that aisle and turn left . Finally, you will find it on the left
 OR
 just go down that aisle and turn left then you will find gina on the left when you walk
 OR
 go straight down that aisle and turn left and you will find gina on your left when you walk
 ```

**Route :** robot_infodesk -- os_exp_1 -- door_h20 -- os_hall -- door_A
 ```
go through the door . Finally, you will find door a on the right when you walk
OR
go through the door . you will find it on the right when you walk
OR
go through the door . Finally, it will be on your right when you walk
```

**Route :** robot_infodesk -- os_exp_1 -- door_h20 -- os_hall -- stairs_1 -- ff_corridor_3 -- ff_c34_intersection -- ff_corridor_4 -- burger_king
```
go through the door , and then, take the stair at your left and turn left , then go almost at the very end of the corridor and turn left . Finally, you will see it on the right side
OR
go through the door . take the stair at the left and turn left . go almost at the very end of the aisle and turn left . you will find it on the right side
```

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

[Ontologenius-Dependency-Image]: https://img.shields.io/badge/dependencies-ontologenius-1eb0fc.svg
[Ontologenius-Dependency-Url]: https://github.com/sarthou/ontologenius
[Description-Dependency-Image]: https://img.shields.io/badge/dependencies-semantic_route_description-1eb0fc.svg
[Description-Dependency-Url]: https://github.com/LAAS-HRI/semantic_route_description
