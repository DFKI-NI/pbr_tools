# Convention used to store and name new maps

<world>/<world_config>/<world_config>.yaml
<world>/<world_config>/<world_config>.[png|pgm]
<world>/<world_config>/<world_config>_virtual_walls.yaml (optional)
<world>/<world_config>/<world_config>_virtual_walls.[png|pgm] (optional)

world: a real or simulated environment, e.g. cic, moelk
world_config: a particular object arrangement within the previous world, e.g.
              table_1 located at ..., table_2 located at ... (only objs which have an impact on the map)
