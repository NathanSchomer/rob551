import numpy as np

def get_map():
    map = None
    return map

def discretize_map(curr_map):
    w, h = 4, 4
    return (w, h)

def init_nav_field(field, curr_map):
    return field

def init_obs_field(field, curr_map):
    return field

def sum_fields(*fields):
    fields[0]['mag'] = np.add(*[f['mag'] for f in fields])
    fields[0]['theta'] = np.mean(*[f['theta'] for f in fields])
    return fields[0]

if __name__ == '__main__':

    # get and discretize map
    curr_map = get_map()
    map_shape = discretize_map(curr_map)

    # generate navigation and obstacle fields
    new_field = lambda : np.zeros(map_shape, dtype={'names':('theta', 'mag'),
                                  'formats':('double', 'double')})

    # init new fields
    nav_field = init_nav_field(curr_map, new_field, map_shape)
    obs_field = init_obs_field(curr_map, new_field, map_shape)

    field = sum_fields(nav_field, obs_field)

    # TODO: navigate using field