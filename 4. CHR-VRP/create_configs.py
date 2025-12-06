import json
from itertools import product
from copy import copy
import os
import sys

with open('instances.txt') as f:
    instances = f.readline().strip().split(',')

instances = [int(x) for x in instances]

config_template = sys.argv[1]
d = json.loads(open(config_template).read())

for instance in instances:
    if not os.path.exists(f'instance_{instance}'):
        os.mkdir(f'instance_{instance}')
    template = {'StartingPoint': instance,
                'EndingPoint': instance + 1}
    list_vals = []

    for key, val in d.items():
        if type(val) is not list:
            template[key] = val
        else:
            list_vals.append(key)
            template[key] = -1

    tuples = list(product(*(d[key] for key in list_vals)))
    configs = []

    for tup in tuples:
        configs.append(copy(template))
        for i, key in enumerate(list_vals):
            configs[-1][key] = tup[i]

    for i, config in enumerate(configs):
        with open(f'instance_{instance}/config_{instance}_{"_".join(map(str, tuples[i]))}.json', 'w') as f:
            f.write(json.dumps(config, indent=4))
