# add laneless nodes
import yaml
from collections import OrderedDict
import os

# Custom loader/dumper to preserve order
def ordered_load(stream, Loader=yaml.Loader, object_pairs_hook=OrderedDict):
    class OrderedLoader(Loader):
        pass
    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)
    return yaml.load(stream, OrderedLoader)

def ordered_dump(data, stream=None, Dumper=yaml.Dumper, **kwds):
    class OrderedDumper(Dumper):
        pass
    def _dict_representer(dumper, data):
        return dumper.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
            data.items())
    OrderedDumper.add_representer(OrderedDict, _dict_representer)
    return yaml.dump(data, stream, OrderedDumper, **kwds)

# Load original YAML
current_dir = os.path.dirname(os.path.realpath(__file__))
with open(os.path.join(current_dir, 'config/runs.yaml'), 'r') as f:
    data = ordered_load(f)

# Define replacement rules
replacements = {
    520: 372,
    314: 350,
    328: 259
}

# Process each run
for run_name in data:
    modified_nodes = []
    for node in data[run_name]:
        modified_nodes.append(node)
        if node in replacements:
            new_node = replacements[node]
            if new_node is not None:
                modified_nodes.append(new_node)
                print(f"added {new_node} after {node}")
    data[run_name] = modified_nodes

# Save modified YAML
with open(os.path.join(current_dir, 'config/runs_mod2.yaml'), 'w') as f:
    ordered_dump(data, f, default_flow_style=False)

print("Modified YAML saved to config/runs_mod.yaml")

# replace
# 399 -> 484
# 403 -> 520
# 343 -> 350
# 368 -> 372
# 316 -> 314
# 319 -> 321
# 330 -> 328
# 257 -> 255
# 261 -> 259
# 225, 228, 240 -> 240
# 288 -> 280
# 198 -> 196
# 42 -> 201
# 301 -> 297
# 177 -> 173
# 82 -> 158
# 80 -> 179
# 93 -> 182
# 25 -> 133
# 135 -> 136
# 29 -> 132
# 110 -> 111
# 121 -> none
# 130 -> none
# 117 -> 118
# 127 -> 125
# 71 -> 209
# 185 -> 184