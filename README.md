# Layout_Generator

## Usage

### Build

```shell
make -B
```

### Generate Cases

edit specs in [case_gen.py](./case_gen.py):

```python
# (test_num, width, height, layers, obs_num, min_obs_size, max_obs_size, net_num, pin_num)
levels = [
    (2000, s, s, l, int(s * 0.50), min_obs_size, max_obs_size, 1, 4),  # only level_0 1-net
    (800, s, s, l, int(s * 0.50), min_obs_size, max_obs_size, 15, 5),  # only level_0 1-net
    (160, s, s, l, int(s * 0.25), min_obs_size, max_obs_size, 75, 5),
    (80, s, s, l, int(s * 0.10), min_obs_size, max_obs_size, 150, 5),
    (40, s, s, l, int(s * 0.05), min_obs_size, max_obs_size, 300, 5),
]
```

then execute:

```shell
python case_gen.py --main MAIN --size SIZE --layer LAYER
```

default: 
- `MAIN` = main
- `SIZE` = 500
- `LAYER` = 3
