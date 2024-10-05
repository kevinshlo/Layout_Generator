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
    (5000, s, s, l, s, int(s * 0.05), int(s * 0.5), 1, 2),
    (2500, s, s, l, s, int(s * 0.05), int(s * 0.5), 1, 5),
    (2500, s, s, l, s, int(s * 0.05), int(s * 0.5), 5, 5),  # eval
    (250, s, s, l, int(s * 0.5), int(s * 0.1), int(s * 0.5), 20, 5),
    (100, s, s, l, int(s * 0.25), int(s * 0.1), int(s * 0.5), 100, 5),
    (50, s, s, l, int(s * 0.25), int(s * 0.1), int(s * 0.5), 300, 5),
    (50, s, s, l, int(s * 0.10), int(s * 0.1), int(s * 0.5), 700, 5),
]  # max 670 nets
```

then execute:

```shell
python case_gen.py --main MAIN --size SIZE --layer LAYER
```

default: 
- `MAIN` = main
- `SIZE` = 500
- `LAYER` = 3
