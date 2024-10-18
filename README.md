# Layout_Generator

## Usage

### Build

```shell
make -B
```

### Generate Cases

Edit training set specs in [case_gen.py](./case_gen.py):

```python
# (test_num, width, height, layers, obs_num, min_obs_size, max_obs_size, net_num, pin_num)
levels = [
    (2000, s, s, l, int(s * 0.50), min_obs_size, max_obs_size, 1, 4),
    (800, s, s, l, int(s * 0.50), min_obs_size, max_obs_size, 15, 5),
    (160, s, s, l, int(s * 0.25), min_obs_size, max_obs_size, 75, 5),
    (80, s, s, l, int(s * 0.10), min_obs_size, max_obs_size, 150, 5),
    (40, s, s, l, int(s * 0.05), min_obs_size, max_obs_size, 300, 5),
]  # level_0 must be single net and other should not```
```

where:
- `test_num`: number of cases in that level (2000 cases in level_0)
- `net_num`: number of nets in any case in that level (any case in level_0 has 1 net)
- `pin_num`: number of pins in each net in any case in that level (any net in level_0 has 4 pins)

Similar for eval & test set:

```python
gen_eval(
    args.size,
    args.layer,
    (40, s, s, l, int(s * 0.50), min_obs_size, max_obs_size, 1, 5),
)
gen_test(
    args.size,
    args.layer,
    (40, s, s, l, int(s * 0.05), min_obs_size, max_obs_size, 300, 5),
)
```

Then execute this command to generate data set:

```shell
python case_gen.py --main MAIN --size SIZE --layer LAYER
```

default: 
- `MAIN` = main
- `SIZE` = 500
- `LAYER` = 3

## Structure of Datasets

Here is an example of a training set

```
train_500x500x3
├── level_0
│   ├── id_10000.txt
│   ...
├── level_1
├── level_2
├── level_3
├── level_4
├── raw  # raw info, does not required by RL env
│   ├── level_0
│   │   ├── 0.png  # first case has visualized layout
│   │   ├── 0.txt
│   │   ├── 1.txt
│   │   ...
│   ├── level_1
│   ├── level_2
│   ├── level_3
│   └── level_4
├── config.pickle  # file required by RL env
└── config.txt  # file required by RL env
```