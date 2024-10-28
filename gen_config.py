import argparse
import os
import pickle
import re


def gen_cases(dir, level_num, indices, types, trainings, numbers):
    """
    modified from GNN_based_router_new/case_gen.py
    """
    config_dict = locals()
    # write config file
    with open(os.path.join(dir, "config.txt"), "w") as f:
        f.write(f"level_num {level_num}\n")
        for i in range(level_num):
            f.write(f"level_{i} {indices[i]} {types[i]} {trainings[i]} {numbers[i]}\n")
    with open(os.path.join(dir, "config.pickle"), "wb") as f:
        pickle.dump(config_dict, f)


def count(dir: str) -> tuple[int, list[int], list[int]]:
    """
    count indices and number of cases for each level
    return: (level_num, [indices,], [case_num,])
    """
    d: dict[int, tuple[int, int]] = {}
    assert os.path.exists(dir)
    for level in os.scandir(dir):
        if level.is_dir() and re.match(r"^level_[0-9]+$", level.name):
            level_i = int(level.name[len("level_") :])
            cnt = 0
            indices = None
            for id in os.scandir(level):
                if id.is_file() and re.match(r"^id_[0-9]+[.]txt$", id.name):
                    cnt += 1
                    id_i = int(id.name[len("id_") : -len(".txt")])
                    indices = id_i if indices is None else min(indices, id_i)
            d[level_i] = (indices, cnt)
    level_num = len(d)
    indices = [d[k][0] for k in sorted(d.keys())]
    numbers = [d[k][1] for k in sorted(d.keys())]
    return level_num, indices, numbers


def gen_train(dir: str):
    level_num, indices, numbers = count(dir)
    gen_cases(
        dir=dir,
        level_num=level_num,
        indices=indices,
        types=[0] * level_num,
        trainings=[True] + [False] * (level_num - 1),
        numbers=numbers,
    )


def gen_evel(dir: str):
    level_num, indices, numbers = count(dir)
    gen_cases(
        dir=dir,
        level_num=level_num,
        indices=indices,
        types=[0] * level_num,
        trainings=[False] * level_num,
        numbers=numbers,
    )


def gen_test(dir: str):
    level_num, indices, numbers = count(dir)
    gen_cases(
        dir=dir,
        level_num=level_num,
        indices=indices,
        types=[1] * level_num,
        trainings=[False] * level_num,
        numbers=numbers,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", type=str)
    parser.add_argument("-t", type=str)  # train / eval / test
    args = parser.parse_args()
    dir = args.i
    if args.t == "train":
        gen_train(dir)
    elif args.t == "eval":
        gen_evel(dir)
    elif args.t == "test":
        gen_test(dir)
    else:
        raise ValueError("invalid '-t' option ('train', 'eval', 'test')")
