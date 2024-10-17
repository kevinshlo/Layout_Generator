import os
import argparse
import pickle
import subprocess
import shutil
import glob
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def visual(filepath):
    """
    visualize a raw case (modified from visual.py)
    """
    file_name = filepath[:-4]
    with open(filepath, "r") as f:
        # read width
        line = f.readline().split()
        width = int(line[2])
        # read height
        line = f.readline().split()
        height = int(line[2])
        # read total WL
        line = f.readline().split()
        wl = int(line[1])
        _ = f.readline()  # total_via _
        # read layer
        line = f.readline().split()
        layer = int(line[1])
        for _ in range(layer):  # Track_ _ _ _
            f.readline()
        # read obstacles
        line = f.readline().split()
        obs_num = int(line[1])
        h_obs_list = []
        v_obs_list = []
        for i in range(obs_num):
            line = f.readline().split()
            assert int(line[2]) == int(line[5])
            if int(line[2]) % 2 == 0:  # support layer > 2
                h_obs_list.append(
                    [int(line[0]), int(line[1]), int(line[3]), int(line[4])]
                )
            else:
                v_obs_list.append(
                    [int(line[0]), int(line[1]), int(line[3]), int(line[4])]
                )

        pin_list = []
        pin_id_list = []
        via_list = []
        h_wire_list = []
        v_wire_list = []
        # read nets
        line = f.readline().split()
        net_num = int(line[1])
        for i in range(net_num):
            # read net id
            line = f.readline().split()
            net_id = int(line[1])
            # read pins
            line = f.readline().split()
            pin_num = int(line[1])
            for _ in range(pin_num):
                _ = f.readline()  # pin_id _
                _ = f.readline()  # ap_num 1
                line = f.readline().split()
                # if net_id == 166:
                pin_list.append([int(line[0]), int(line[1]), int(line[2])])
                pin_id_list.append(net_id)
            # read vias
            line = f.readline().split()
            via_num = int(line[1])
            for _ in range(via_num):
                line = f.readline().split()
                # if net_id == 166:
                via_list.append([int(line[0]), int(line[1])])
            # read horizontal wires
            line = f.readline().split()
            h_seg_num = int(line[1])
            for _ in range(h_seg_num):
                line = f.readline().split()
                # if net_id == 166:
                h_wire_list.append(
                    [int(line[0]), int(line[1]), int(line[3]), int(line[4])]
                )
            # read vertical wires
            line = f.readline().split()
            v_seg_num = int(line[1])
            for _ in range(v_seg_num):
                line = f.readline().split()
                # if net_id == 166:
                v_wire_list.append(
                    [int(line[0]), int(line[1]), int(line[3]), int(line[4])]
                )

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_aspect("equal", adjustable="box")
        for obs in h_obs_list:
            rect = Rectangle(
                (obs[0], obs[1]),
                obs[2] - obs[0],
                obs[3] - obs[1],
                color="darkgreen",
                fill=False,
                hatch="xx",
                alpha=1,
            )
            ax.add_patch(rect)
        for obs in v_obs_list:
            rect = Rectangle(
                (obs[0], obs[1]),
                obs[2] - obs[0],
                obs[3] - obs[1],
                color="navy",
                fill=False,
                hatch="xx",
                alpha=1,
            )
            ax.add_patch(rect)
        for wire in h_wire_list:
            rect = Rectangle(
                (0.25 + wire[0], 0.25 + wire[1]),
                wire[2] - wire[0] - 0.5,
                0.5,
                color="green",
                alpha=0.3,
            )
            ax.add_patch(rect)
        for wire in v_wire_list:
            rect = Rectangle(
                (0.25 + wire[0], 0.25 + wire[1]),
                0.5,
                wire[3] - wire[1] - 0.5,
                color="blue",
                alpha=0.5,
            )
            ax.add_patch(rect)
        for pin, id in zip(pin_list, pin_id_list):
            rect = Rectangle((pin[0], pin[1]), 1, 1, color="orange", alpha=0.5)
            ax.add_patch(rect)
            ax.annotate(
                id,
                (pin[0] + 0.5, pin[1] + 0.5),
                color="black",
                fontsize=3,
                ha="center",
                va="center",
            )
        for via in via_list:
            rect = Rectangle(
                (0.25 + via[0], 0.25 + via[1]),
                0.5,
                0.5,
                color="red",
                fill=False,
                alpha=1,
            )
            ax.add_patch(rect)
        plt.xlim([0, width])
        plt.ylim([0, height])
        plt.title(f"{file_name} WL = {wl}")
        plt.savefig(f"{file_name}.png", dpi=600)
        plt.close(fig)


def gen_cases(dir, width, height, layer, level_num, indices, types, trainings, numbers):
    """
    copy from GNN_based_router_new/case_gen.py
    """
    config_dict = locals()
    # write config file
    with open(os.path.join(dir, "config.txt"), "w") as f:
        f.write(f"level_num {level_num}\n")
        for i in range(level_num):
            f.write(f"level_{i} {indices[i]} {types[i]} {trainings[i]} {numbers[i]}\n")
    with open(os.path.join(dir, "config.pickle"), "wb") as f:
        pickle.dump(config_dict, f)


def format(fin_name, fout_name):
    """
    read files from layout generate than format for RL router
    """
    fin = open(fin_name, "r")
    fout = open(fout_name, "w")
    fout.write(fin.readline())  # Width _
    fout.write(fin.readline())  # Height _
    fout.write(fin.readline())  # total_WL _
    fout.write(fin.readline())  # total_via _
    l = fin.readline()  # Layer _
    Layer = int(l.split()[1])
    fout.write(l)
    for _ in range(Layer):  # Track_ _ _ _
        fout.write(fin.readline())
    # Obstacle_num _
    l = fin.readline()
    Obstacle_num = int(l.split()[1])
    fout.write(l)
    for _ in range(Obstacle_num):
        fout.write(fin.readline())
    # Net_num _
    l = fin.readline()
    Net_num = int(l.split()[1])
    fout.write(l)
    for _ in range(Net_num):
        fout.write(fin.readline())  # Net_id _
        # pin_num _
        l = fin.readline()
        pin_num = int(l.split()[1])
        fout.write(l)
        for _ in range(pin_num):
            fout.write(fin.readline())  # pin_id _
            # ap_num _
            l = fin.readline()
            ap_num = int(l.split()[1])
            fout.write(l)
            for _ in range(ap_num):
                fout.write(fin.readline())
        # Via_num _
        l = fin.readline()
        Via_num = int(l.split()[1])
        for _ in range(Via_num):
            fin.readline()
        # H_segment_num _
        l = fin.readline()
        H_segment_num = int(l.split()[1])
        for _ in range(H_segment_num):
            fin.readline()
        # V_segment_num _
        l = fin.readline()
        V_segment_num = int(l.split()[1])
        for _ in range(V_segment_num):
            fin.readline()


MAIN = "main"
SUFFIX = ""


def gen_train(size, layer, argvs):
    """
    generate cases for training set
    """
    index = 10000
    dir = f"train_{size}x{size}x{layer}{SUFFIX}"
    os.mkdir(dir)
    raw_dir = f"{dir}/raw"
    os.mkdir(raw_dir)
    for lv, argv in enumerate(argvs):
        lv_raw_dir = f"{raw_dir}/level_{lv}"
        os.mkdir(lv_raw_dir)
        assert 0 == subprocess.call(
            f"./{MAIN} {lv_raw_dir} {' '.join([str(arg) for arg in argv])}",
            shell=True,
        )
        lv_dir = f"{dir}/level_{lv}"
        os.mkdir(lv_dir)
        for fin in os.listdir(lv_raw_dir):
            if 0 == int(fin[:-4]):
                visual(f"{lv_raw_dir}/{fin}")
            fout = f"id_{(lv + 1) * index + int(fin[:-4])}.txt"
            format(f"{lv_raw_dir}/{fin}", f"{lv_dir}/{fout}")
    gen_cases(
        dir=dir,
        width=size,
        height=size,
        layer=layer,
        level_num=len(argvs),
        indices=[(i + 1) * index for i in range(len(argvs))],
        types=[0 for _ in range(len(argvs))],
        trainings=[argvs[0][-2] == 1] + [False] * (len(argvs) - 1),  # net_num == 1
        numbers=[i[0] for i in argvs],  # test_num
    )


def gen_eval(size, layer, argv):
    """
    generate cases for evaluating set
    """
    dir = f"eval_{size}x{size}x{layer}{SUFFIX}"
    os.mkdir(dir)
    raw_dir = f"{dir}/raw"
    os.mkdir(raw_dir)
    lv_raw_dir = f"{raw_dir}/level_0"
    os.mkdir(lv_raw_dir)
    assert 0 == subprocess.call(
        f"./{MAIN} {lv_raw_dir} {' '.join([str(arg) for arg in argv])}",
        shell=True,
    )
    lv_dir = f"{dir}/level_0"
    os.mkdir(lv_dir)
    for fin in os.listdir(lv_raw_dir):
        if 0 == int(fin[:-4]):
            visual(f"{lv_raw_dir}/{fin}")
        fout = f"id_{int(fin[:-4])}.txt"
        format(f"{lv_raw_dir}/{fin}", f"{lv_dir}/{fout}")
    gen_cases(
        dir=dir,
        width=size,
        height=size,
        layer=layer,
        level_num=1,
        indices=[0],
        types=[0],
        trainings=[False],
        numbers=[argv[0]],  # test_num
    )


def gen_test(size, layer, argv):
    """
    copy evaluating cases as testing set
    """
    dir = f"test_{size}x{size}x{layer}{SUFFIX}"
    os.mkdir(dir)
    raw_dir = f"{dir}/raw"
    os.mkdir(raw_dir)
    lv_raw_dir = f"{raw_dir}/level_0"
    os.mkdir(lv_raw_dir)
    assert 0 == subprocess.call(
        f"./{MAIN} {lv_raw_dir} {' '.join([str(arg) for arg in argv])}",
        shell=True,
    )
    lv_dir = f"{dir}/level_0"
    os.mkdir(lv_dir)
    for fin in os.listdir(lv_raw_dir):
        if 0 == int(fin[:-4]):
            visual(f"{lv_raw_dir}/{fin}")
        fout = f"id_{int(fin[:-4])}.txt"
        format(f"{lv_raw_dir}/{fin}", f"{lv_dir}/{fout}")
    gen_cases(
        dir=dir,
        width=size,
        height=size,
        layer=layer,
        level_num=1,
        indices=[0],
        types=[1],
        trainings=[False],
        numbers=[argv[0]],  # test_num
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--main", default="main", type=str)
    parser.add_argument("--suffix", default="", type=str)
    parser.add_argument("--size", default=500, type=int)
    parser.add_argument("--layer", default=3, type=int)
    args = parser.parse_args()
    MAIN = args.main
    SUFFIX = args.suffix
    s = args.size
    l = args.layer
    min_obs_size, max_obs_size = int(s * 0.05), int(s * 0.5)
    # (test_num, width, height, layers, obs_num, min_obs_size, max_obs_size, net_num, pin_num)
    levels = [
        (2000, s, s, l, int(s * 0.50), min_obs_size, max_obs_size, 1, 4),
        (800, s, s, l, int(s * 0.50), min_obs_size, max_obs_size, 15, 5),
        (160, s, s, l, int(s * 0.25), min_obs_size, max_obs_size, 75, 5),
        (80, s, s, l, int(s * 0.10), min_obs_size, max_obs_size, 150, 5),
        (40, s, s, l, int(s * 0.05), min_obs_size, max_obs_size, 300, 5),
    ]  # level_0 must be single net and other should not
    gen_train(args.size, args.layer, levels)
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
