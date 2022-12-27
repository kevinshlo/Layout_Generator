import glob
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('--dir', type=str, help='directory of the test files')
args = parser.parse_args()
file_list = glob.glob(f"./{args.dir}/*.txt")

for count, file in enumerate(file_list):
    file_name = file.split('/')[-1][:-4]
    with open(file, 'r') as f:

        #read width
        line = f.readline().split()
        width = int(line[1])

        #read height
        line = f.readline().split()
        height = int(line[1])

        #read layer
        line = f.readline().split()
        layer = int(line[1])

        #read total WL
        line = f.readline().split()
        wl = int(line[1])

        #read obstacles
        line = f.readline().split()
        obs_num = int(line[1])
        h_obs_list = []
        v_obs_list = []
        for i in range(obs_num):
            line = f.readline().split()
            assert(int(line[2]) == int(line[5]))
            if int(line[2]) == 0:
                h_obs_list.append([int(line[0]), int(line[1]), int(line[3]), int(line[4])])
            else:
                v_obs_list.append([int(line[0]), int(line[1]), int(line[3]), int(line[4])])

        pin_list = []
        pin_id_list = []
        via_list = []
        h_wire_list = []
        v_wire_list = []
        #read nets
        line = f.readline().split()
        net_num = int(line[1])
        for i in range(net_num):
            #read net id
            line = f.readline().split()
            net_id = int(line[1])
            
            #read pins
            line = f.readline().split()
            pin_num = int(line[1])
            for i in range(pin_num):
                line = f.readline().split()
                pin_list.append([int(line[0]), int(line[1]), int(line[2])])
                pin_id_list.append(net_id)

            #read vias
            line = f.readline().split()
            via_num = int(line[1])
            for i in range(via_num):
                line = f.readline().split()
                via_list.append([int(line[0]), int(line[1])])

            #read horizontal wires
            line = f.readline().split()
            h_seg_num = int(line[1])
            for i in range(h_seg_num):
                line = f.readline().split()
                h_wire_list.append([int(line[0]), int(line[1]), int(line[3]), int(line[4])])

            #read vertical wires
            line = f.readline().split()
            v_seg_num = int(line[1])
            for i in range(v_seg_num):
                line = f.readline().split()
                v_wire_list.append([int(line[0]), int(line[1]), int(line[3]), int(line[4])])
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_aspect('equal',adjustable='box')
        for obs in h_obs_list:
            rect = Rectangle((obs[0], obs[1]), obs[2]-obs[0], obs[3]-obs[1], color='darkgreen',fill=False, hatch='xx', alpha = 1)
            ax.add_patch(rect)

        for obs in v_obs_list:
            rect = Rectangle((obs[0], obs[1]), obs[2]-obs[0], obs[3]-obs[1], color='navy',fill=False, hatch='xx', alpha = 1)
            ax.add_patch(rect)

        for wire in h_wire_list:
            rect = Rectangle((wire[0], wire[1]),wire[2]-wire[0], 1, color="green", alpha = 0.3)
            ax.add_patch(rect)

        for wire in v_wire_list:
            rect = Rectangle((wire[0], wire[1]),1, wire[3]-wire[1], color="blue", alpha = 0.5)
            ax.add_patch(rect)

        for pin, id in zip(pin_list, pin_id_list):
            rect = Rectangle((pin[0],pin[1]), 1, 1, color="orange", alpha = 0.7)
            ax.add_patch(rect)
            ax.annotate(id, (pin[0] + 0.5, pin[1] + 0.5), color='black', fontsize=5, ha='center', va='center')

        for via in via_list:
            rect = Rectangle((via[0], via[1]), 1, 1, color="red", fill=False,alpha = 1)
            ax.add_patch(rect)
        plt.xlim([0,width])
        plt.ylim([0,height])
        plt.title(f"{file_name} WL = {wl}")
        plt.savefig(f"./{args.dir}/{file_name}.png", dpi=300)
        plt.close(fig)