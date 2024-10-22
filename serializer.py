""" Data format (id_xxxxx.txt)
Width [h_start_point] [h_end_point]   # start/end of x-axis
Height [v_start_point] [v_end_point]  # start/end of y-axis
total_WL [Triton Route wl]
total_via [Triton Route via_number]
Layer [layer number]
Track[i] [start point] [spacing] [preferred direction]
...
Obstacle_num [obs_number]
[obs_x1] [obs_y1] [obs_z1] [obs_x2] [obs_y2] [obs_z2]
...
Net_num [net_number]
Net_id [net_id]
pin_num [pin_number]
pin_id [pin_id]
ap_num [access point number]
[ap_x] [ap_y] [ap_z]
...
Via_num [via number]
[via_x] [via_y] [via_z (lower layer)]
...
H_segment_num [horizontal segment numbers]
[seg_x1] [seg_y1] [seg_z1] [seg_x2] [seg_y2] [seg_z2]
...
V_segment_num [vertical segment numbers]
[seg_x1] [seg_y1] [seg_z1] [seg_x2] [seg_y2] [seg_z2]
...
"""

import argparse
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HORIZONTAL = 0


class Net:
    def __init__(self, id, pins, vias, h_segs, v_segs) -> None:
        self.id: int = id
        self.pins: list[list[tuple[int, int, int]]] = pins
        self.vias: list[tuple[int, int, int]] = vias
        self.h_segs: list[tuple[int, int, int, int, int, int]] = h_segs
        self.v_segs: list[tuple[int, int, int, int, int, int]] = v_segs

    def WL(self):
        h = sum([x2 - x1 for x1, _, _, x2, _, _ in self.h_segs])
        v = sum([y2 - y1 for _, y1, _, _, y2, _ in self.v_segs])
        return h + v


class Testcase:
    def __init__(
        self,
        h_start_point: int,
        h_end_point: int,
        v_start_point: int,
        v_end_point: int,
        Layer: int,
        tracks: list[tuple[int, int, int]],
        obstacles: list[tuple[int, int, int, int, int, int]],
        nets: list[Net],
    ) -> None:
        self.h_start_point = h_start_point
        self.h_end_point = h_end_point
        self.v_start_point = v_start_point
        self.v_end_point = v_end_point
        self.total_WL = sum([net.WL() for net in nets])
        self.total_via = sum([len(net.vias) for net in nets])
        assert len(tracks) == Layer
        self.Layer = Layer
        self.tracks = tracks
        self.Obstacle_num = len(obstacles)
        self.obstacles = obstacles
        self.Net_num = len(nets)
        self.nets = nets

    @classmethod
    def deserialize(cls, in_file: str):
        read_segments = "segment" in " ".join(open(in_file, "r").readlines())
        print(f"deserialize {in_file}, read_segments = {read_segments}")
        f = open(in_file, "r")
        # Width [h_start_point] [h_end_point]
        _, h_start_point, h_end_point = f.readline().split()
        h_start_point, h_end_point = int(h_start_point), int(h_end_point)
        # Height [v_start_point] [v_end_point]
        _, v_start_point, v_end_point = f.readline().split()
        v_start_point, v_end_point = int(v_start_point), int(v_end_point)
        # total_WL [Triton Route wl]
        _, total_WL = f.readline().split()
        total_WL = int(total_WL)
        # total_via [Triton Route via_number]
        _, total_via = f.readline().split()
        total_via = int(total_via)
        # Layer [layer number]
        _, Layer = f.readline().split()
        Layer = int(Layer)
        tracks = []
        for _ in range(Layer):
            # track[i] [start point] [spacing] [preferred direction]
            _, start, spacing, direction = f.readline().split()
            tracks.append((int(start), int(spacing), int(direction)))
        # Obstacle_num [obs_number]
        _, Obstacle_num = f.readline().split()
        obstacles = []
        for _ in range(int(Obstacle_num)):
            # [obs_x1] [obs_y1] [obs_z1] [obs_x2] [obs_y2] [obs_z2]
            obs = [int(i) for i in f.readline().split()]
            assert len(obs) == 6
            obstacles.append(obs)
        # Net_num [net_number]
        _, Net_num = f.readline().split()
        nets: list[Net] = []
        for _ in range(int(Net_num)):
            # Net_id [net_id]
            _, net_id = f.readline().split()
            net_id = int(net_id)
            # pin_num [pin_number]
            _, pin_num = f.readline().split()
            pin_num = int(pin_num)
            pins = []
            for _ in range(pin_num):
                # pin_id [pin_id]
                _, pin_id = f.readline().split()
                pin_id = int(pin_id)
                # ap_num [access point number]
                _, ap_num = f.readline().split()
                ap_num = int(ap_num)
                pin = []
                for _ in range(ap_num):
                    # [ap_x] [ap_y] [ap_z]
                    ap = [int(i) for i in f.readline().split()]
                    assert len(ap) == 3
                    pin.append(tuple(ap))
                pins.append(pin)
            # read segments (routing results)
            vias, h_segs, v_segs = [], [], []
            if read_segments:
                # Via_num [via number]
                _, Via_num = f.readline().split()
                Via_num = int(Via_num)
                for _ in range(Via_num):
                    # [via_x] [via_y] [via_z (lower layer)]
                    via = [int(i) for i in f.readline().split()]
                    assert len(via) == 3
                    vias.append(tuple(via))
                # H_segment_num [horizontal segment numbers]
                _, H_segment_num = f.readline().split()
                H_segment_num = int(H_segment_num)
                for _ in range(H_segment_num):
                    # [seg_x1] [seg_y1] [seg_z1] [seg_x2] [seg_y2] [seg_z2]
                    h_seg = [int(i) for i in f.readline().split()]
                    assert len(h_seg) == 6
                    h_segs.append(tuple(h_seg))
                # V_segment_num [vertical segment numbers]
                _, V_segment_num = f.readline().split()
                V_segment_num = int(V_segment_num)
                for _ in range(V_segment_num):
                    # [seg_x1] [seg_y1] [seg_z1] [seg_x2] [seg_y2] [seg_z2]
                    v_seg = [int(i) for i in f.readline().split()]
                    assert len(v_seg) == 6
                    v_segs.append(tuple(v_seg))
            nets.append(
                Net(id=net_id, pins=pins, vias=vias, h_segs=h_segs, v_segs=v_segs)
            )
        f.close()
        ret = cls(
            h_start_point=h_start_point,
            h_end_point=h_end_point,
            v_start_point=v_start_point,
            v_end_point=v_end_point,
            Layer=Layer,
            tracks=tracks,
            obstacles=obstacles,
            nets=nets,
        )
        if ret.total_WL != total_WL:
            print(f"[Warning] overwrite total_WL: {ret.total_WL} to {total_WL}")
        ret.total_WL = total_WL
        if ret.total_via != total_via:
            print(f"[Warning] overwrite total_via: {ret.total_via} to {total_via}")
        ret.total_via = total_via
        return ret

    def serialize(self, out_file: str, write_segments: bool):
        print(f"serialize {out_file}, write_segments = {write_segments}")
        f = open(out_file, "w")
        f.write(f"Width {self.h_start_point} {self.h_end_point}\n")
        f.write(f"Height {self.v_start_point} {self.v_end_point}\n")
        f.write(f"total_WL {self.total_WL}\n")
        f.write(f"total_via {self.total_via}\n")
        f.write(f"Layer {self.Layer}\n")
        assert self.Layer == len(self.tracks)
        for i, track in enumerate(self.tracks):
            start, spacing, direction = track
            f.write(f"Track{i} {start} {spacing} {direction}\n")
        f.write(f"Obstacle_num {self.Obstacle_num}\n")
        assert self.Obstacle_num == len(self.obstacles)
        for obs in self.obstacles:
            f.write(" ".join([str(o) for o in obs]) + "\n")
        f.write(f"Net_num {self.Net_num}\n")
        assert self.Net_num == len(self.nets)
        for i, net in enumerate(self.nets):
            f.write(f"Net_id {i}\n")
            assert i == net.id
            f.write(f"pin_num {len(net.pins)}\n")
            for j, pin in enumerate(net.pins):
                f.write(f"pin_id {j}\n")
                f.write(f"ap_num {len(pin)}\n")
                for ap in pin:
                    f.write(" ".join([str(a) for a in ap]) + "\n")
            # write segments
            if write_segments:
                f.write(f"Via_num {len(net.vias)}\n")
                for via in net.vias:
                    f.write(" ".join([str(v) for v in via]) + "\n")
                f.write(f"H_segment_num {len(net.h_segs)}\n")
                for h_seg in net.h_segs:
                    f.write(" ".join([str(h) for h in h_seg]) + "\n")
                f.write(f"V_segment_num {len(net.v_segs)}\n")
                for v_seg in net.v_segs:
                    f.write(" ".join([str(v) for v in v_seg]) + "\n")

    def visualize(self, png_name: str) -> None:
        if png_name[-4:] != ".png":
            png_name += ".png"
        print(f"visualize {png_name}")
        width = self.h_end_point
        height = self.v_end_point
        wl = self.total_WL
        h_obs_list = []
        v_obs_list = []
        for x1, y1, z1, x2, y2, z2 in self.obstacles:
            assert z1 == z2
            _, _, direction = self.tracks[z1]
            if direction == HORIZONTAL:
                h_obs_list.append([x1, y1, x2, y2, x2 - x1 != width, z1])
            else:
                v_obs_list.append([x1, y1, x2, y2, y2 - y1 != height, z1])

        pin_list = []
        pin_id_list = []
        via_list = []
        h_wire_list = []
        v_wire_list = []
        for net in self.nets:
            net_id = net.id
            for pin in net.pins:
                for x, y, z in pin:  # ap
                    pin_list.append([x, y, z])
                    pin_id_list.append(net_id)
            for x, y, z in net.vias:
                via_list.append([x, y])
            for x1, y1, z1, x2, y2, z2 in net.h_segs:
                h_wire_list.append([x1, y1, x2, y2])
            for x1, y1, z1, x2, y2, z2 in net.v_segs:
                v_wire_list.append([x1, y1, x2, y2])

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_aspect("equal", adjustable="box")
        h_track_obs: dict[tuple[int, int], list[int]] = {}
        for x1, y1, x2, y2, fill, z in h_obs_list:
            rect = Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                color="darkgreen",
                lw=0.1,
                fill=fill,
                hatch="xx",
                alpha=1,
            )
            ax.add_patch(rect)
            if not fill:
                if (x1, y1) not in h_track_obs:
                    h_track_obs[(x1, y1)] = []
                h_track_obs[(x1, y1)].append(z)
        for k, v in h_track_obs.items():
            ax.annotate(
                ",".join([str(i) for i in v]),
                (k[0], k[1] + 0.5),
                color="red",
                size=3,
                ha="center",
                va="center",
            )
        v_track_obs: dict[tuple[int, int], list[int]] = {}
        for x1, y1, x2, y2, fill, z in v_obs_list:
            rect = Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                color="navy",
                lw=0.1,
                fill=fill,
                hatch="xx",
                alpha=1,
            )
            ax.add_patch(rect)
            if not fill:
                if (x1, y1) not in v_track_obs:
                    v_track_obs[(x1, y1)] = []
                v_track_obs[(x1, y1)].append(z)
        for k, v in v_track_obs.items():
            ax.annotate(
                ",".join([str(i) for i in v]),
                (k[0] + 0.5, k[1]),
                color="red",
                size=3,
                ha="center",
                va="center",
                rotation="vertical",
            )
        for x1, y1, x2, y2 in h_wire_list:
            rect = Rectangle(
                (x1, y1), x2 - x1, y2 - y1, color="green", lw=0.1, fill=True, alpha=0.3
            )
            ax.add_patch(rect)
        for x1, y1, x2, y2 in v_wire_list:
            rect = Rectangle(
                (x1, y1), x2 - x1, y2 - y1, color="blue", lw=0.1, fill=True, alpha=0.5
            )
            ax.add_patch(rect)
        for pin, id in zip(pin_list, pin_id_list):
            x, y, z = pin
            rect = Rectangle((x, y), 1, 1, color="orange", alpha=0.5)
            ax.add_patch(rect)
            ax.annotate(
                id, (x + 0.5, y + 0.5), color="black", size=3, ha="center", va="center"
            )
        for x, y in via_list:
            rect = Rectangle(
                (x + 0.25, y + 0.25), 0.5, 0.5, color="red", lw=0, fill=True, alpha=0.5
            )
            ax.add_patch(rect)
        plt.xlim([0, width])
        plt.ylim([0, height])
        plt.title(f"{png_name} WL = {wl}")
        plt.savefig(f"{png_name}", dpi=600)
        plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", type=str)
    parser.add_argument("-o", type=str)
    parser.add_argument("--write_segments", type=bool, default=True)
    parser.add_argument("--image", type=str)
    args = parser.parse_args()
    testcase = Testcase.deserialize(args.i)
    if args.o is not None:
        testcase.serialize(args.o, args.write_segments)
    if args.image is not None:
        testcase.visualize(args.image)
