from serializer import Testcase, Net, HORIZONTAL
import argparse
import numpy as np
import os
import shutil
import re


SPACE = -1
OBSTACLE = -2


class Converter:
    """
    Convert the testcase from original coord to spacing-uniformed coord
    - layer 0 should horizontal (rotate not supported yet)
    - array coord is ordered as (z, x, y) aka (layer, width, height),
      so does size.
    - width & height convert from original coordinates
      to coordinates based on spacing of layer 0 & 1.
    - try to align each track in a layer to spacing-uniformed coord,
      which might insert obstacles in converted layout
    """

    def __init__(
        self, in_file: str, remove_empty_net: bool, remove_non_bottom_pins: bool
    ) -> None:
        self.testcase = Testcase.deserialize(in_file)
        self.remove_empty_net = remove_empty_net
        self.remove_non_bottom_pins = remove_non_bottom_pins
        self.convert_size_and_tracks()
        # size: (layer, width, height)
        # tracks: [(start, spacing, direction, track_num),]
        self.convert_start_and_spacing()
        # x_start, x_spacing, y_start, y_start
        self.init_track_arr()
        # track_arr: array marks available tracks
        # track_coords: each track: [z][x_or_y(spacing-uniformed)] -> x_or_y(original)
        self.convert_obs()
        # obstacles: converted obstacles (including track obstacles)
        self.convert_net()
        # nets: converted nets (including vias & segments)
        layer, width, height = self.size
        self.converted_testcase = Testcase(
            h_start_point=0,
            h_end_point=height,
            v_start_point=0,
            v_end_point=width,
            Layer=layer,
            tracks=[(0, 1, direction) for _, _, direction, _ in self.tracks],
            obstacles=self.obstacles,
            nets=self.nets,
        )  # converted cases

    def serialize(self, out_file: str, write_segments: bool):
        self.converted_testcase.serialize(out_file, write_segments)

    def visualize(self, png_name: str):
        self.converted_testcase.visualize(png_name)

    def convert_size_and_tracks(self):
        """init spacing-uniformed size of plane and number of tracks of each layer"""
        x_min, x_max = self.testcase.h_start_point, self.testcase.h_end_point
        y_min, y_max = self.testcase.v_start_point, self.testcase.v_end_point
        width, height = 0, 0
        tracks = []  # append number of tracks for each layer
        for start, spacing, direction in self.testcase.tracks:
            if direction == HORIZONTAL:
                w = (y_max - start) // spacing
                if start + spacing * w + 1 < y_max:
                    w += 1
                width = max(width, w)
                tracks.append((start, spacing, direction, w))
            else:
                h = (x_max - start) // spacing
                if start + spacing * h + 1 < x_max:
                    h += 1
                height = max(height, h)
                tracks.append((start, spacing, direction, h))
        self.size = (self.testcase.Layer, width, height)
        self.tracks: list[tuple[int, int, int, int]] = tracks

    def convert_start_and_spacing(self):
        """
        init start point & spacing of the plane
        determined by the lowest H and V layer with largest number of tracks
        """
        _, width, height = self.size
        self.x_start, self.x_spacing = None, None
        self.y_start, self.y_spacing = None, None
        for start, spacing, direction, track_num in self.tracks:
            if direction == HORIZONTAL and track_num == height:
                self.y_start, self.y_spacing = start, spacing
                break  # H-layer with maximum tracks
        assert self.y_start is not None and self.y_spacing is not None
        for start, spacing, direction, track_num in self.tracks:
            if direction != HORIZONTAL and track_num == width:
                self.x_start, self.x_spacing = start, spacing
                break  # V-layer with maximum tracks
        assert self.x_start is not None and self.x_spacing is not None

    def init_track_arr(self):
        """init `track_arr` which mark routable tracks as `SPACE` other as `OBSTACLE`"""
        _, width, height = self.size
        self.track_arr = np.full(self.size, OBSTACLE, dtype=int)
        # store coord of each track: [z][x_or_y(spacing-uniformed)] -> x_or_y(original)
        self.track_coords: list[list[int | None]] = []
        for z, track in enumerate(self.tracks):
            start, spacing, direction, track_num = track
            tracks = [None] * (height if direction == HORIZONTAL else width)
            for p in range(start, start + spacing * track_num, spacing):
                if direction == HORIZONTAL:
                    y = self.map_y(p)
                    self.track_arr[z, :, y] = SPACE
                    tracks[y] = p
                else:
                    x = self.map_x(p)
                    self.track_arr[z, x, :] = SPACE
                    tracks[x] = p
            self.track_coords.append(tracks)

    def convert_obs(self):
        """
        map all obstacles from original to spacing-uniformed coord,
        and add new obstacles from `self.track_arr`
        """
        _, width, height = self.size
        self.obstacles = []
        for obs in self.testcase.obstacles:
            mapped_obs = self.map_obs(obs)
            if mapped_obs is not None:
                self.obstacles.append(mapped_obs)
                x1, y1, z1, x2, y2, z2 = mapped_obs
                if x1 < 0 or x1 >= width or x2 <= 0 or x2 > width:
                    raise ValueError(f"found illegal mapped_obs {mapped_obs}")
                if y1 < 0 or y1 >= height or y2 <= 0 or y2 > height:
                    raise ValueError(f"found illegal mapped_obs {mapped_obs}")
        redundant_obs_num = len(self.testcase.obstacles) - len(self.obstacles)
        print(
            f"Remove {redundant_obs_num} / {len(self.testcase.obstacles)} redundant obstacles."
        )
        track_obs_num = 0
        layers, width, height = self.size
        for z in range(layers):
            _, _, direction, _ = self.tracks[z]
            if direction == HORIZONTAL:
                for y in range(height):
                    if OBSTACLE in self.track_arr[z, :, y]:
                        self.obstacles.append((0, y, z, width, y + 1, z))
                        track_obs_num += 1
                        x1, y1, z1, x2, y2, z2 = self.obstacles[-1]
                        if x1 < 0 or x1 >= width or x2 <= 0 or x2 > width:
                            raise ValueError(
                                f"found illegal h_track_obs {self.obstacles[-1]}"
                            )
                        if y1 < 0 or y1 >= height or y2 <= 0 or y2 > height:
                            raise ValueError(
                                f"found illegal h_track_obs {self.obstacles[-1]}"
                            )
            else:
                for x in range(width):
                    if OBSTACLE in self.track_arr[z, x, :]:
                        self.obstacles.append((x, 0, z, x + 1, height, z))
                        track_obs_num += 1
                        x1, y1, z1, x2, y2, z2 = self.obstacles[-1]
                        if x1 < 0 or x1 >= width or x2 <= 0 or x2 > width:
                            raise ValueError(
                                f"found illegal v_track_obs {self.obstacles[-1]}"
                            )
                        if y1 < 0 or y1 >= height or y2 <= 0 or y2 > height:
                            raise ValueError(
                                f"found illegal v_track_obs {self.obstacles[-1]}"
                            )
        print(f"Add {track_obs_num} track obstacles.")

    def convert_net(self):
        _, width, height = self.size
        self.nets: list[Net] = []
        id = 0
        for net in self.testcase.nets:
            if self.remove_empty_net and len(net.pins) == 0:
                continue
            flag = False
            pins = []
            for pin in net.pins:
                aps = []
                for x, y, z in pin:
                    if z == 0:
                        x, y, z = *self.map_coord((x, y)), z
                        if 0 <= x and x < width and 0 <= y and y < height:
                            aps.append((x, y, z))
                if self.remove_non_bottom_pins and len(aps) == 0:
                    flag = True
                    break
                pins.append(list(set(aps)))  # remove redundant aps
            if flag:
                continue
            self.nets.append(
                Net(
                    id=id,
                    pins=pins,
                    vias=[(*self.map_coord((x, y)), z) for x, y, z, in net.vias],
                    h_segs=[
                        (
                            self.map_x(x1),
                            self.map_y(y1),
                            z1,
                            min(self.map_x(x2) + 1, width),  # left open
                            self.map_y(y1) + 1,  # wire size is 1
                            z2,
                        )
                        for x1, y1, z1, x2, _, z2 in net.h_segs
                    ],
                    v_segs=[
                        (
                            self.map_x(x1),
                            self.map_y(y1),
                            z1,
                            self.map_x(x1) + 1,  # wire size is 1
                            min(self.map_y(y2) + 1, height),  # left open
                            z2,
                        )
                        for x1, y1, z1, _, y2, z2 in net.v_segs
                    ],
                )
            )
            id += 1

    def map_coord(self, coord: tuple[int, int]) -> tuple[int, int]:
        """map original coord to spacing-uniformed coord"""
        x, y = coord
        return self.map_x(x), self.map_y(y)

    def map_x(self, x: int) -> int:
        _, width, _ = self.size  # round 0.5 up
        return min(max(round((x - self.x_start) / self.x_spacing + 1e-9), 0), width)

    def map_y(self, y: int) -> int:
        _, _, height = self.size
        return min(max(round((y - self.y_start) / self.y_spacing + 1e-9), 0), height)

    def map_obs(
        self, obs: tuple[int, int, int, int, int, int]
    ) -> tuple[int, int, int, int, int, int]:
        """
        map obstacle from original coord to spacing-uniformed coord\\
        return `None` if that obstacle do not block any track
        """

        def fix_range(r1, r2, z) -> tuple[int, int]:
            if z < 0 or z >= len(self.tracks):
                return None
            _, width, height = self.size
            _, _, direction, _ = self.tracks[z]
            if direction == HORIZONTAL:
                r1m = max(self.map_y(r1) - 1, 0)
                r2m = min(self.map_y(r2) + 1, height)
            else:
                r1m = max(self.map_x(r1) - 1, 0)
                r2m = min(self.map_x(r2) + 1, width)
            r_min, r_max = None, None
            for r in range(r1m, r2m):
                track = self.track_coords[z][r]
                if r1 <= track and track < r2:
                    r_min = r if r_min is None else min(r, r_min)
                    r_max = r if r_max is None else max(r, r_max)
            if r_min is None or r_max is None:
                return None
            else:
                return r_min, r_max + 1

        x1, y1, z1, x2, y2, z2 = obs
        assert z1 == z2, "only supports obstacle within a layer"
        _, _, direction, _ = self.tracks[z1]
        # print(x1m, y1m, z1, x2m, y2m, z2)
        if direction == HORIZONTAL:
            y_range = fix_range(y1, y2, z1)
            if y_range is None:
                return None
            y_min, y_max = y_range
            x_min = self.map_x(x1)
            x_max = self.map_x(x1)
            # return self.map_x(x1), y_min, z1, self.map_x(x2), y_max, z2
            lower_x_range = fix_range(x1, x2, z1 - 1)
            upper_x_range = fix_range(x1, x2, z1 + 1)
            if lower_x_range is None and upper_x_range is None:
                return None
            if lower_x_range is not None and upper_x_range is not None:
                x_min = min(lower_x_range[0], upper_x_range[0])
                x_max = max(lower_x_range[1], upper_x_range[1])
            elif lower_x_range is not None:
                x_min, x_max = lower_x_range
            else:
                x_min, x_max = upper_x_range
            return x_min, y_min, z1, x_max, y_max, z2
        else:
            x_range = fix_range(x1, x2, z1)
            if x_range is None:
                return None
            x_min, x_max = x_range
            # return x_min, self.map_y(y1), z1, x_max, self.map_y(y2), z2
            lower_y_range = fix_range(y1, y2, z1 - 1)
            upper_y_range = fix_range(y1, y2, z1 + 1)
            if lower_y_range is None and upper_y_range is None:
                return None
            if lower_y_range is not None and upper_y_range is not None:
                y_min = min(lower_y_range[0], upper_y_range[0])
                y_max = max(lower_y_range[1], upper_y_range[1])
            elif lower_y_range is not None:
                y_min, y_max = lower_y_range
            else:
                y_min, y_max = upper_y_range
            return x_min, y_min, z1, x_max, y_max, z2


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", type=str)
    parser.add_argument("-o", type=str)
    parser.add_argument("--write_segments", type=bool, default=True)
    parser.add_argument("--image", type=str)
    parser.add_argument("--remove_empty_net", type=bool, default=False)
    parser.add_argument("--remove_non_bottom_pins", type=bool, default=False)
    args = parser.parse_args()

    if os.path.isdir(args.i):  # convert all cases in directory
        shutil.copytree(args.i, args.o)
        illegal_testcases: list[str] = []
        for root, dirs, files in os.walk(args.o):
            for file in files:
                in_file = os.path.join(root, file)
                if re.match(r"^id_[0-9]+[.]txt$", file):
                    if not re.match(r".*level_[0-9]+$", root):
                        print(root)
                        break
                    converter = Converter(
                        in_file, args.remove_empty_net, args.remove_non_bottom_pins
                    )
                    converter.serialize(in_file, args.write_segments)
                    if len(converter.nets) == 0:
                        illegal_testcases.append(in_file)
        for in_file in illegal_testcases:
            tokens = in_file[: -len(".txt")].split("_")
            id = int(tokens[-1])
            replace_file = "_".join(tokens[:-1]) + "_" + str(id - 1) + ".txt"
            print(f"testcase without net found: {in_file}")
            shutil.copy(replace_file, in_file)
            print(f"replaced by: {replace_file}")

    elif os.path.isfile(args.i):  # convert a single file
        converter = Converter(args.i)
        if args.o is not None:
            converter.serialize(args.o, args.write_segments)
        if args.image is not None:
            converter.visualize(args.image)
    else:
        raise ValueError("value of '-i' is neither file nor directory!")
