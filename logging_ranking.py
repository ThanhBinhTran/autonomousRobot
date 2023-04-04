import csv
import os

import Program_config
from RRTree_star import RRTree_star
from Tree import Node


class Logging_ranking:
    def __init__(self) -> None:
        self.coord_key = "coordination"
        self.lmc_key = "lmc"
        self.cost_key = "cost"
        self.parent_key = "parent"
        self.children_key = "children"
        self.neighbours_key = "neighbours"
        self.neighbours_weight_key = "neighbours_weight"

    @staticmethod
    def set_logging_name(map_name, start, goal, radius=10, step_size=5, sample_size=10):

        return f"tree_data_{map_name}_s({start[0]}_{start[1]})_g({goal[0]}_{goal[1]})" + \
               f"_radius{radius}_step{step_size}_sample{sample_size}.csv"

    @staticmethod
    def is_existed_log_file(file_name):
        full_path = os.path.join(Program_config.result_repo, file_name)
        return os.path.exists(full_path)

    def write_nodes_info(self, node: Node, file_writer):
        result_items = [node.coords[0], node.coords[1], node.cost]
        if node.parent is not None:
            result_items.append(node.parent.coords[0])
            result_items.append(node.parent.coords[1])
        else:
            result_items.append("")
            result_items.append("")
        file_writer.writerow(result_items)
        for childnode in node.children:
            self.write_nodes_info(childnode, file_writer=file_writer)

    ''' logging tree as json format '''

    def save_tree(self, rrtree_star: RRTree_star, file_name):
        isExist = os.path.exists(Program_config.result_repo)
        if not isExist:
            os.mkdir(Program_config.result_repo)

        # Join various path components
        full_path = os.path.join(Program_config.result_repo, file_name)

        f = open(full_path, 'w', newline='', encoding="utf-8")
        writer = csv.writer(f, delimiter=",")
        writer.writerow(["coordinate_x", "coordinate_y", "cost",
                         "parent_coordinate_x", "parent_coordinate_y"])
        self.write_nodes_info(node=rrtree_star.root, file_writer=writer)
        # for result_items in self.results_data:
        #    writer.writerow(result_items)
        f.close()

    @staticmethod
    def load(file_name):
        first_line = True
        first_tree = True
        RRtree_star = None
        isExist = os.path.exists(Program_config.result_repo)
        if not isExist:
            os.mkdir(Program_config.result_repo)

        # Join various path components
        full_path = os.path.join(Program_config.result_repo, file_name)

        with open(full_path, newline='') as f:
            reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
            for row in reader:
                if first_line:
                    first_line = False
                    continue
                else:
                    coordinate = tuple((float(row[0]), float(row[1])))
                    cost = float(row[2])
                    new_node = Node(coords=coordinate, cost=cost)
                    if first_tree:
                        RRtree_star = RRTree_star(new_node)
                        first_tree = False
                    else:
                        parent_coordinate = float(row[3]), float(row[4])
                        parent_node = RRtree_star.get_node_by_coords(parent_coordinate)
                        RRtree_star.add_node(new_node)
                        RRtree_star.node_link(parent_node=parent_node, node=new_node)
        return RRtree_star
