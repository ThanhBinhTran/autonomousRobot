"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import csv
import pandas as pd
import os
import Program_config


class Result_Log:
    def __init__(self, header_csv=None):
        if header_csv is None:
            header_csv = ["header1"]
        self.results_data = []
        self.header_csv = header_csv

    ''' clear data'''

    def clear_data(self):
        self.results_data = []

    ''' save result in to list '''

    def add_result(self, result):
        self.results_data.append(result)

    @staticmethod
    def prepare_name(experiment_title=None, map_name=None, start=(0, 0), goal=(0, 0), pick=None, range=20, open_points_type=None):
        if map_name is None:
            return None
        mn = map_name.replace('.csv', '')

        if experiment_title is None:
            paths = os.path.join(Program_config.result_repo)
            fname = f"{mn}_s({start[0]}_{start[1]})_g({goal[0]}_{goal[1]})_range{range}"
        else:
            paths = os.path.join(Program_config.result_repo, experiment_title, mn)
            fname = f"s({start[0]}_{start[1]})_g({goal[0]}_{goal[1]})_range{range}"
        if not os.path.exists(paths):
            os.makedirs(paths)     

        if pick is not None:
            fname += f'_{pick}'

        if open_points_type is not None:
            fname += f'_{open_points_type}'

        return os.path.join(paths, fname)
    
    
    def set_file_name(self, name):
        self.file_name = name

    ''' set header '''

    def set_header(self, header_csv):
        self.header_csv = header_csv

    ''' write result to csv file '''

    def write_csv(self):
        if len(self.results_data) <= 0:
            print("No data was recorded")
            return

        f = open(self.file_name, 'w', newline='', encoding="utf-8")
        writer = csv.writer(f, delimiter=",")
        writer.writerow(self.header_csv)
        for result_items in self.results_data:
            writer.writerow(result_items)
        f.close()
        print (f"saved :{self.file_name}")

    ''' read csv as dataframe '''

    @staticmethod
    def read_csv_as_dataframe(result_file):
        # read result as frame
        return pd.read_csv(result_file)
