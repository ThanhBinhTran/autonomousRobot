"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import pandas as pd
import argparse
import matplotlib.pyplot as plt

class Experimental_Astar_Asp():

    ''' visualizate result from result file '''
    def result_plot(self, result_file):
        # read result as frame
        result_data = pd.read_csv(result_file)
        df_time = result_data[['asp_time', 
                'Astar_time',
                'RRTStar_time'
            ]]
        df_time.plot(logy=False, kind="bar")
        #data = np.log2(df_time[['asp_time', 'Astar_time']])
        
        #data.plot.bar()
        df_path_cost = result_data[['asp_path_cost', 
            'Astar_path_cost',
            'RRTStar_path_cost'
            ]]
        df_path_cost.plot(kind="bar")
        plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-r', metavar="result file",  help='result file', default="result.csv")

    menu_result = parser.parse_args()

    # get user input
    result_file = menu_result.r

    result = Experimental_Astar_Asp()
    result.result_plot(result_file=result_file)