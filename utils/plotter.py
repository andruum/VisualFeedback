import csv

import matplotlib.pyplot as plt
import numpy as np

from utils.plot_data import extract_results

LINE_STYLES = ['-', '--', '-.', ':',  (5, (4, 2)) ]

class Plotter():

    def __init__(self, title = "",
                 xaxisname = 'Время, с',
                 yaxisname = 'Угол, град',
                 xaxis_id = -1,
                 label_prefix_plot = "Положение звена"):
        self.data = []
        self.title = title
        self.xaxisname = xaxisname
        self.yaxisname = yaxisname
        self.xaxis_id = xaxis_id
        self.label_prefix_plot = label_prefix_plot


    def addData(self, data_array):
        self.data.append(data_array)

    def plotData(self, starttime, endtime, timescale=1):
        data = extract_results(self.data, starttime, endtime, timescale)
        data = np.asarray(data)

        with open('plotter.csv', mode='w') as data_csv:
            writer_csv = csv.writer(data_csv, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for datarow in data[:]:
                writer_csv.writerow(datarow)

        for i in range(len(data[0]) - 1):
            plt.plot(data[:, self.xaxis_id], data[:, i], label=self.label_prefix_plot + str(i + 1),
                     linestyle=LINE_STYLES[i])

        plt.xlabel(self.xaxisname )
        plt.ylabel(self.yaxisname)
        plt.title(self.title)

        plt.legend()
        plt.show()