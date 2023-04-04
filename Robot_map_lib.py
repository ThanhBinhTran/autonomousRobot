win_size = 100
from Program_config import show_plot_title


class Map:

    @staticmethod
    def generate(plt, i, part, MAX_VERTICES):
        # displaying the title 
        plt.title("Obstacle part[{0}/{1}]: left click to input point, right click to undo (1 step)\n".format(i, part) +
                  "middle click to turn to next obstacle, MAX_VERTICES={0}".format(MAX_VERTICES))
        plt.axis([0, win_size, 0, win_size])

        return plt.ginput(MAX_VERTICES, show_clicks=True, timeout=-1)  # no timeout

    @staticmethod
    def display(plt, title, obstacles, alpha: float = 0.4, hatch: str = '//'):

        # displaying the title 
        if show_plot_title:
            plt.title(title)
        for obstacle in obstacles:
            x = [point[0] for point in obstacle]
            y = [point[1] for point in obstacle]
            x.append(obstacle[0][0])
            y.append(obstacle[0][1])
            plt.fill(x, y, color='k', alpha=alpha, hatch=hatch)
