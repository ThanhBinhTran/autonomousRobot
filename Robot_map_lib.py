win_size = 100
from Program_config import ls_map
def map_generator(plt, N):
    # displaying the title 
    plt.title("click on plot to generate {0} points of map".format(N))
    plt.axis([0,win_size,0,win_size])

    return plt.ginput(N,show_clicks=True, timeout=-1) # no timeout
    
def map_display(plt, mapname, ob):
    # displaying the title 
    plt.title("Display map: {0}".format(mapname))
    plt.plot(ob[:,0], ob[:,1], ls_map)
