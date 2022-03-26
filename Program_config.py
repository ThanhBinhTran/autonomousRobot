'''
Parameters to show/hide element(s) of animation 
'''
show_animation = True
show_map = True
show_world = False

show_traversalSights = True

show_openSight = True
show_closedSight = True
show_circleRange = False

show_sketelonPath = True
show_approximately_shortest_path = True
show_critical_line_segments = True
show_cls_orderednumber = True  # show ordered number of critical line segments

show_visitedPath = True
show_visibilityGraph = True

show_local_openpt = False
show_active_openpt = True
show_next_point = True

show_robot = True
show_goal = True
show_start = True
show_text_goal = True

'''
Assumption of Hoai An Theory.
'''
# show animation
show_active_openpt_HA_Assumption = True         # active open points (HA Assumption)
show_activeArc_HA_Assumption = True             # arc points (HA Assumption)
show_parentArc_HA_Assumption = True             # parent arc points (HA Assumption)
show_boundaryPts_HA_Assumption = True           # boundary points intersection between circle and obstacles
show_circleGoal_HA_Assumption = True            # big circle (at goal within radius = dist(goal, robot center))
show_activeArcPts_order_HA_Assumption = True    # show order of arc points according to its angles
# line styles
ls_bp_HA = "k."     # boundary points (HA Assumption)
ls_aap_HA = "c."    # active arc points (HA Assumption)
ls_pap_HA = "go"    # parent arc points (HA Assumption)

transparent = 0.1   # closed sights
'''
parementers to hide/show information
'''
print_local_obstacles_boundary = False
print_closed_sights = False
print_open_sights = False
print_ref_csight_line_segments = False
print_csight_line_segments = False
print_traversalSights = False
print_visited_path = False

'''
stype for robot
'''
ls_ts = "-"  # true sight
cl_ts = "m"  # color true sight
ls_goal = "*r"  # goal
ls_start = "*b"  # start
ls_nextpt = ".r"  # next points
ls_lopt = ".k"  # local open_point
ls_aopt = ".b"  # active open points
ls_vg = ":k"  # visibility graph
ls_vp = "-r"  # visited path
ls_goingp = "-1g"  # going path
ls_cls = '--g'  # critical line segment
ls_sp = "-r"  # shortest skeleton path
ls_asp = "-m"  # approximately shortest path
ls_cspace = "-r"  # configuration space

''' stype for RRTree'''
ls_tree_node_active = ".b"         # tree's node
ls_tree_node_inactive = ".k"
ls_tree_edge = "-k"         # tree's edge
ls_goal_path_edge = "-g"    # path to goal edge
ls_goal_path_node = "1r"    # path to goal node
ls_node_active = "or"


################################################
# instead of show plot, saving image
################################################
easy_experiment = True
save_image = True       # set easy_experiment = True to enable save_image
g_strategy = "global"     # pick on open point global set
l_strategy = "local"     # pick on open point local set first, if not exist go for global


################################################
# Note: in case there is no obstracles:
# the system will create 6 open points if divide_6 = True
# if divide_6 = false, 3 open points will be created
divide_6 = True
