'''
Parameters to show/hide element(s) of animation 
'''
show_animation = True
show_map = True
show_world = True

show_traversalSights = True

show_openSight = True
show_closedSight = True
show_circleRange = True

show_sketelonPath = True
show_approximately_shortest_path = True
show_critical_line_segments = True
show_cls_orderednumber = True  # show ordered number of critical line segments

show_visitedPath = True
show_visibilityGraph = True

show_local_openpt = True
show_active_openpt = True
show_next_point = True

show_robot = True
show_goal = True
show_start = True
show_text_goal = True

'''
    transparent 
'''
transparent = 0.1

'''
Assumption of Hoai An Theory.
'''
# ALGORITHMS CONTROL
THEORY_AR_ALGORITHMS = True
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

'''
parementers to hide/show information
'''
print_boundary_line_segments = False
print_closed_sights = False
print_closed_line_segments = False
print_open_sights = False
print_ref_sight = False
print_ref_csight_line_segments = False
print_csight_line_segments = False
print_traversalSights = False

'''
LINE STYPE FOR DISPLAY PLOT
'''
ls_is = ":.c"  # intersection
ls_bp = "-r"  # boundary points
ls_ts = "-"  # true sight
cl_ts = "m"  # color true sight
cl_os = "c"
ls_bp = ":g"  # blind sight
ls_map = "-b"  # map
ls_cs = ":m"  # close sight
ls_os = "c"  # open sight
ls_goal = "*r"  # goal
ls_start = "*b"  # goal
ls_nextpt = ".r"  # next points
ls_lopt = ".k"  # local open_point
ls_aopt = ".b"  # active open points
ls_em = "-m"  # explored_map
ls_vg = ":k"  # visible graph
ls_vp = "-r"  # visited path
ls_goingp = "-1g"  # going path
ls_cls = '--g'  # critical line segment
ls_sp = "-r"  # shortest skeleton path
ls_asp = "-m"  # approximately shortest path