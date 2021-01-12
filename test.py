def detect_blind_sight(center, ref_sight, check_sight):
    """ check sight  --> C0, C1
        reference sight -->  R0 R1 """
    print ("_____________________")
    
    true_sight = []
    blind_sight = []
    true_blind = False
    
    true_sight.append (ref_sight)
    
    pointC_0 = is_inside_area(check_sight[0], center, ref_sight)
    pointC_1 = is_inside_area(check_sight[1], center, ref_sight)

    if pointC_0 >= 0:
        plt.plot(check_sight[0][0], check_sight[0][1], "*r")
    if pointC_1 >= 0:
        plt.plot(check_sight[1][0], check_sight[1][1], "*r")


    if pointC_0 >= 0 and pointC_1 >= 0: # whole check_sight are inside ref_sight, true blind
        blind_sight.append (check_sight)
        true_blind = True
        
    elif pointC_0 == 0 or pointC_1 == 0: # ether C0 or C1 is at the boundary segment
        """ check if C0 C1 coverages ref_sight """
        pointR_0 = is_inside_area(ref_sight[0], center, check_sight)
        pointR_1 = is_inside_area(ref_sight[1], center, check_sight)
        if pointR_0 < 0 or pointR_1 < 0: # ref sight is outside of the area of AB
            true_sight.append(check_sight)
            
    elif pointC_0 > 0 and pointC_1 < 0: # C0 inside, C1 outside
        """ check if R0 is inside area of [R1, C1]
            if R0 is outside then R1 is inside area of [R0, C1] for sure
        """
        pointR0 = is_inside_area(ref_sight[0], center, [ref_sight[1], check_sight[1]])
        if pointR0:  """ R1 C0 R0 C1 """
            true_sight.append([ref_sight[0], check_sight[1]])
            blind_sight.append([check_sight[0], ref_sight[0]])
        else: """ R0 C0 R1 C1 """
            true_sight.append([ref_sight[1], check_sight[1]])
            blind_sight.append([check_sight[0], ref_sight[1]])
            
    elif pointC_0 < 0 and pointC_1 > 0: # C1 inside, C0 outside
        """ check if R0 is inside area of [R1, C0]
            if R0 is outside then R1 is inside area of [R0, C0] for sure
        """
        pointR0 = is_inside_area(ref_sight[0], center, [ref_sight[1], check_sight[0]])
        if pointR0:  """ R1 C1 R0 C0 """
            true_sight.append([ref_sight[0], check_sight[0]])
            blind_sight.append([check_sight[1], ref_sight[0]])
        else: """ R0 C1 R1 C0 """
            true_sight.append([ref_sight[1], check_sight[0]])
            blind_sight.append([check_sight[1], ref_sight[1]])
            
    else:   # both C1 and C2 are outside
        true_sight.append (check_sight)
        
    return [true_sight, blind_sight, true_blind]