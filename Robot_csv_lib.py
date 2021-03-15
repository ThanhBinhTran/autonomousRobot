import csv

def write_map_csv(file_name, f_data, data_header):
    f = open(file_name, 'w', newline='', encoding="utf-8")
    writer = csv.writer(f, delimiter=",")
    writer.writerow(data_header)
    for pt in f_data:
        writer.writerow([int(pt[0]),int(pt[1])])
    f.close()

def read_map_csv(mapname):
    first_line = True
    ob=[]
    
    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            if first_line:
                first_line = False
                continue
            ob.append([float(row[0]),float(row[1])])
    return ob
    
def read_map_csv_dwa(mapname):
    first_line = True
    ob=[]
    
    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            if first_line:
                first_line = False
                continue
            ob.append([float(row[0])/6.6,float(row[1])/6.6])
    return ob