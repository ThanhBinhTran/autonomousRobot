# path for window
from pathlib import Path
import io
import sys, getopt

# python program -p <pattern>

try:
    opts, args = getopt.getopt(sys.argv[1:],"hp:r:", ["pattern=","replace="])
except getopt.GetoptError:
    print ('python find -p <pattern> -r <replace>')
    sys.exit(2)

patternStr = ""
for opt, arg in opts:
    if opt == '-h':
        print ('python find -p <pattern>')
        sys.exit()
    elif opt in ("-p", "--pattern"):
            patternStr = arg
            print ("DEBUG ", opt, arg, patternStr)
    elif opt in ("-r", "--replace"):
            replace = arg
            print ("DEBUG ", opt, arg, patternStr)
    else:
        print ('python find -p <pattern>')
        sys.exit()

# path for window
from pathlib import Path

# import input/output handling
import io

print ("Finding ", patternStr)

searchdir = Path(r".")
tex_file_lists = [str(pp) for pp in searchdir.glob("*.py")]
resultfile = []

for texfile in tex_file_lists:
    f = open(texfile, encoding='utf-8')
    data = f.readlines()
    for line in data:
        if line.find(patternStr) != -1:
            print ("FOUND ", texfile)
            resultfile.append(texfile)
            break
            
for texfile in resultfile:
    f = open(texfile, encoding='utf-8')
    data = f.readlines()
    rlines = []
    for line in data:
        rlines.append(line.replace(patternStr,replace))
    
    #write file
    with io.open(texfile, 'w', encoding='utf8') as fw:
        for update_line in rlines:
            fw.write(update_line)
    fw.close()

