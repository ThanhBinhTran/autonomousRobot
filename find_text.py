# path for window
from pathlib import Path
import io
import sys, getopt

# python program -p <pattern>

try:
    opts, args = getopt.getopt(sys.argv[1:],"hp:", ["pattern="])
except getopt.GetoptError:
    print ('python find -p <pattern>')
    sys.exit(2)

patternStr = ""
for opt, arg in opts:
    if opt == '-h':
        print ('python find -p <pattern>')
        sys.exit()
    elif opt in ("-p", "--pattern"):
            patternStr = arg
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
results = []
for texfile in tex_file_lists:
    f = open(texfile, encoding='utf-8')
    data = f.readlines()
    for line in data:
        if line.find(patternStr) != -1:
            print ("FOUND ", texfile)
            results.append(texfile)
            break
            
## log the result
#with io.open("result_file.txt", 'w', encoding='utf8') as f:
#    for filename in results:
#            f.write(filename + "\n")
#f.close()
