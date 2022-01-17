import sys

if len(sys.argv) != 2:
    print("bad arg num:", sys.argv)
else:
    fname = sys.argv[1]
    xVals = []
    yVals = []
    with open(fname) as f:
        for l in f.readlines():
            x, y = l.strip().split()
            xVals.append(x)
            yVals.append(y)
    print("plot([" + ','.join(xVals)  + "], [" + ','.join(yVals) + "], 'linestyle', 'none', 'marker', '.', 'markerSize', 2);")




