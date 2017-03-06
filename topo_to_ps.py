import sys

if len(sys.argv) != 2:
	print "Incorrect usage. Please supply a topology data file as argument."
	print "python topo_to_ps [data_file]"
else:
	topo = open(sys.argv[1])
	graph = open("graph.ps", 'w')
	scale = 170.0 / 3

	for line in topo.readlines():
		node = line.split()
		id = int(node[0])
		x = float(node[1])
		y = float(node[2])

		graph.write("newpath %f %f 2.5 0 360 arc fill stroke\n" % (x * scale + 15.0, y * scale + 15.0))
		graph.write("/Times-Roman findfont 14 scalefont setfont newpath %f %f moveto (%d) show\n" % (x * scale + 18.0, y * scale + 18.0, id))
