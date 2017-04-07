densities = [32,64,95,127,159,191,223,255,286,318,350,382,414,446,477,509,541,573,605,637]

source = open('source.xml', 'r')
source_lines = [line for line in source]

for density in range(20):
	source_lines[5] = '<simulation nodes="%d" duration="30s" x="100" y="100" z="0"/>\n' % (densities[density])
	config = open("geocast_density_%d.xml" % (density + 1), 'w')
	config.writelines(source_lines)
	config.close()

source.close()
print "Done!"
