from subprocess import Popen, PIPE
import sys

for density in range(1,21):
#	density = int(sys.argv[1]
	run_cmd = "wsnet -c geocast_density_%d.xml" % (density)
	save_cmd = "mv results.txt results_%d.txt" % (density)

	print run_cmd
	print save_cmd

	i = 0
	j = 0
	k = 0
	while i < 1000:
	        j += 1
		p = Popen(run_cmd , shell=True, stdout=PIPE, stderr=PIPE)
		out, err = p.communicate()	

		if "<failed>" not in out and "<failed>" not in err:
			i += 1
			print i / 10.0, "% complete ... "
	        else:
			k += 1
	#		print "failed - ", j


	p = Popen(save_cmd , shell=True, stdout=PIPE, stderr=PIPE)
	out, err = p.communicate()

	#print "Done! - %d,%d,%d" % (i,j,k)
	print "Done!"

