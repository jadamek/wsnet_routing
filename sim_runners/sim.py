from subprocess import Popen, PIPE
import sys

density = int(sys.argv[1])
run_cmd = "wsnet -c geocast_density_%d.xml" % (density)
save_cmd = "mv results.txt results_%d.txt" % (density)

print run_cmd
print save_cmd

i = 0
while i < 100:
	p = Popen(run_cmd , shell=True, stdout=PIPE, stderr=PIPE)
	out, err = p.communicate()

	if "<failed>" not in out and "<failed>" not in err:
		i += 1
		print i, "% complete ..."

p = Popen(save_cmd , shell=True, stdout=PIPE, stderr=PIPE)
out, err = p.communicate()

print "Done!"

