from subprocess import Popen, PIPE
cmd = "wsnet -c test_geocast.xml"

i = 0

while i < 100:
	p = Popen(cmd , shell=True, stdout=PIPE, stderr=PIPE)
	out, err = p.communicate()

	if "<failed>" not in out and "<failed>" not in err:
		i += 1
		print i, "% complete ..."

print "Done!"
