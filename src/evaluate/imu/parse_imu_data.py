filename = "imu_out.txt"

lines = []

with open(filename) as f:
	for line in f:
		line_stripped = line.rstrip('\n')
		#print "SEO: line: "+line_stripped
		lines.append(line_stripped)

with open("imu_massaged.txt", 'w') as fout:
	index = 0
	fout.write("ox,oy,oz,ow,ax,ay,az,lx,ly,lz\n")
	for line in lines:
		if line.startswith("orientation:"):
			o_x = lines[index+1][5:]
			o_y = lines[index+2][5:]
			o_z = lines[index+3][5:]
			o_w = lines[index+4][5:]

			a_x = lines[index+7][5:]
			a_y = lines[index+8][5:]
			a_z = lines[index+9][5:]

			l_x = lines[index+12][5:]
			l_y = lines[index+13][5:]
			l_z = lines[index+14][5:]

			fout.write(o_x+","+o_y+","+o_z+","+o_w+","+a_x+","+a_y+","+a_z+","+l_x+","+l_y+","+l_z+"\n")

		index = index+1
