import sys
import numpy as np

if len(sys.argv) < 2:
    print "usage: python visualize_custom_wind_field.py {input_file}.txt"
    exit(1)

try:
    f = open(sys.argv[1])
except:
    print "File '" + sys.argv[1] + "' does not exist!"
    exit(1)

name = ""
data = dict()
for i in range(26):
    val = f.readline()
    if i%2 == 0:
        val = str(val)
        val = val.replace(" ", "")
        val = val.replace("\n", "")
        val = val.replace(":", "")
        name = val
    else:
        line = np.fromstring(str(val), sep=" ")
        if (len(line) == 1):
            line = line[0]
        data[name] = line
        
min_x = data["min_x"]
min_y = data["min_y"]
n_x = int(data["n_x"])
n_y = int(data["n_y"])
res_x = data["res_x"]
res_y = data["res_y"]
vertical_spacing_factors = data["vertical_spacing_factors"]
bottom_z = data["bottom_z"]
top_z = data["top_z"]
u_vec = data["u"]
v_vec = data["v"]
w_vec = data["w"]

n_z = len(vertical_spacing_factors)

def ijk2node(i,j,k):
    return int(i + j*n_x + k*n_x*n_y)

def ij2node(i,j):
    return int(i + j*n_x)

points = list()
for k in range(n_z):
    for j in range(n_y):
        for i in range(n_x):
            n3 = ijk2node(i,j,k)
            n2 = ij2node(i,j)
            x = min_x + i*res_x
            y = min_y + j*res_y
            z = bottom_z[n2] + vertical_spacing_factors[k]*(top_z[n2] - bottom_z[n2])
            
            u = u_vec[n3]
            v = v_vec[n3]
            w = w_vec[n3]
            
            points.append([n3, x, y, z, u, v, w])

lv = 0
cells = list()
for k in range(n_z-1):
    for j in range(n_y-1):
        for i in range(n_x-1):
            n0=ijk2node(i,j,k)
            n1=ijk2node(i+1,j,k)
            n2=ijk2node(i+1,j+1,k)
            n3=ijk2node(i,j+1,k)
            n4=ijk2node(i,j,k+1)
            n5=ijk2node(i+1,j,k+1)
            n6=ijk2node(i+1,j+1,k+1)
            n7=ijk2node(i,j+1,k+1)

            # 6 tetraedra for each cube
            cells.append([lv+0,n0,n1,n2,n4])
            cells.append([lv+1,n1,n2,n4,n5])
            cells.append([lv+2,n2,n4,n5,n6])
            cells.append([lv+3,n0,n2,n3,n6])
            cells.append([lv+4,n0,n3,n4,n6])
            cells.append([lv+5,n3,n4,n6,n7])
            
            lv += 6

nbr_points = len(points)
nbr_cells = len(cells)

out  = '<?xml version="1.0" encoding="UTF-8"?>\n\n'
out += '<VTKFile type="UnstructuredGrid" version="0.1">\n'
out += '  <UnstructuredGrid>\n'
out += '    <Piece NumberOfPoints="'+str(nbr_points)+'" NumberOfCells="'+str(nbr_cells)+'">\n'
out += '      <Points>\n'
out += '        <DataArray type="Float64" NumberOfComponents="3" format="ascii">\n          '
for p in points:
    out += str(p[1])+' '+str(p[2])+' '+str(p[3])+' '
out += '\n'
out += '        </DataArray>\n'
out += '      </Points>\n'
out += '      <Cells>\n'
out += '        <DataArray type="UInt32" Name="connectivity" format="ascii">\n          '
for c in cells:
    out += str(c[1])+' '+str(c[2])+' '+str(c[3])+' '+str(c[4])+' '
out += '\n'
out += '        </DataArray>\n'
out += '        <DataArray type="UInt32" Name="offsets" format="ascii">\n          '
for c in range(nbr_cells):
    out += str((c+1)*4)+' '
out += '\n'
out += '        </DataArray>\n'
out += '        <DataArray type="UInt8" Name="types" format="ascii">\n          '
for c in range(nbr_cells):
    out += '10 '
out += '\n'
out += '        </DataArray>\n'
out += '      </Cells>\n'
out += '      <PointData Vectors="u_0">\n'
out += '        <DataArray type="Float64" Name="u_0" NumberOfComponents="3" format="ascii">\n'
for p in points:
    out += str(p[4])+' '+str(p[5])+' '+str(p[6])+' '
out += '\n'
out += '        </DataArray>\n'
out += '      </PointData>\n'
out += '    </Piece>\n'
out += '  </UnstructuredGrid>\n'
out += '</VTKFile>\n'

f = open("u_vis.vtu", "w")
f.write(out)
f.close()

points = list()
for k in range(2):
    for j in range(n_y):
        for i in range(n_x):
            n3 = ijk2node(i,j,k)
            n2 = ij2node(i,j)
            x = min_x + i*res_x
            y = min_y + j*res_y
            if (k == 1):
                z = 0
            else:
                z = bottom_z[n2]
                
            points.append([n3,x,y,z])
            
lv = 0
cells = list()
for k in range(1):
    for j in range(n_y-1):
        for i in range(n_x-1):
            n0=ijk2node(i,j,k,)
            n1=ijk2node(i+1,j,k,)
            n2=ijk2node(i+1,j+1,k,)
            n3=ijk2node(i,j+1,k,)
            n4=ijk2node(i,j,k+1,)
            n5=ijk2node(i+1,j,k+1,)
            n6=ijk2node(i+1,j+1,k+1,)
            n7=ijk2node(i,j+1,k+1,)
            
            # 6 tetraedra for each cube
            cells.append([lv+0,n0,n1,n2,n4])
            cells.append([lv+1,n1,n2,n4,n5])
            cells.append([lv+2,n2,n4,n5,n6])
            cells.append([lv+3,n0,n2,n3,n6])
            cells.append([lv+4,n0,n3,n4,n6])
            cells.append([lv+5,n3,n4,n6,n7])
            
            lv += 6

nbr_points = len(points)
nbr_cells = len(cells)

out  = '<?xml version="1.0" encoding="UTF-8"?>\n\n'
out += '<VTKFile type="UnstructuredGrid" version="0.1">\n'
out += '  <UnstructuredGrid>\n'
out += '    <Piece NumberOfPoints="'+str(nbr_points)+'" NumberOfCells="'+str(nbr_cells)+'">\n'
out += '      <Points>\n'
out += '        <DataArray type="Float64" NumberOfComponents="3" format="ascii">\n          '
for p in points:
    out += str(p[1])+' '+str(p[2])+' '+str(p[3])+' '
out += '\n'
out += '        </DataArray>\n'
out += '      </Points>\n'
out += '      <Cells>\n'
out += '        <DataArray type="UInt32" Name="connectivity" format="ascii">\n          '
for c in cells:
    out += str(c[1])+' '+str(c[2])+' '+str(c[3])+' '+str(c[4])+' '
out += '\n'
out += '        </DataArray>\n'
out += '        <DataArray type="UInt32" Name="offsets" format="ascii">\n          '
for c in range(nbr_cells):
    out += str((c+1)*4)+' '
out += '\n'
out += '        </DataArray>\n'
out += '        <DataArray type="UInt8" Name="types" format="ascii">\n          '
for c in range(nbr_cells):
    out += '10 '
out += '\n'
out += '        </DataArray>\n'
out += '      </Cells>\n'
out += '      <PointData Vectors="terrain">\n'
out += '        <DataArray type="Float64" Name="terrain" NumberOfComponents="3" format="ascii">\n'
for p in points:
    out += str(p[1])+' '+str(p[2])+' '+str(p[3])+' '
out += '\n'
out += '        </DataArray>\n'
out += '      </PointData>\n'
out += '    </Piece>\n'
out += '  </UnstructuredGrid>\n'
out += '</VTKFile>\n'

f = open("terrain.vtu", "w")
f.write(out)
f.close()

print "Files 'u_vis.vtu' and 'terrain.vtu' sucessfully generated"