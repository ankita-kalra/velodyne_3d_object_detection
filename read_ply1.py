from plyfile import PlyData, PlyElement
import numpy as np
name1='car3.ply'
name2='car3.npy'
ply1=PlyData.read(name1)

temp=np.array(ply1['vertex']['x'][0])
temp=np.append(temp,ply1['vertex']['y'][0])
temp=np.append(temp,ply1['vertex']['z'][0])
car1=temp
print car1
car1 = []

for x,y,z in zip(ply1['vertex']['x'], ply1['vertex']['y'], ply1['vertex']['z']):
		car1.append(np.asarray([x, y, z], dtype=np.float32))
car1 = np.asarray(car1)

'''for i in range(len(ply1['vertex']['x'])):
	temp=np.array(ply1['vertex']['x'][i])
	temp=np.append(temp,ply1['vertex']['y'][i])
	temp=np.append(temp,ply1['vertex']['z'][i])
	car1.append(np.asarray(temp))
'''
np.save(name2,np.asarray(car1))