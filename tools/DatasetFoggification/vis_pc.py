import os
import numpy as np
import struct
import open3d as o3d

def read_bin_velodyne(path):
    pc_list=[]
    with open(path,'rb') as f:
        content=f.read()
        pc_iter=struct.iter_unpack('ffff',content)
        for idx,point in enumerate(pc_iter):
            pc_list.append([point[0],point[1],point[2]])
    return np.asarray(pc_list,dtype=np.float32)

def main():
    #root_dir='C:/simulator/SeeingThroughFog/tools/DatasetFoggification/example_data/LidarData/'
    file_name = 'C:/simulator/SeeingThroughFog/tools/DatasetFoggification/example_data/LidarData/2019-09-11_19-13-44_00960.bin'
    #filename=os.listdir(root_dir)
    #file_number=len(filename)

    points = np.fromfile(file_name, dtype=np.float32)
    #points = points.reshape(-1, 4)
    points = points.reshape((-1, 5))[:,0:3]

    #points = points[:, :3]

    pcd=o3d.open3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    """
    for i in range(file_number):
        path=os.path.join(root_dir, filename[i])
        print(path)
        example=read_bin_velodyne(path)
        # From numpy to Open3D
        pcd.points= open3d.open3d.utility.Vector3dVector(example)
        open3d.open3d.visualization.draw_geometries([pcd])
    */"""
    o3d.open3d.visualization.draw_geometries([pcd])
    #o3d.open3d.visualization.draw([pcd])

if __name__=="__main__":
    main()