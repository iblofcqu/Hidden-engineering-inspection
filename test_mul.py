import copy
import time
import math
import numpy as np
import open3d as o3d
import os

#依据采集数据位姿进行粗配准
class PointCloudRegistration:
    def __init__(self, txt_rot_path, txt_dis_path, ply_folder_path, save_folder_path):
        self.txt_rot_path = txt_rot_path
        self.txt_dis_path = txt_dis_path
        self.ply_folder_path = ply_folder_path
        self.save_folder_path = save_folder_path
        self.txt_rot = None
        self.txt_dis = None
        self.row_txt_rot = 0
        self.row_txt_dis = 0
        self.points = {}
        self.colors = {}
        self.name = []
        self.Ply_data = {}

    @staticmethod
    def read_data(file_path_name):
        with open(file_path_name, "r", encoding="utf-8") as file:
            lines = file.readlines()
        data_list = []
        for line in lines:
            line = line.strip('\n')
            line = str.split(line, ',')
            line = np.array(line, dtype=np.float64)
            data_list.append(line)
        data_list = np.array(data_list)
        return data_list, data_list.shape[0], data_list.shape[1]

    def read_ply(self):
        for i in range(self.row_txt_rot):
            name = str(i + 1) + '.ply'
            path = os.path.join(self.ply_folder_path, name)
            self.name.append(name)
            self.Ply_data[i] = o3d.io.read_point_cloud(path)
            self.points[i] = np.asarray(self.Ply_data[i].points)
            self.colors[i] = np.asarray(self.Ply_data[i].colors)

    @staticmethod
    def transformation(angles, displacement, points_0):
        a_x, a_y = angles[0] * math.pi / 180, angles[1] * math.pi / 180
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(a_x), -np.sin(a_x)],
                        [0, np.sin(a_x), np.cos(a_x)]])
        R_y = np.array([[np.cos(a_y), 0, np.sin(a_y)],
                        [0, 1, 0],
                        [-np.sin(a_y), 0, np.cos(a_y)]])
        points = copy.deepcopy(points_0)
        for i in range(len(points_0)):
            a = points[i, :]
            a[0] += displacement[0][0]
            a[1] -= displacement[0][1]
            a[2] += displacement[0][2]
            a = np.matmul(R_x, np.transpose(a))
            a[1] -= displacement[1][1]
            a = np.matmul(R_y, a)
            points[i] = np.transpose(a)
        return points

    @staticmethod
    def ply_vision(points, colors, n, f):
        if f == 1:
            for i in range(len(n)):
                combined_pcd = o3d.geometry.PointCloud()
                combined_pcd.points = o3d.utility.Vector3dVector(points[n[i] - 1])
                combined_pcd.colors = o3d.utility.Vector3dVector(colors[n[i] - 1])
                o3d.visualization.draw_geometries([combined_pcd], width=800, height=600)
        elif f == 2:
            combined_pcd = o3d.geometry.PointCloud()
            for i in range(len(n)):
                combined_pcd.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(combined_pcd.points),
                                                                            points[n[i] - 1])))
                combined_pcd.colors = o3d.utility.Vector3dVector(np.vstack((np.asarray(combined_pcd.colors),
                                                                            colors[n[i] - 1])))
            o3d.visualization.draw_geometries([combined_pcd], width=800, height=600)

    def run(self):
        Start_time = time.time()
        # 读取旋转和平移数据
        self.txt_dis, self.row_txt_dis, _ = self.read_data(self.txt_dis_path)
        self.txt_rot, self.row_txt_rot, _ = self.read_data(self.txt_rot_path)
        print("---------已读取转角与平移数据---------")

        # 读取点云数据
        self.read_ply()
        print("---------已读取并处理点云数据---------")

        # 相机坐标系 -> 框架坐标系转换
        angle_x = [self.txt_rot[i][0] for i in range(self.row_txt_rot)]
        angle_y = [-(self.txt_rot[i][1] + 180) for i in range(self.row_txt_rot)]
        displacement = np.array([[167.915, 94.1067, 56.2], [0, 84.9424, 0]])
        points_calibra = copy.deepcopy(self.points)

        for i in range(self.row_txt_rot):
            angles = [angle_x[i], angle_y[i]]
            points_calibra[i] = self.transformation(angles, displacement, self.points[i])
        print("已完成第一步，相机坐标系转换到框架坐标系")

        # 坐标平移对齐配准
        points_regist = copy.deepcopy(points_calibra)
        for i in range(self.row_txt_dis):
            print("开始对第" + str(i + 1) + "个点云数据进行配准！")
            for j in range(i, self.row_txt_dis):
                for k in range(len(points_regist[0])):
                    for l in range(3):
                        points_regist[i][k][l] += self.txt_dis[j][l]
            save_path = os.path.join(self.save_folder_path, self.name[i])
            combined_pcd = o3d.geometry.PointCloud()
            combined_pcd.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(combined_pcd.points),
                                                                        points_regist[i])))
            combined_pcd.colors = o3d.utility.Vector3dVector(np.vstack((np.asarray(combined_pcd.colors),
                                                                        self.colors[i])))
            o3d.io.write_point_cloud(save_path, combined_pcd)

        # 保存最后一帧未变换点云
        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(combined_pcd.points),
                                                                    points_regist[self.row_txt_dis])))
        combined_pcd.colors = o3d.utility.Vector3dVector(np.vstack((np.asarray(combined_pcd.colors),
                                                                    self.colors[self.row_txt_dis])))
        o3d.io.write_point_cloud(os.path.join(self.save_folder_path, self.name[self.row_txt_dis]), combined_pcd)

        print("完成点云配准")
        End_time = time.time()
        print("Time:%.5f" % (End_time - Start_time) + "s")

        # 显示结果
        n = [i + 1 for i in range(self.row_txt_rot)]
        self.ply_vision(points_regist, self.colors, n, 2)

if __name__ == '__main__':
    processor = PointCloudRegistration(
        txt_rot_path="./转角.txt",
        txt_dis_path="./平移量.txt",
        ply_folder_path="./ply/",
        save_folder_path="./results/"
    )
    processor.run()

