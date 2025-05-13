import os
import copy
import time
import math
import numpy as np
import open3d as o3d


class PointCloudRegistration:
    def __init__(self, txt_rot_path, txt_dis_path, ply_folder_path, save_folder_path, icp_save_path):
        self.txt_rot_path = txt_rot_path
        self.txt_dis_path = txt_dis_path
        self.ply_folder_path = ply_folder_path
        self.save_folder_path = save_folder_path
        self.icp_save_path = icp_save_path
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
        data_list = [np.array(str.split(line.strip(), ','), dtype=np.float64) for line in lines]
        return np.array(data_list), len(data_list), len(data_list[0])

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
    def draw_registration_result(src, tar, transformation, save_path=None):
        source_temp = copy.deepcopy(src)
        target_temp = copy.deepcopy(tar)
        source_temp.transform(transformation)
        if save_path:
            o3d.io.write_point_cloud(save_path, source_temp)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          window_name="GICP配准结果", width=800, height=600)

    def run(self):
        Start_time = time.time()
        self.txt_dis, self.row_txt_dis, _ = self.read_data(self.txt_dis_path)
        self.txt_rot, self.row_txt_rot, _ = self.read_data(self.txt_rot_path)
        print("---------已读取转角与平移数据---------")

        self.read_ply()
        print("---------已读取并处理点云数据---------")

        angle_x = [self.txt_rot[i][0] for i in range(self.row_txt_rot)]
        angle_y = [-(self.txt_rot[i][1] + 180) for i in range(self.row_txt_rot)]
        displacement = np.array([[167.915, 94.1067, 56.2], [0, 84.9424, 0]])  # 需根据实际设备安装情况调整
        points_calibra = copy.deepcopy(self.points)

        for i in range(self.row_txt_rot):
            angles = [angle_x[i], angle_y[i]]
            points_calibra[i] = self.transformation(angles, displacement, self.points[i])
        print("已完成第一步，相机坐标系转换到框架坐标系")

        points_regist = copy.deepcopy(points_calibra)
        for i in range(self.row_txt_dis):
            print("开始对第" + str(i + 1) + "个点云数据进行配准！")
            for j in range(i, self.row_txt_dis):
                points_regist[i] += self.txt_dis[j]
            combined_pcd = o3d.geometry.PointCloud()
            combined_pcd.points = o3d.utility.Vector3dVector(points_regist[i])
            combined_pcd.colors = o3d.utility.Vector3dVector(self.colors[i])
            o3d.io.write_point_cloud(os.path.join(self.save_folder_path, self.name[i]), combined_pcd)

        # 保存最后一帧未变换点云
        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(points_regist[self.row_txt_dis])
        combined_pcd.colors = o3d.utility.Vector3dVector(self.colors[self.row_txt_dis])
        o3d.io.write_point_cloud(os.path.join(self.save_folder_path, self.name[self.row_txt_dis]), combined_pcd)

        print("完成点云粗配准")

        # ---------------- GICP 精配准 ----------------
        print("开始执行 Generalized ICP 两两精配准")
        for i in range(self.row_txt_dis):
            source_path = os.path.join(self.save_folder_path, f"{i+1}.ply")
            target_path = os.path.join(self.save_folder_path, f"{i+2}.ply")

            source_raw = o3d.io.read_point_cloud(source_path)
            target_raw = o3d.io.read_point_cloud(target_path)

            source = source_raw.voxel_down_sample(voxel_size=1)
            target = target_raw.voxel_down_sample(voxel_size=1)

            trans_init = np.eye(4)
            threshold = 5.0

            evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
            print(f"[Pair {i+1}-{i+2}] 初始 fitness: {evaluation.fitness:.4f}, RMSE: {evaluation.inlier_rmse:.4f}")

            result = o3d.pipelines.registration.registration_generalized_icp(
                source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))

            print(f"[Pair {i+1}-{i+2}] GICP fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.4f}")
            print("Transformation:\n", result.transformation)

            save_trans_path = os.path.join(self.icp_save_path, f"trans_of_{i+1}.ply")
            self.draw_registration_result(source_raw, target_raw, result.transformation, save_path=save_trans_path)

        End_time = time.time()
        print("全部完成，用时 %.3fs" % (End_time - Start_time))


if __name__ == '__main__':
    processor = PointCloudRegistration(
        txt_rot_path="./转角.txt",
        txt_dis_path="./平移量.txt",
        ply_folder_path="./ply/",
        save_folder_path="./results/",
        icp_save_path="./glb_icp/"
    )
    processor.run()
