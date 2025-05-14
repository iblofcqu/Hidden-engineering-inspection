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

    def estimate_pairwise_threshold(pcd1, pcd2, multiplier=2.0):
        """根据两个点云的平均最近邻距离计算自适应 threshold。"""
        dists1 = pcd1.compute_nearest_neighbor_distance()
        dists2 = pcd2.compute_nearest_neighbor_distance()
        avg_dist1 = np.mean(dists1)
        avg_dist2 = np.mean(dists2)
        avg_pair_dist = (avg_dist1 + avg_dist2) / 2
        return avg_pair_dist * multiplier

    @staticmethod
    def draw_registration_result(src, tar, transformation, save_path=None):
        target_temp = copy.deepcopy(tar)
        target_temp.transform(transformation)
        if save_path:
            o3d.io.write_point_cloud(save_path, target_temp)
        # o3d.visualization.draw_geometries([source_temp, target_temp],
        #                                   window_name="GICP配准结果", width=800, height=600)
        #此段代码为显示两两精确配准结果，可不必显示

    def estimate_pairwise_threshold(self, pcd1, pcd2, multiplier=2.0):
        """根据两个点云的平均最近邻距离计算自适应 threshold。"""
        dists1 = pcd1.compute_nearest_neighbor_distance()
        dists2 = pcd2.compute_nearest_neighbor_distance()
        avg_dist1 = np.mean(dists1)
        avg_dist2 = np.mean(dists2)
        avg_pair_dist = (avg_dist1 + avg_dist2) / 2
        return avg_pair_dist * multiplier

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
        #后续需要增加相邻两帧数据间的标定数据
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
        transform_accumulated = np.eye(4)

        # Step 1: 保存第 1 帧为基准帧（无变换）
        first_frame = o3d.io.read_point_cloud(os.path.join(self.save_folder_path, "1.ply"))
        first_save_path = os.path.join(self.icp_save_path, "trans_of_1.ply")
        o3d.io.write_point_cloud(first_save_path, first_frame)

        initial_threshold = 2
        threshold_step = 1

        for i in range(self.row_txt_dis):
            # source 从前一步的配准结果中读取
            source_path = os.path.join(self.icp_save_path, f"trans_of_{i + 1}.ply")
            # target 从原始数据中读取
            target_path = os.path.join(self.save_folder_path, f"{i + 2}.ply")

            source_raw = o3d.io.read_point_cloud(source_path)
            target_raw = o3d.io.read_point_cloud(target_path)

            # 下采样
            source = source_raw.voxel_down_sample(voxel_size=0.2)
            target = target_raw.voxel_down_sample(voxel_size=0.2)

            # 设置动态阈值,目前还是根据设备经验选择
            threshold_mot = initial_threshold + i * threshold_step
            # ⬅️ 自动计算 threshold（核心集成点）
            threshold = self.estimate_pairwise_threshold(source, target, multiplier=threshold_mot)#初始值差距过大的话，multiplier要选大一点
            print(f"[Pair {i + 1}-{i + 2}] 采用 threshold={threshold:.3f} 进行精配准")

            trans_init = np.eye(4)

            evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
            print(
                f"[Pair {i + 1}-{i + 2}] threshold={threshold} 初始 fitness: {evaluation.fitness:.4f}, RMSE: {evaluation.inlier_rmse:.4f}")

            result = o3d.pipelines.registration.registration_generalized_icp(
                source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))

            print(f"[Pair {i + 1}-{i + 2}] GICP fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.4f}")
            print("Transformation:\n", result.transformation)

            # 应用变换到 target_raw
            target_transformed = copy.deepcopy(target_raw)
            target_transformed.transform(transform_accumulated)

            save_trans_path = os.path.join(self.icp_save_path, f"trans_of_{i + 2}.ply")
            o3d.io.write_point_cloud(save_trans_path, target_transformed)

        print("所有 GICP 精配准后的点云已保存。")
        # ---------------- 合并点云 ----------------
        print("开始融合所有 GICP 精配准后的点云")

        merged_pcd = o3d.io.read_point_cloud(os.path.join(self.icp_save_path, "trans_of_1.ply"))

        for i in range(2, self.row_txt_dis + 2):  # 注意这里 +2 是因为有 N+1 帧
            trans_path = os.path.join(self.icp_save_path, f"trans_of_{i - 1}.ply")
            if os.path.exists(trans_path):
                pcd = o3d.io.read_point_cloud(trans_path)
                merged_pcd += pcd
            else:
                print(f"[警告] 找不到 {trans_path}，跳过该帧")

        # 保存最终融合点云
        merged_path = os.path.join("merged.ply")
        o3d.io.write_point_cloud(merged_path, merged_pcd)
        print("最终融合点云已保存到：", merged_path)

        # 可视化融合结果
        o3d.visualization.draw_geometries([merged_pcd], window_name="融合拼接结果", width=960, height=720)
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
