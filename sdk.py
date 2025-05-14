"""

质量检测, 接口约定

"""

from typing import TypedDict


def merge_points_files(files_paths: list[str], result_path: str) -> bool:
    """
    合并多个点云文件成为一个点云文件,要求：扫描点云的路径和输出结果的路径均
    由形参传入

    Args:
        files_paths (list[str])"./转角.txt",
        files_paths (list[str])"./平移量.txt",这两个数据可能是固定的，所以也可能直接写在底层函数上
        files_paths (list[str]):  传入多个点云文件的路径,如 ['1.txt','2.txt']
        save_folder_path (list[str]):传入粗配准点云的路径,如 ['coarse_1.txt','coarse_2.txt']
        icp_save_path (list[str]):传入精确配准点云的路径,如 ['accuracy_1.txt','accuracy_2.txt']
        savehe icp这两个文件夹也可以一起，分开主要是为了清晰保存
    Returns:
        bool: 执行结果, true 表示正确执行完毕,false 表示失败.
        xx.ply or xx.txt 整体点云数据

    ps:可能每次扫描的数据都需要一个单独的文件夹，
    """
    ...


class Component(TypedDict):
    # 构件ID/UID/编码,类型待定,寓意待定
    uid: str | int
    # 点云存放路径
    points_file: str


def split_points(
    merged_points_file: str, components: list[Component], out_path: str
) -> list[Component]:
    """
    点云拆分算法API.

    Args:
        merged_points_file (str): 多站点扫描合并后的点云文件
        components (list[Component]): 从云端拉取得到的构件信息
        out_path (str): 拆分结果输出目录

    Returns:
        list[Component]: 将拆分结果整理返回,此时points_file 存放输出的点云文件路径
    """
    ...


class SuperParameters(TypedDict):
    """
    定义检测指标
    """

    ...


class QualityResult(TypedDict):
    """
    定义检测结果
    """

    ...


def make_quality(
    points_splited: str,
    points_cloud: str,
    super_para: SuperParameters,
) -> QualityResult:
    """
    质量检测API


    Args:
        points_splited (str): 分割后的构件A 的点云文件路径
        points_cloud (str): 从云端获取的构件A 的点云文件路径
        super_para (SuperParameters): 从云端获取的检测项B,在检测期间需要提供的超参,由算法提要求

    Returns:
        QualityResult: 与检测项B 对应的, 检测结果. 返回结果内容应至少包含UI中需要展示的检测项.
    """
    pass


def diff_points(
    points_splited: str, points_cloud: str, points_result: str
) -> bool:
    """
    差异化点云API.生成点云路径由形参传入

    Args:
        points_splited (str): 分割后的点云文件
        points_cloud (str): 从云端获取的原始点云
        points_result (str): 差异化计算得到的新点云文件路径,结果写入该文件

    Returns:
        bool: 执行结果,true 表示正确执行完毕,false 表示执行失败
    """
    pass
