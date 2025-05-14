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
        savehe icp这两个文件夹也可以一起，分开主要是为了保存清晰
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
    点云拆分分为几步
    1、仅仅去除内页板混凝土的BIM模型，存储为OBJ模型，并离散化为点云，从云端获取
    2、将仅去除内页板混凝土的BIM模型点云与多站点扫描合并后的点云文件匹配，此处需使用点云配准函数将BIM模型点云与扫描点云匹配并保存旋转平移矩阵
    3、待检测预埋件IFC模型（仅包括待检测预埋件的模型、且包含语义信息）转为OBJ文件，在转为点云，基于上一步计算出来的旋转平移矩阵进行变换
    4、基于变换结果和最近邻算法提取对应预埋件点云，完成预埋件点云的拆分

    Args:
        merged_points_file (str): 多站点扫描合并后的点云文件
        components (list[Component]):
        1、仅仅去除内页板混凝土的BIM模型，存储为OBJ模型
        2、待检测预埋件IFC模型（仅包括待检测预埋件的模型、且包含语义信息）转为OBJ文件
        out_path (str): 拆分结果输出目录

    Returns:
        list[Component]: 将拆分结果整理返回,此时points_file 存放输出的点云文件路径（为各种待检测的拆分预埋件点云数据xx.txt）
    """
    ...


class SuperParameters(TypedDict):
    """
    1、计算模具长宽及两对角线长度，及对角线差
    2、计算门窗长宽、对角线长度和对角线差
    3、预埋钢板水平位置（X/Y）及垂直位置（Z）
    4、插筋锚固长度，水平位置（X）及垂直位置（Z）
    5、波纹管长度，水平位置（X）及垂直位置（Z）
    6、吊钉垂直位置（Z）
    7、线盒水平位置(X/Y)

    水平位置主要是计算四个边，取最近的两条边作为（X/Y）位置
    垂直位置（Z）计算到保温板平面的距离
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
    质量检测分以下几步
    1、计算模具长宽及两对角线长度，及对角线差
    2、计算门窗长宽、对角线长度和对角线差

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
        points_cloud (str): 从云端获取的点云文件或者构件OBJ格式文件，obj文件需现场离散为点云文件
        points_result (str): 差异化计算得到的新点云文件路径,结果写入该文件，或者存为png格式图片显示

    Returns:
        bool: 执行结果,true 表示正确执行完毕,false 表示执行失败
    """
    pass
