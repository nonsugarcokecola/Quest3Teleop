import h5py
# 打开 HDF5 文件（只读模式）
file_path = "/media/lzy/20241d81-13dc-4b7a-8215-8dc09a458811/home/lzy/Downloads/project/air/DISCOVERSE/data/hdf5/mmk2_plate_coffecup/episode_0.hdf5"
with h5py.File(file_path, "r") as f:
    def print_hdf5_structure(name, obj):
        """递归打印 HDF5 文件的结构"""
        if isinstance(obj, h5py.Dataset):
            print(f"Dataset: {name}, Shape: {obj.shape}, Dtype: {obj.dtype}")
        elif isinstance(obj, h5py.Group):
            print(f"Group: {name}")

    # 遍历 HDF5 文件并打印结构
    f.visititems(print_hdf5_structure)
