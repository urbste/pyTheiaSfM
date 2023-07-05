import open3d as o3d
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--path_to_images', type=str,
                        default="")
    parser.add_argument('--reconstruction_path', type=str, 
                        default='')
    parser.add_argument('--img_ext', default='.png')
    args = parser.parse_args()
