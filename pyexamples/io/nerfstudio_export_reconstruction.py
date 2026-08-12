import pytheia as pt
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--path_to_images', type=str,
                        default="")
    parser.add_argument('--path_to_recon', type=str, 
                        default='')
    parser.add_argument('--path_out_nerfstudio_json', type=str, 
                        default='')
    parser.add_argument('--aabb_scale', type=int, default=16)
    args = parser.parse_args()

    res, recon = pt.io.ReadReconstruction(args.path_to_recon)
    if not res:
        print("Error could not read reconstruction {}.".format(args.path_to_recon))

    res = pt.io.WriteNerfStudio(args.path_to_images, recon, 
                          args.aabb_scale, args.path_out_nerfstudio_json)
    
    if not res:
        print("Was not able to write nerfstudio files.")
