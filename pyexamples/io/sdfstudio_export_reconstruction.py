import pytheia as pt
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--path_to_images', type=str,
                        default="")
    parser.add_argument('--path_to_recon', type=str, 
                        default='')
    args = parser.parse_args()

    res, recon = pt.io.ReadReconstruction(args.path_to_recon)
    if not res:
        print("Error could not read reconstruction {}.".format(args.path_to_recon))

    res = pt.io.WriteSdfStudio(args.path_to_images, recon, [2, 6], 1.0)
    
    if not res:
        print("Was not able to write nerfstudio files.")
