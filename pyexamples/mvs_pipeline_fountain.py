# Steffen Urban, 2023
import cv2, os, argparse, torch
import numpy as np
import torchvision.transforms.functional as F
import pytheia as pt

def form_mvs_input(recon_view_ids, recon, min_max_d, img_path, img_ext, device, dtype, scale=4):
    # depth = model(
    #     images,        # B x N x 3 x H x W
    #     intrinsics,    # B x N x 3 x 3
    #     extrinsics,    # B x N x 4 x 4
    #     depth_values,  # B x D (128 usually)
    #     hints,         # B x 1 x H x W (optional)
    # )
    images = []
    extrinsics = []
    intrinsics = []

    for recon_view_id in recon_view_ids:
        v = recon.View(recon_view_id)
        R_c_w = v.Camera().GetOrientationAsRotationMatrix()
        T_w_c = np.eye(4)
        T_w_c[:3, :3] = R_c_w.T
        T_w_c[:3, 3] = v.Camera().GetPosition()
        extrinsics.append(np.linalg.inv(T_w_c))

        K = v.Camera().GetCalibrationMatrix()
        K /= scale
        K[2, 2] = 1.0
        intrinsics.append(K)
        image = cv2.imread(os.path.join(img_path, v.Name() + img_ext))
        image = cv2.resize(image, (int(image.shape[1]//scale), int(image.shape[0]//scale)))
        images.append(F.to_tensor(image))

    extrinsics = np.stack(extrinsics, axis=0)
    intrinsics = np.stack(intrinsics, axis=0)

    images_t = torch.stack(images, dim=0).unsqueeze(0).to(device).to(dtype)
    extrinsics_t = torch.from_numpy(extrinsics).unsqueeze(0).to(device).to(dtype)
    intrinsics_t = torch.from_numpy(intrinsics).unsqueeze(0).to(device).to(dtype)
    depth_values_t = torch.from_numpy(np.linspace(min_max_d[0], min_max_d[1], 128)).unsqueeze(0).to(device).to(dtype)

    return {"images": images_t, "extrinsics": extrinsics_t, 
            "intrinsics": intrinsics_t, "depth_values": depth_values_t,
            "view_name": recon.View(recon_view_ids[0]).Name()}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--path_to_images', type=str,
                        default="")
    parser.add_argument('--reconstruction_path', type=str, 
                        default='')
    parser.add_argument('--img_ext', default='.png')
    args = parser.parse_args()

    # read the reconstrution
    ret, recon = pt.io.ReadReconstruction(args.reconstruction_path)

    if not ret:
        print('Failed to read the reconstruction!')
        exit()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float32

    model = torch.hub.load(
        "andreaconti/multi-view-guided-multi-view-stereo",
        "cas_mvsnet",               # mvsnet | ucsnet | d2hc_rmvsnet | patchmatchnet | cas_mvsnet
        pretrained=True,
        dataset="blended_mvg",  # blended_mvg | dtu_yao_blended_mvg
        hints="not_guided",     # mvguided_filtered | not_guided | guided | mvguided
    )
    model.to(device).to(dtype)
    model.eval()

    # perform view selection for MVSNet
    neighbors = 3
    mvs_selection = pt.mvs.ViewSelectionMVSNet(recon, neighbors, 5.0, 1.0, 10.0)

    for view_i in sorted(mvs_selection.keys()):
        # get top
        top_view_j = mvs_selection[view_i]
        
        if np.any(np.array(list(top_view_j.keys())) == 0):
            print('No top view found for view {}'.format(view_i))
            continue
        print("Neighbors for view {}: {}".format(view_i, top_view_j.values()))
        views = [view_i]
        views.extend(top_view_j.values())

        # get min max depth values
        view = recon.View(view_i)
        track_ids = view.TrackIds()
        depths = []
        for i in track_ids:
            track = recon.Track(i)
            if not track.IsEstimated():
                continue
            d, _ = view.Camera().ProjectPoint(track.Point())
            depths.append(d)
        # now get a valid depth range
        median_depth = np.median(np.array(depths))
        min_depth = np.quantile(np.array(depths), 0.05)
        min_depth -= 0.2 * (median_depth - min_depth)
        max_depth = np.quantile(np.array(depths), 0.95)
        max_depth += 0.2 * (max_depth - median_depth)

        # get image
        print("min depth: {}, max depth: {}".format(min_depth, max_depth))
        with torch.no_grad():
            mvs_input = form_mvs_input(views, recon, [min_depth, max_depth],
                                    args.path_to_images, args.img_ext, device, dtype)
            
            depth = model(mvs_input["images"], mvs_input["intrinsics"], 
                mvs_input["extrinsics"], mvs_input["depth_values"])
            
        # plot depth
        depth = depth.squeeze().cpu().numpy()     
        # 16bit depth is saved in "millimeters" although sfm recon so no real metric here
        # check your depth values to find a good scaling here
        depth = depth * 1000
        depth = depth.astype(np.uint16)
        cv2.imwrite(os.path.join(args.path_to_images,mvs_input["view_name"]+"_depth.png"),depth)

        depth = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
        depth = cv2.applyColorMap((depth*255).astype(np.uint8), cv2.COLORMAP_JET)
        cv2.imshow('depth', depth)
        cv2.waitKey(0)

