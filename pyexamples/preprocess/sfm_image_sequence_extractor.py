# Steffen Urban, June 2023
# This file can be used to select good images from a video sequence
# given as a list of files.
import cv2, os, glob, natsort
import numpy as np
import copy

def convert_image(img, size):
    if len(img.shape) > 2:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.resize(img, size)

def extract_features(img):
    pts = cv2.goodFeaturesToTrack(img, 200, qualityLevel=0.02, minDistance=10)
    return pts, len(pts)

def save_image(out_path, img, idx, img_ext):
    cv2.imwrite(os.path.join(out_path,"image_"+str(idx)+"."+img_ext), img)

def get_retrack_dist(p, p_back):
    dx = (p[:,0,0] - p_back[:,0,0])**2
    dy = (p[:,0,1] - p_back[:,0,1])**2
    dist = np.sqrt(dx+dy)
    return dist

def get_flow_magnitude(old, new):
    dx = (old[:,0] - new[:,0])**2
    dy = (old[:,1] - new[:,1])**2
    dist = np.median(np.sqrt(dx+dy))
    return dist

def read_image(img_path, img_name, ext):
    return cv2.imread(os.path.join(img_path, img_name+"."+ext))

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Argument parser for image extractor')
    parser.add_argument('--path_to_image_sequence', 
                        default="/home/steffen/Data/UnderwaterData/aqualoc/Harbor/harbor_sequence_02_raw_data/raw_data/harbor_images_sequence_02/", 
                        type=str)
    parser.add_argument('--path_to_image_output', 
                        default="/home/steffen/Data/UnderwaterData/aqualoc/Harbor/harbor_sequence_02_raw_data/raw_data/harbor_images_sequence_02_selected/", 
                        type=str)
    parser.add_argument('--img_ext', default='png')
    parser.add_argument('--debug', default=1)
    parser.add_argument('--percent_lost', 
                        help="we will insert a new frame after a certain percent of points are lost after LK tracking",
                        type=int, default=10) # for video sequences, if 0 no temporal window
    parser.add_argument('--retrack_err_dist', type=float, default=1.)
    parser.add_argument('--opt_flow_magn_insert_frame', type=float, default=10)
    args = parser.parse_args()

    img_size_track = (640, 512)
    
    # opencv extraction of features from images
    images_files = glob.glob(os.path.join(args.path_to_image_sequence,'*.'+args.img_ext))
    image_names = natsort.natsorted([os.path.splitext(os.path.split(f)[1])[0] for f in images_files])

    lk_params = dict(winSize=(11,11), maxLevel=3)
    debug = True

    if not os.path.exists(args.path_to_image_output):
        os.makedirs(args.path_to_image_output)

    # get first image
    prev_img = read_image(args.path_to_image_sequence, image_names[0], args.img_ext)
    prev_img = convert_image(prev_img, img_size_track)
    save_image(args.path_to_image_output, prev_img, 0, args.img_ext)
    p0, num_track_pts = extract_features(prev_img)
    p0_keyframe = p0

    # start main tracking loop
    line_image = cv2.cvtColor(np.zeros_like(prev_img), cv2.COLOR_GRAY2BGR)
    img_idx, no_gp_img = 0, 0
    for img_name in image_names[1:]:
        img = read_image(args.path_to_image_sequence, img_name, args.img_ext)        
        img_gray = convert_image(img, img_size_track)
        
        # track re-track to check bad tracks
        p1, st, err = cv2.calcOpticalFlowPyrLK(prev_img, img_gray, p0, None, **lk_params)
        p0_back, st_, err_ = cv2.calcOpticalFlowPyrLK(img_gray, prev_img, p1, None, **lk_params)

        # check distance after re-track, if too large set outlier
        dist = get_retrack_dist(p0, p0_back)
        st[dist > args.retrack_err_dist] = 0

        magn = get_flow_magnitude(p0_keyframe[st==1], p1[st==1])

        if args.debug:            
            good_new = p1[st==1]
            good_old = p0[st==1]
            img_small = cv2.resize(img, img_size_track)
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.astype(np.int32).ravel()
                c, d = old.astype(np.int32).ravel()
                mask = cv2.line(line_image, (a,b), (c,d), (0, 255, 0), 2)
                img_small = cv2.circle(img_small, (a,b), 5, (255,0,0), -1)
            plot_img = cv2.add(img_small, line_image)
            cv2.imshow("track", plot_img)
            cv2.waitKey(1)

        # check if too many points are lost after tracking --> save image
        if np.sum(st) < (1.0 - args.percent_lost/100)*num_track_pts or magn > args.opt_flow_magn_insert_frame:
            save_image(args.path_to_image_output, img, img_idx, args.img_ext)           
            p0, num_track_pts = extract_features(prev_img)  
            p0_keyframe = p0

            prev_img = img_gray.copy()  
            line_image = np.zeros_like(line_image)
            print("Saving image: {}.".format(img_idx))
            img_idx += 1
        else:
            prev_img = img_gray.copy()   
            p0 = p1
        