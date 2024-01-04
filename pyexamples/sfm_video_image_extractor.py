# Steffen Urban, June 2023
import cv2, os
import numpy as np

def convert_image(img, size):
    if len(img.shape) > 2:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.resize(img, size)

def extract_features(img):
    pts = cv2.goodFeaturesToTrack(img, 500, qualityLevel=0.05, minDistance=10)
    return pts, len(pts)

def save_image(out_path, img, idx, img_ext):
    cv2.imwrite(os.path.join(out_path,"image_"+str(idx)+"."+img_ext), img)

def get_retrack_dist(p, p_back):
    dx = (p[:,0,0] - p_back[:,0,0])**2
    dy = (p[:,0,1] - p_back[:,0,1])**2
    dist = np.sqrt(dx+dy)
    return dist


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Argument parser for image extractor')
    parser.add_argument('--path_to_video', default="", type=str, required=True)
    parser.add_argument('--path_to_image_output', default="", type=str, required=True)
    parser.add_argument('--img_ext', default='jpg')
    parser.add_argument('--debug', default=0)
    parser.add_argument('--percent_lost', 
                        help="we will insert a new frame after a certain percent of points are lost after LK tracking",
                        type=int, default=10) # for video sequences, if 0 no temporal window
    parser.add_argument('--min_framerate_to_extract', type=int, default=10, help="min framerate to extract images ( eg. 10 means at minimum every 10th frame is extracted)")
    parser.add_argument('--retrack_err_dist', type=float, default=10.)
    args = parser.parse_args()

    img_size_track = (640,480)
    
    vid = cv2.VideoCapture(args.path_to_video)
    print("Video has {} frames and a framerate of {}".format(
        int(vid.get(cv2.CAP_PROP_FRAME_COUNT)), int(vid.get(cv2.CAP_PROP_FPS))))
    lk_params = dict(winSize=(11,11), maxLevel=3)
    debug = True

    if not os.path.exists(args.path_to_image_output):
        os.makedirs(args.path_to_image_output)

    # get first image
    ret, prev_img = vid.read()
    prev_img = convert_image(prev_img, img_size_track)
    save_image(args.path_to_image_output, prev_img, 0, args.img_ext)
    p0, num_track_pts = extract_features(prev_img)

    # start main tracking loop
    line_image = cv2.cvtColor(np.zeros_like(prev_img), cv2.COLOR_GRAY2BGR)
    img_idx, no_gp_img, prev_img_idx, total_img_id = 0, 0, 0, 0
    while True:
        ret, img = vid.read()

        if no_gp_img > 200:
            break
        if not ret:
            no_gp_img += 1
            continue
    
        
        img_gray = convert_image(img, img_size_track)

        # track re-track to check bad tracks
        p1, st, err = cv2.calcOpticalFlowPyrLK(prev_img, img_gray, p0, None, **lk_params)
        p0_back, st_, err_ = cv2.calcOpticalFlowPyrLK(img_gray, prev_img, p1, None, **lk_params)

        # check distance after re-track, if too large set outlier
        dist = get_retrack_dist(p0, p0_back)
        st[dist > args.retrack_err_dist] = 0

        # select points
        good_new = p1[st==1]
        good_old = p0[st==1]

        if args.debug:    
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
        print("total_img_id - prev_img_idx: {}".format(total_img_id - prev_img_idx))
        if np.sum(st) < (1.0 - args.percent_lost/100)*num_track_pts or total_img_id - prev_img_idx > args.min_framerate_to_extract:
            save_image(args.path_to_image_output, img, img_idx, args.img_ext)
            p0, num_track_pts = extract_features(prev_img)   
            prev_img = img_gray.copy()     
            line_image = np.zeros_like(line_image)
            print("Saving image: {}.".format(img_idx))
            prev_img_idx = total_img_id
            img_idx += 1
        else:
            prev_img = img_gray.copy()
            p0 = good_new.reshape(-1,1,2)
        
        total_img_id += 1