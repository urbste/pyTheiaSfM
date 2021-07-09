
import pytheia as pt
from scipy.spatial.transform import Rotation as R
import numpy as np

class RandomReconGenerator:
    def __init__(self, seed=42):

        self.seed = seed
        np.random.seed(self.seed)

        self.recon = pt.sfm.Reconstruction()
        self.nr_views = 0

    def _sample_views(self, nr_views, 
                        xyz_min=[0,0,0], xyz_max=[2,2,2],
                        rot_ax_min=[-0.1,-0.1,-0.1], 
                        rot_ax_max=[0.1,0.1,0.1],max_rot_angle=np.pi/4):
        print("Sampling {} views".format(nr_views))

        self.nr_cams = nr_views

        X = np.random.uniform(low=xyz_min[0], high=xyz_max[0], size=(nr_views,))
        Y = np.random.uniform(low=xyz_min[1], high=xyz_max[1], size=(nr_views,))
        Z = np.random.uniform(low=xyz_min[2], high=xyz_max[2], size=(nr_views,)) 
        RX = np.random.uniform(low=rot_ax_min[0], high=rot_ax_max[0], size=(nr_views,)) 
        RY = np.random.uniform(low=rot_ax_min[1], high=rot_ax_max[1], size=(nr_views,)) 
        RZ = np.random.uniform(low=rot_ax_min[2], high=rot_ax_max[2], size=(nr_views,)) 
        angles = np.random.uniform(low=-max_rot_angle, high=max_rot_angle, size=(nr_views,)) 
        for i in range(self.nr_cams):
            view_id = self.recon.AddView(str(i),0,i)
            view = self.recon.View(view_id)
            view.MutableCamera().Position = np.array([X[i],Y[i],Z[i]])
            view.MutableCamera().SetOrientationFromAngleAxis(angles[i] * np.array([RX[i], RY[i], RZ[i]]))
            view.IsEstimated = True

    def _sample_tracks(self, nr_tracks, xyz_min=[-2,-2,-2], xyz_max=[2,2,2]):
        print("Sampling {} tracks".format(nr_tracks))
        self.nr_tracks = nr_tracks

        X = np.random.uniform(low=xyz_min[0], high=xyz_max[0], size=(nr_tracks,))
        Y = np.random.uniform(low=xyz_min[1], high=xyz_max[1], size=(nr_tracks,))
        Z = np.random.uniform(low=xyz_min[2], high=xyz_max[2], size=(nr_tracks,)) 
        for i in range(self.nr_tracks):
            track_id = self.recon.AddTrack()
            point = np.array([X[i],Y[i],Z[i],1],dtype=np.float32)
            track = self.recon.MutableTrack(track_id)
            track.Point = point
            track.IsEstimated = True

    #def _project_points_to_views(self):


    def generate_random_recon(self, 
                            nr_views = 10, 
                            nr_tracks = 100, 
                             pt3_xyz_min = [-2,-2,-2], 
                             pt3_xyz_max = [2,2,2],
                             cam_xyz_min = [0,0,0], 
                             cam_xyz_max = [2,2,2],
                             cam_rot_ax_min = [-0.1,-0.1,-0.1], 
                             cam_rot_ax_max = [0.1,0.1,0.1],
                             cam_rot_max_angle = np.pi/4):

        self._sample_tracks(nr_tracks, pt3_xyz_min, pt3_xyz_max)
        self._sample_views(nr_views, cam_xyz_min, cam_xyz_max, 
                           cam_rot_ax_min, cam_rot_ax_max, cam_rot_max_angle)

        return self.recon

    def add_view(self, view_pos, view_ax_angle, view_name=""):
        num_views = len(self.recon.ViewIds)
        view_id = self.recon.AddView(view_name, 0, num_views+1)

        print("Adding view {}".format(view_id))
        view = self.recon.View(view_id)
        view.MutableCamera().Position = np.array(view_pos)
        view.MutableCamera().SetOrientationFromAngleAxis(view_ax_angle)
        view.IsEstimated = True

    def add_track(self, track_xyz):
        track_id = self.recon.AddTrack()
        print("Adding track {}".format(track_id))

        point = np.array([track_xyz[0],track_xyz[1],track_xyz[2],1],dtype=np.float32)
        track = self.recon.MutableTrack(track_id)
        track.Point = point
        track.IsEstimated = True

if __name__ == "__main__":
    gen = RandomReconGenerator()

    gen.generate_random_recon()

    for i in range(10):
        gen.add_track([i*i,i,i+i])

    for i in range(10):
        gen.add_view(view_pos=[0,i,0], view_ax_angle=[i,0,0], view_name="ii"+str(i))