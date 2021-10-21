
import pytheia as pt
from scipy.spatial.transform import Rotation as R
import numpy as np

class CameraPrior:
    def __init__(self, 
                 focal_length=500.0,
                 aspect_ratio=1.0, 
                 img_size=(1000,1000)):
        self.cam_prior = pt.sfm.CameraIntrinsicsPrior()
        self.cam_prior.focal_length.value = [focal_length]
        self.cam_prior.principal_point.value = [int(img_size[0]/2.0), int(img_size[1]/2.0)]
        self.cam_prior.aspect_ratio.value = [aspect_ratio]
        self.cam_prior.camera_intrinsics_model_type = "PINHOLE"

    def set_to_division_undistortion(self, distortion=1e-6):
        self.cam_prior.camera_intrinsics_model_type = "DIVISION_UNDISTORTION"
        self.cam_prior.radial_distortion.value = [distortion,0.0,0.0,0.0]


class RandomReconGenerator:
    def __init__(self, seed=42, 
        verbose=False, cam_prior=CameraPrior()):

        self.seed = seed
        np.random.seed(self.seed)

        self.recon = pt.sfm.Reconstruction()
        self.nr_views = 0

        self.camera = pt.sfm.Camera()
        self.camera.SetFromCameraIntrinsicsPriors(cam_prior.cam_prior)

        self.verbose = verbose

    def _sample_views(self, nr_views, 
                        xyz_min=[0,0,0], xyz_max=[2,2,2],
                        rot_ax_min=[-0.1,-0.1,-0.1], 
                        rot_ax_max=[0.1,0.1,0.1],max_rot_angle=np.pi/4):
        if self.verbose:
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
            m_cam = view.MutableCamera()
            m_cam.DeepCopy(self.camera)
            m_cam.Position = np.array([X[i],Y[i],Z[i]])
            m_cam.SetOrientationFromAngleAxis(angles[i] * np.array([RX[i], RY[i], RZ[i]]))
            view.IsEstimated = True

    def _sample_tracks(self, nr_tracks, xyz_min=[-2,-2,-2], xyz_max=[2,2,2]):
        if self.verbose:
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
                             pt3_xyz_min = [-4,-4,-1], 
                             pt3_xyz_max = [4, 4, 6],
                             cam_xyz_min = [-6, -6,-2], 
                             cam_xyz_max = [6, 6,-6],
                             cam_rot_ax_min = [-0.1,-0.1,-0.1], 
                             cam_rot_ax_max = [0.1,0.1,0.1],
                             cam_rot_max_angle = np.pi/4,
                             pixel_noise = 0.0):

        self._sample_tracks(nr_tracks, pt3_xyz_min, pt3_xyz_max)
        self._sample_views(nr_views, cam_xyz_min, cam_xyz_max, 
                           cam_rot_ax_min, cam_rot_ax_max, cam_rot_max_angle)
        self._create_observations(pixel_noise=pixel_noise)

        return self.recon

    def _create_observations(self, pixel_noise = 0.0):
        for tid in self.recon.TrackIds:
            track = self.recon.Track(tid).Point
            for vid in self.recon.ViewIds:
                view = self.recon.View(vid)
                cam = view.Camera()
                obs = cam.ProjectPoint(track)
                if obs[0] <= 0:
                    continue
                point2d = obs[1] + np.random.randn(2) * pixel_noise
                if self.verbose:
                    print("Adding observation: track {} in view {} projection {}".format(tid, vid, point2d))
                self.recon.AddObservation(vid, tid, pt.sfm.Feature(point2d))

    def add_view(self, view_pos, view_ax_angle, view_name=""):
        num_views = len(self.recon.ViewIds)
        view_id = self.recon.AddView(view_name, 0, num_views+1)
        if self.verbose:
            print("Adding view {}".format(view_id))
        view = self.recon.View(view_id)
        view.MutableCamera().Position = np.array(view_pos)
        view.MutableCamera().SetOrientationFromAngleAxis(view_ax_angle)
        view.IsEstimated = True

    def add_track(self, track_xyz):
        track_id = self.recon.AddTrack()
        if self.verbose:
            print("Adding track {}".format(track_id))
        track = self.recon.MutableTrack(track_id)
        track.Point = np.array([track_xyz[0],track_xyz[1],track_xyz[2],1],dtype=np.float32)
        track.IsEstimated = True

    def add_noise_to_view(self, view_id, noise_pos, noise_angle):
        view = self.recon.View(view_id)
        view.MutableCamera().Position = view.MutableCamera().Position + noise_pos*np.random.randn(3)
        ax_angle = view.Camera().GetOrientationAsAngleAxis()
        noise_angle_rad = noise_angle * np.pi / 180.
        view.MutableCamera().SetOrientationFromAngleAxis(ax_angle + noise_angle_rad*np.random.randn(3))

    def add_noise_to_views(self, noise_pos=1e-5, noise_angle=1e-2):
        for view_id in self.recon.ViewIds:
            self.add_noise_to_view(view_id, noise_pos, noise_angle)



if __name__ == "__main__":
    gen = RandomReconGenerator(seed=42, verbose=True)

    gen.generate_random_recon()

    for i in range(10):
        gen.add_track([i*i,i,i+i])

    for i in range(10):
        gen.add_view(view_pos=[0,i,0], view_ax_angle=[i,0,0], view_name="ii"+str(i))