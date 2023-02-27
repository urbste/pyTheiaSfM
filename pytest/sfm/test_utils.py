import numpy as np


def Reprojection(camera):
    kTolerance = 1e-5
    kNormalizedTolerance = kTolerance / camera.FocalLength()
    kImageWidth = 1200
    kImageHeight = 980
    kMinDepth = 2
    kMaxDepth = 25

    # Ensure the image -> camera -> image transformation works.
    for x in range(0, kImageWidth, 10):
        for y in range(0, kImageHeight, 10):
            pixel = np.array([x, y])
            # Get the normalized ray of that pixel.
            normalized_ray = camera.ImageToCameraCoordinates(pixel)

            # Test the reprojection at several depths.
            for depth in range(kMinDepth, kMaxDepth):
                # Convert it to a full 3D point in the camera coordinate system.
                point = normalized_ray * depth
                reprojected_pixel = camera.CameraToImageCoordinates(point)

                # Expect the reprojection to be close.
                assert np.linalg.norm(pixel - reprojected_pixel) < kTolerance
                #print( "gt pixel: " + str(pixel.T))
                #print("reprojected pixel: " + str(reprojected_pixel.T))

    # Ensure the camera -> image -> camera transformation works.
    ls = [-0.8 + 0.1 * x for x in range(16)]
    for x in ls:
        for y in ls:
            for depth in range(kMinDepth, kMaxDepth):
                point = np.array([x, y, depth])
                pixel = camera.CameraToImageCoordinates(point)

                # Get the normalized ray of that pixel.
                normalized_ray = camera.ImageToCameraCoordinates(pixel)

                # Convert it to a full 3D point in the camera coordinate system.
                reprojected_point = normalized_ray * depth

                # Expect the reprojection to be close.
                assert np.linalg.norm(
                    point - reprojected_point) < kNormalizedTolerance
                # print( "gt pixel: " + str(point.T))
                # print("reprojected pixel: " + str(reprojected_point.T))


def ReprojectionOrthographic(camera):
    kTolerance = 1e-5
    kNormalizedTolerance = kTolerance / camera.FocalLength()
    kImageWidth = 2560
    kImageHeight = 1920

    # Ensure the image -> camera -> image transformation works.
    for x in range(0, kImageWidth, 20):
        for y in range(0, kImageHeight, 20):
            pixel = np.array([x, y])
            # Get the normalized ray of that pixel.
            point = camera.ImageToCameraCoordinates(pixel)

            # Convert it to a full 3D point in the camera coordinate system.
            reprojected_pixel = camera.CameraToImageCoordinates(point)

            # Expect the reprojection to be close.
            assert np.linalg.norm(pixel - reprojected_pixel) < kTolerance
            #print( "gt pixel: " + str(pixel.T))
            #print("reprojected pixel: " + str(reprojected_pixel.T))

    # Ensure the camera -> image -> camera transformation works.
    for x in np.arange(-0.01, 0.01, 0.001):
        for y in np.arange(-0.01, 0.01, 0.001):
            point = np.array([x, y, 0.0])
            pixel = camera.CameraToImageCoordinates(point)

            # Get the normalized ray of that pixel.
            reprojected_point = camera.ImageToCameraCoordinates(pixel)

            # Expect the reprojection to be close.
            assert np.linalg.norm(
                point[:2] - reprojected_point[:2]) < kNormalizedTolerance
            # print( "gt pixel: " + str(point.T))
            # print("reprojected pixel: " + str(reprojected_point.T))

