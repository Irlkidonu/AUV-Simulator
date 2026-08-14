from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from paper1_sim_extension.fiducial_geometry import (camera_matrix, detector_image,
                                                     marker_world_corners,
                                                     solve_mapped_fix)


def test_detector_image_encoding_conversion():
    radiance=np.arange(100,dtype=np.float32).reshape(10,10)
    converted=detector_image(radiance,"32FC1")
    assert converted.dtype==np.uint8 and converted.shape==(10,10)
    assert converted[0,0]==0 and converted[-1,-1]==255
    rgb=np.zeros((4,5,3),np.uint8)
    assert detector_image(rgb,"bgr8") is rgb


def test_two_mapped_markers_produce_image_derived_fix():
    image = np.full((480, 640), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_map = {}
    for marker_id, x0 in ((0, 170), (1, 350)):
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 120)
        image[180:300, x0:x0+120] = marker
        # Build a consistent planar map by ray-plane backprojection at z=5 in
        # the camera frame. This exercises detection + multi-marker PnP.
        k = camera_matrix(640, 480, 1.047)
        pixels = np.array([[x0,180],[x0+120,180],[x0+120,300],[x0,300]], float)
        rays = np.c_[pixels, np.ones(4)] @ np.linalg.inv(k).T
        marker_map[marker_id] = rays * (5.0 / rays[:, 2:3])
    fix = solve_mapped_fix(image, marker_map, camera_matrix(640,480,1.047), 0.9)
    assert fix is not None
    assert fix.marker_count == 2 and fix.inlier_count >= 8
    assert np.linalg.norm(fix.position_world_m) < 0.1
    assert np.all(np.linalg.eigvalsh(fix.covariance_m2) > 0)


def test_single_marker_is_not_a_valid_mapped_fix():
    image = np.full((480,640), 255, np.uint8)
    marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 120)
    image[180:300,260:380] = marker
    marker_map = {0: marker_world_corners((0,0,0), 0.9)}
    assert solve_mapped_fix(image, marker_map, camera_matrix(640,480,1.047), 1.0) is None


def test_logical_marker_source_decodes_exact_dictionary_id():
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters=cv2.aruco.DetectorParameters_create()
    for marker_id in range(8):
        logical=cv2.aruco.drawMarker(dictionary,marker_id,6,borderBits=1)
        core=cv2.resize(logical,(240,240),interpolation=cv2.INTER_NEAREST)
        rendered=np.full((320,320),255,np.uint8); rendered[40:280,40:280]=core
        ids=cv2.aruco.detectMarkers(rendered,dictionary,parameters=parameters)[1]
        assert ids is not None and ids.ravel().tolist()==[marker_id]
