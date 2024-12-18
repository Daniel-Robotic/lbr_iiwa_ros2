import os
import cv2
import numpy as np

from typing import List, Tuple


def search_chessboard_pattern(frame: np.ndarray,
                              pattern_size: Tuple[int] = (6,9),
                              conv_size: Tuple[int] = (3,3),
                              ) -> Tuple:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    if ret:
        corners = cv2.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)

        return ret, corners
    
    return ret, None



def find_calibration_template(frame: np.ndarray, 
                              size: Tuple[int]=(6, 9),
                              conv_size: Tuple[int]=(3, 3)) -> Tuple:
    """Функция для поиска калибровочного шаблона на кадре.

    Args:
        `frame` (np.ndarray): Кадр на котором производиться поиск калибровочного шаблона.
        `size` (tuple, optional): Размер сетки калибровочного шаблока. По умолчанию `(6,9)`.

    Raises:
        TypeError: Возникает при перидаче неверного типа шаблона

    Returns:
        set[bool, np.ndarray]: Статус о найденном шаблоне и кадр с распозноным шаблоном шахматной доски
    """

    ret = False

    
    ret, corners = search_chessboard_pattern(frame=frame,
                                             pattern_size=size,
                                             conv_size=conv_size)
    
    if ret:
        frame_chessboard = cv2.drawChessboardCorners(frame, size, corners, ret)

        return ret, frame_chessboard
    
    return ret, frame


def mono_calibration(path: str, 
                     images_format_valid: Tuple[str]= ("jpg", "png"),
                     pattern_size: Tuple[int] = (6,9),
                     conv_size: Tuple[int] = (3,3)) -> Tuple:

    ret, mtx, dist = False, None, None
    msg = "Calibration error: "

    try:
        
        assert pattern_size[0] > 0, f"The first value of the {pattern_size[0]} chessboard pattern is <= 0"
        assert pattern_size[1] > 0, f"The second value of the {pattern_size[1]} chessboard pattern is <= 0"
        assert conv_size[0] > 0, f"The first value of the {conv_size[0]} convolution is <= 0"
        assert conv_size[1] > 0, f"The second value of the {conv_size[1]} convolution is <= 0"
        assert conv_size[0] == conv_size[1], f"The sizes of the convolution are not the same: {conv_size}"
        assert conv_size[0] % 2 == 1, f"The size must be odd and not equal to 0"

        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

        objpoints = []
        imgpoints = []
        img_correct = 0
        images_path = list(filter(lambda file: file.split(".")[-1] in images_format_valid, os.listdir(path)))

        for fname in images_path:
            
            img_orig = cv2.imread(os.path.join(path, fname))
            ret, corners = search_chessboard_pattern(frame=img_orig,
                                                     pattern_size=pattern_size,
                                                     conv_size=conv_size)
            if ret:
                img_correct += 1
                objpoints.append(objp)
                imgpoints.append(corners)

        height, width = img_orig.shape[:2]
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, 
                                                           imgpoints, 
                                                           (width, height),
                                                            None, None)
        
        msg = f"Processed {img_correct} images out of {len(images_path)}"

    except AssertionError as e:
        msg += str(e)
    except Exception as e:
        msg += str(e)
    finally:
        return ret, mtx, dist, msg
        
def stereo_calibration(path_image_folde1: str,
                       path_image_folde2: str,
                       cam_matrix1: np.ndarray,
                       cam_matrix2: np.ndarray,
                       cam_dist1: np.ndarray,
                       cam_dist2: np.ndarray,
                       images_format_valid: Tuple[str]= ("jpg", "png"),
                       pattern_size: Tuple[int] = (6,9),
                       conv_size: Tuple[int] = (3,3)) -> Tuple:
    
    ret = False
    msg = "Calibration error: "
    mtx1, dist1, mtx2, dist2, R, T = None, None, None, None, None, None

    try:
        assert pattern_size[0] > 0, f"The first value of the {pattern_size[0]} chessboard pattern is <= 0"
        assert pattern_size[1] > 0, f"The second value of the {pattern_size[1]} chessboard pattern is <= 0"
        assert conv_size[0] > 0, f"The first value of the {conv_size[0]} convolution is <= 0"
        assert conv_size[1] > 0, f"The second value of the {conv_size[1]} convolution is <= 0"
        assert conv_size[0] == conv_size[1], f"The sizes of the convolution are not the same: {conv_size}"
        assert conv_size[0] % 2 == 1, f"The size must be odd and not equal to 0"

        images1_path = list(filter(lambda file: file.split(".")[-1] in images_format_valid, os.listdir(path_image_folde1)))
        images2_path = list(filter(lambda file: file.split(".")[-1] in images_format_valid, os.listdir(path_image_folde2)))

        assert len(images1_path) == len(images2_path), f"The number of images in the first folder is not equal to the second one: {len(images1_path)} != {len(images2_path)}"

        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

        img_points1 = []
        img_points2 = []
        obj_points = []
        img_correct = 0

        for fname1, fname2 in zip(images1_path, images2_path):
            img1 = cv2.imread(os.path.join(path_image_folde1, fname1))
            img2 = cv2.imread(os.path.join(path_image_folde2, fname2))
            ret1, corners1 = search_chessboard_pattern(frame=img1,
                                                      pattern_size=pattern_size,
                                                      conv_size=conv_size)
            ret2, corners2 = search_chessboard_pattern(frame=img2,
                                                      pattern_size=pattern_size,
                                                      conv_size=conv_size)
            
            if ret1 and ret2:
                img_correct += 1
                obj_points.append(objp)
                img_points1.append(corners1)
                img_points2.append(corners2)

        height, width = img1.shape[:2]
        ret, mtx1, dist1, mtx2, dist2, R, T, E, F = cv2.stereoCalibrate(objectPoints=obj_points,
                                                                        imagePoints1=img_points1,
                                                                        imagePoints2=img_points2,
                                                                        cameraMatrix1=cam_matrix1,
                                                                        distCoeffs1=cam_dist1,
                                                                        cameraMatrix2=cam_matrix2,
                                                                        distCoeffs2=cam_matrix2,
                                                                        imageSize=(width, height),
                                                                        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                                                                        flags=cv2.CALIB_FIX_INTRINSIC)

        msg = f"Processed {img_correct} images out of {len(img_points1)}"
        
    except AssertionError as e:
        msg += str(e)
    except Exception as e:
        msg += str(e)
    finally:
        return ret, msg, mtx1, dist1, mtx2, dist2, R, T