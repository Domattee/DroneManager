""" Utilities for engdel image processing/checking """

import json
import pathlib
from argparse import ArgumentParser
import cv2
import numpy as np
from skimage.metrics import structural_similarity

from dronecontrol.missions.engel import ENGELCaptureInfo

# TODO: Currently only written for lowres "visible" images


def load_images(json_file: str):
    with open(json_file, "rt") as f:
        json_dict = json.load(f)
    captures = [ENGELCaptureInfo.from_json_dict(capture_dict) for capture_dict in json_dict]
    replay_imgs = []
    og_imgs = []
    base_img_path = pathlib.Path(captures[0].images[0].file_location).parent.parent
    for capture_info in captures:
        if capture_info.reference_id is None:
            print("Not a replay capture!")
            return False
        for image in capture_info.images:
            if "visible" in image.file_location:
                replay_imgs.append(cv2.imread(image.file_location))
                # Go through the images in the reference capture and find the "visible" one
                reference_capture_image_list = [f for f in base_img_path.joinpath(str(capture_info.reference_id)).iterdir() if f.is_file()]
                og_image = [str(f) for f in reference_capture_image_list if "visible" in str(f)][0]
                og_imgs.append(cv2.imread(og_image))
    return list(zip(og_imgs, replay_imgs))


def get_difference_image(img1, img2):
    diff_rgb = cv2.absdiff(img1, img2)
    return cv2.cvtColor(diff_rgb, cv2.COLOR_RGB2GRAY)


# TODO: basic homography correction
def get_overlap(img1, img2, ratio=0.75):
    h1, w1, c1 = img1.shape
    h2, w2, c2 = img2.shape

    # TODO: rectify images

    # Get Keypoints
    orb = cv2.SIFT.create()
    img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)
    img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2BGR)
    p1, desc1 = orb.detectAndCompute(img1, None)
    p2, desc2 = orb.detectAndCompute(img2, None)

    # Get Matches between Keypoints
    index_params = {"algorithm": 1, "trees": 5}
    search_params = {"checks": 50, }
    k = 2
    matcher = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)
    matches = matcher.knnMatch(desc1, desc2, k)

    filtered_matches = []

    for x, y in matches:
        if x.distance < y.distance * ratio:
            filtered_matches.append(x)

    p1_array = np.float32([p.pt for p in p1])
    p2_array = np.float32([p.pt for p in p2])

    matched_p1 = np.float32([p1_array[m.queryIdx] for m in filtered_matches])
    matched_p2 = np.float32([p2_array[m.trainIdx] for m in filtered_matches])

    H_matrix, status = cv2.findHomography(matched_p1, matched_p2, cv2.RANSAC, 4)
    print(H_matrix)

    # Maximum size of target image, note cv swaps width and height compared to numpy
    raw_size = (w1 + w2, h1 + h2)

    # Warp the second image
    warped_img2 = cv2.warpPerspective(img2, H_matrix, raw_size)
    cv2.imshow("dummy", warped_img2)
    cv2.waitKey(0)
    compound_img = warped_img2.copy()
    compound_img[0:h1, 0:w1] = img1

    cv2.imshow("dummy", compound_img)
    cv2.waitKey(0)


CHARUCO_SIZE = (7, 11)  # Square horizontally and vertically. Should be different since orientation matters
CHARUCO_SQUARE_SIZE = 0.023  # meters, measure after printing
CHARUCO_MARKER_SIZE = 0.0115  # meters, measure after printing
CHARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)  # Note that the last number (250 here) must be more than the total number of squares (7*11 = 77 by default)

def main():
    parser = ArgumentParser()
    command_parsers = parser.add_subparsers(title="command", description="Command to execute.", dest="command")

    # Homography tests
    homography_parser = command_parsers.add_parser("hom", help="Do a homogrophy over a capture series")
    homography_parser.add_argument("path", type=str)

    # Make charuco board
    board_parser = command_parsers.add_parser("board", help="Create a charuco board for calibration")

    # Do calibration with series of images of charuco
    calib_parser = command_parsers.add_parser("calib", help="Determine camera calibration")
    calib_parser.add_argument("path", type=str)

    args = parser.parse_args()

    if args.command == "hom":
        # Do the image correction
        imgs = load_images(args.path)
        clear_window_id = "Single Image"
        diff_window_id = "Difference Image"
        for pair in imgs:
            cv2.imshow(clear_window_id, pair[0])
            diff = get_difference_image(*pair)
            cv2.imshow(diff_window_id, diff)
            ssim, diff_img_ssim = structural_similarity(*pair, channel_axis=2, full=True)
            cv2.setWindowTitle(diff_window_id, f"PSNR: {cv2.PSNR(*pair)}, SSIM: {ssim}")
            cv2.waitKey(0)
            get_overlap(*pair)
    elif args.command == "board":
        # Do camera calibration
        print("Making board...")
        img_height = 1080  # pixels
        img_width = int(img_height * CHARUCO_SIZE[0] / CHARUCO_SIZE[1])
        margin_size = 40  # pixels
        board = cv2.aruco.CharucoBoard(CHARUCO_SIZE, CHARUCO_SQUARE_SIZE, CHARUCO_MARKER_SIZE, CHARUCO_DICTIONARY)
        board_img = board.generateImage((img_width, img_height), marginSize=margin_size)
        img_file = pathlib.Path.cwd().joinpath("charcu_board.png")
        cv2.imwrite(str(img_file), board_img)
        print(f"Saved board png to {img_file}")
    elif args.command == "calib":

        img_folder = args.path
        image_files = [str(f) for f in pathlib.Path(img_folder).iterdir() if f.is_file() and str(f).endswith(".png")]

        board = cv2.aruco.CharucoBoard(CHARUCO_SIZE, CHARUCO_SQUARE_SIZE, CHARUCO_MARKER_SIZE, CHARUCO_DICTIONARY)
        charuco_detector = cv2.aruco.CharucoDetector(board=board)

        all_images = []
        all_corners = []
        all_corner_ids = []
        all_marker_corners = []
        all_marker_ids = []
        all_obj_points = []
        all_img_points = []
        image_size = None
        for image_file in image_files:
            image = cv2.imread(image_file)

            if image_size is None:
                image_size = image.shape[:2]
            else:
                if image.shape[:2] != image_size:
                    print(f"All images but have the same size, but {image_file} differs from previous.")
                    return False

            all_images.append(image)

            # Detect corners und markers
            cur_char_corners, cur_char_ids, cur_marker_corners, cur_marker_ids = charuco_detector.detectBoard(image)
            detected_img = cv2.aruco.drawDetectedMarkers(image.copy(), cur_marker_corners, cur_marker_ids)
            detected_img = cv2.aruco.drawDetectedCornersCharuco(detected_img, cur_char_corners, cur_char_ids)
            cv2.namedWindow("detections", cv2.WINDOW_NORMAL)
            cv2.imshow("detections", detected_img)
            cv2.waitKey(0)
            obj_points, img_points = board.matchImagePoints(cur_char_corners, cur_char_ids)

            all_corners.append(cur_char_corners)
            all_corner_ids.append(cur_char_ids)
            all_marker_corners.append(cur_marker_corners)
            all_marker_ids.append(cur_marker_ids)
            all_obj_points.append(obj_points)
            all_img_points.append(img_points)
            print(f"Detected {obj_points.shape} object points and {img_points.shape} image points")

        ret, cam_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(all_obj_points, all_img_points, image_size, None, None)
        print(ret)
        print(cam_matrix, distortion)

        for image in all_images:
            undistorted = cv2.undistort(image, cam_matrix, distortion)
            cv2.namedWindow("Undistorted", cv2.WINDOW_NORMAL)
            cv2.imshow("Undistorted", undistorted)
            cv2.waitKey(0)


if __name__ == "__main__":
    main()
