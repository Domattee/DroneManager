""" Utilities for engdel image processing/checking """

import json
import pathlib
from argparse import ArgumentParser
import cv2
import numpy as np
from skimage.metrics import structural_similarity

import largestinteriorrectangle as lir

from dronemanager.missions.engel import ENGELCaptureInfo

# TODO: Currently only written for lowres "visible" images

INTRINSIC_IPHONE = np.asarray(
    [[3119.486, 0, 2055.170],
     [0.0, 3110.758, 1489.212],
     [0.0, 0.0, 1.0]])
DISTORTION_IPHONE = np.asarray([[0.1919431, -0.9801058, -0.0031962, 0.0017257, 1.261248]])


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


def get_overlap(img1, img2, ratio=0.75):
    h1, w1, c1 = img1.shape
    h2, w2, c2 = img2.shape

    # Get Keypoints
    orb = cv2.SIFT.create()
    img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    p1, desc1 = orb.detectAndCompute(img1_gray, None)
    p2, desc2 = orb.detectAndCompute(img2_gray, None)

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

    H_matrix, status = cv2.findHomography(matched_p2, matched_p1, cv2.RANSAC, 2)

    # Determine the corner points of the second image, to prevent them from lying outside
    corners_before = np.array([[0,0], [w2, 0], [w2, h2], [0, h2]], dtype=np.float32).reshape(-1, 1, 2)
    corners_after = cv2.perspectiveTransform(corners_before, H_matrix)
    h_min_warped = int(np.floor(corners_after[:, :, 1].min()))
    h_max_warped = int(np.ceil(corners_after[:, :, 1].max()))
    w_min_warped = int(np.floor(corners_after[:, :, 0].min()))
    w_max_warped = int(np.floor(corners_after[:, :, 0].max()))
    h_min = h_min_warped
    h_max = h_max_warped
    w_min = w_min_warped
    w_max = w_max_warped
    if h_min_warped >= 0:
        h_min = 0
    if w_min_warped >= 0:
        w_min = 0
    if h_max_warped <= h1:
        h_max = h1
    if w_max_warped <= w1:
        w_max = w1
    # If any of these corner points are negative, we have to shift the homography to avoid cropping the corners
    # Create shifted homography matrix
    H_shift_matrix = np.array([[1, 0, -w_min], [0, 1, -h_min], [0, 0, 1]], dtype=np.float64)
    H_full_matrix = np.dot(H_shift_matrix, H_matrix)  # Combined homography matrix for shift + perspective warp
    # We then also have to shift the original image and adjust the full size
    w_warped = w_max - w_min
    h_warped = h_max - h_min
    warped_size = w_warped, h_warped

    cv2.imwrite("img1.png", img1)
    cv2.imwrite("img2.png", img2)

    warped_img2 = cv2.warpPerspective(img2, H_full_matrix, warped_size)
    cv2.imshow("warped", warped_img2)
    cv2.imwrite("warped2.png", warped_img2)
    compound_img = warped_img2.copy()
    compound_img[-h_min:h1-h_min, -w_min:w1-w_min] = img1  # Other image gets copied in as is

    cv2.imshow("compound", compound_img)
    cv2.imwrite("compound.png", compound_img)

    # Get area of overlap. 0 = Not covered by any image, 127 = covered by one image, 254 = covered by both images
    # For unwarped image, just set the shifted rectangle to the base value
    mask_raw = np.zeros((h_warped, w_warped), dtype=np.uint8)
    mask_raw[-h_min:h1-h_min, -w_min:w1-w_min] = 127  # original image mask

    # Clip the gray version of second image, then warp it same as colour image and threshold it for mask
    img2_gray = img2_gray.clip(1, 255, img2_gray)
    warped_img2_gray = cv2.warpPerspective(img2_gray, H_full_matrix, warped_size)
    _, mask_img2_warped = cv2.threshold(warped_img2_gray, 0, 127, cv2.THRESH_BINARY)

    # Add both masks and select only those covered by both images
    overlap = mask_raw + mask_img2_warped
    mask = overlap > 200

    # Get largest interior rectangle on the area of overlap
    contours, _ = cv2.findContours((mask*200).astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contour = contours[0][:, 0, :]
    box = lir.lir(mask, contour)
    crop_w_min = box[0]
    crop_w_max = box[0] + box[2]
    crop_h_min = box[1]
    crop_h_max = box[1] + box[3]

    overlap = cv2.cvtColor(overlap, cv2.COLOR_GRAY2BGR)
    contour_overlap = cv2.drawContours(overlap, contours, -1, (0, 255, 0), 1)
    contour_overlap = cv2.rectangle(contour_overlap, lir.pt1(box), lir.pt2(box), (0, 0, 255), 1)
    cv2.imshow("Regions of overlap", contour_overlap)
    cv2.imwrite("overlap.png", contour_overlap)

    #Crop both images to LIR
    cropped_img1 = compound_img[crop_h_min:crop_h_max, crop_w_min: crop_w_max]  # Crop of img1
    cropped_img2 = warped_img2[crop_h_min:crop_h_max, crop_w_min: crop_w_max]  # Cropped of warped img2

    # Get difference between final versions of our two images.
    crop_diff = get_difference_image(cropped_img1, cropped_img2)
    psnr = cv2.PSNR(cropped_img1, cropped_img2)
    ssim = structural_similarity(cropped_img1, cropped_img2, channel_axis=2) * 100
    cv2.imshow("Difference image of crops", crop_diff)
    cv2.imwrite(f"cropdiff_psnr_{psnr:.2f}_ssim_{ssim:.2f}.png", crop_diff)
    print(f"Warped and cropped images - \t PSNR: {psnr:.2f}, SSIM: {ssim:.2f}")
    print(f"Warped and cropped size: W {cropped_img1.shape[1]}, H {cropped_img1.shape[0]}")

def homography_pipe(pair):
    clear_window_id = "Single Image"
    diff_window_id = "Difference Image"
    cv2.imshow(clear_window_id, pair[0])
    diff = get_difference_image(*pair)
    cv2.imshow(diff_window_id, diff)
    ssim = structural_similarity(*pair, channel_axis=2) * 100
    psnr = cv2.PSNR(*pair)
    cv2.imwrite(f"diff_img_psnr_{psnr:.2f}_ssim_{ssim:.2f}.png", diff)
    cv2.setWindowTitle(diff_window_id, f"PSNR: {psnr:.2f}, SSIM: {ssim:.2f}")
    print(f"Base images - \t\t\t PSNR: {psnr:.2f}, SSIM: {ssim:.2f}")
    get_overlap(*pair)
    cv2.waitKey(0)

CHESS_SIZE = (10,7)

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

    # Homography tests on raw images if capture meta info messed up or just for tests
    homography_test_parser = command_parsers.add_parser("homtest")
    homography_test_parser.add_argument("img1", type=str)
    homography_test_parser.add_argument("img2", type=str)

    # Do calibration with series of images of charuco
    calib_parser = command_parsers.add_parser("calib", help="Determine camera calibration")
    calib_parser.add_argument("path", type=str)

    # Make charuco board
    charboard_parser = command_parsers.add_parser("charboard", help="Create a charuco board for calibration")

    # Do calibration with series of images of charuco
    charcalib_parser = command_parsers.add_parser("charcalib", help="Determine camera calibration")
    charcalib_parser.add_argument("path", type=str)

    args = parser.parse_args()

    if args.command == "hom":
        # Do the image correction
        imgs = load_images(args.path)
        for pair in imgs:
            homography_pipe(pair)
    elif args.command == "homtest":
        img1 = cv2.imread(args.img1)
        img2 = cv2.imread(args.img2)
        homography_pipe((img1, img2))
    elif args.command == "calib":
        # TODO: Chess calibration
        pass
    elif args.command == "charboard":
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
    elif args.command == "charcalib":

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
            image = cv2.imread(image_file, cv2.COLOR_BGR2GRAY)

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
            #img_points_subpix = cv2.cornerSubPix(image, img_points)

            all_corners.append(cur_char_corners)
            all_corner_ids.append(cur_char_ids)
            all_marker_corners.append(cur_marker_corners)
            all_marker_ids.append(cur_marker_ids)
            all_obj_points.append(obj_points)
            all_img_points.append(img_points)
            print(f"Detected {obj_points.shape} object points and {img_points.shape} image points")

        ret, cam_matrix, distortion, rvecs, tvecs, stdDev, stdDist, perview = cv2.calibrateCameraExtended(all_obj_points, all_img_points, image_size, INTRINSIC_IPHONE, DISTORTION_IPHONE, None, None)
        print(ret)
        print(cam_matrix, distortion)
        print(stdDev, stdDist)
        print(perview)

        for image in all_images:
            undistorted = cv2.undistort(image, cam_matrix, distortion)
            cv2.namedWindow("Undistorted", cv2.WINDOW_NORMAL)
            cv2.imshow("Undistorted", undistorted)
            cv2.waitKey(0)


if __name__ == "__main__":
    main()
