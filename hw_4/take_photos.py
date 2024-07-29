
#!/usr/bin/env python3
import argparse
import cv2
import os
from os import path
from os.path import abspath
import textwrap
import traceback


# def parse_arguments():
#     this_dir = path.dirname(abspath(__file__))
#     arg_parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
#                                          description=textwrap.dedent('''\
#         ================================== Description ===================================
#         This script is needed in order to take pictures. When launched, a window opens  in
#         which the image from the camera is displayed. To save the current frame, press the
#         space bar or the 's' key. To exit press Escape
#         =================================================================================='''))

#     arg_parser.add_argument("-ci", "--cameraIndex", type=int, default=0,
#                             help="Camera index in system. Default: 0")
#     # arg_parser.add_argument("-fw", "--frameWidth", type=int, required=True,
#     #                         help="Frame width")
#     # arg_parser.add_argument("-fh", "--frameHeight", type=int, required=True,
#     #                         help="Frame height")
#     arg_parser.add_argument("-op", "--outputPath", type=str, default=f'{this_dir}/../images',
#                             help=f"Path - where the images will be saved. Default: '{abspath(f'{this_dir}/../images')}'"
#                                  "\nThe script creates folder if it doesn't exist, deletes all existing files in it")

#     return vars(arg_parser.parse_args())


def configure_video_capturer(camera_index, frame_width=None, frame_height=None):
    cap = cv2.VideoCapture(camera_index)

    if frame_width != None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    if frame_height != None:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    if not cap.isOpened():
        raise ValueError(f"Can not open camera by index {camera_index}")

    return cap


def take_images(video_capturer, output_path):
    if not path.isdir(output_path):
        os.makedirs(output_path)
    else:
        for f in os.listdir(output_path):
            os.remove(os.path.join(output_path, f))

    count = 1
    while video_capturer.isOpened():
        ret_val, frame = video_capturer.read()

        if ret_val:
            cv2.imshow('img', frame)
            key = cv2.waitKey(1)

            # save frame to file when pressing spacebar or 's' key
            if key == ord(' ') or key == ord('s'):
                file_path = f"{output_path}/img{count}.png"
                cv2.imwrite(file_path, frame)
                count += 1
            # exit when pressing 'q' or Esc
            elif key == ord('q') or key == 27:
                break

    cv2.destroyAllWindows()


def main(camera_index):
    # args = parse_arguments()
    # camera_index = args['cameraIndex']
    # frame_width = args['frameWidth']
    # frame_height = args['frameHeight']
    # output_path = abspath(args['outputPath'])
    output_path = './images/'

    try:
        capturer = configure_video_capturer(camera_index) # frame_width, frame_height)
        take_images(capturer, output_path)
        capturer.release()
        print(f"Taken images were saved in '{output_path}'")

    except ValueError as e:
        print("\033[31m{}\033[0m".format(f'[ ERROR: ] {e}'))

    except cv2.error as e:
        print("\033[31m{}\033[0m".format(f'[ OPENCV ERROR: ] {e}\n{traceback.format_exc()}'))

    except Exception:
        print("\033[31m{}\033[0m".format(traceback.format_exc()))

    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(0)
