import os
import cv2
class DataLoader:
    def __init__(self, cfg):
        self.cfg = cfg


    def load_images(self, path):
        # Get filenames
        filenames = os.listdir(path)

        # Init image list
        imgs = [None] * len(filenames)
         
        # Read images into image list
        for i, filename in enumerate(sorted(filenames)):
            imgs[i] = cv2.imread(f"{path}/{filename}")
            print(f"{path}/{filename}")

        # Check images properly loaded
        if imgs[-1] is not None:
            print(f"[Loader] Loaded images in {path}")
        else:
            raise Exception(f"[Loader] Failed to load images in {path}")

        return imgs
