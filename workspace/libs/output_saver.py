import cv2
class OutputSaver:
    def __init__(self, cfg):
        self.cfg = cfg

    def save_images(self, images, dirname):
        path = f"{self.cfg.save_path}/{dirname}"
        for i, img in enumerate(images):
            cv2.imwrite(f"./{path}/{i}.png", img)
        print(f"[Saver] Saved images in {path}")