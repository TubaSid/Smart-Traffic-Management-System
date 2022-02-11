import os
def remove_img(folder):
    folder_path = (folder)
    test = os.listdir(folder_path)
    for images in test:
        if images.endswith(".jpg"):
            os.remove(os.path.join(folder_path, images))

