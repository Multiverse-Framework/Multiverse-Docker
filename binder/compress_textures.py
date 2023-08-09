import os
from PIL import Image
from tqdm import tqdm
import argparse

def list_files(root_dir, ends=".bmp"):
    bmp_files = []
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.lower().endswith(ends):
                bmp_files.append(os.path.join(dirpath, filename))
    return bmp_files

def convert_bmp_to_jpg(bmp_file, max_size):
    # Open the BMP image using Pillow
    image = Image.open(bmp_file)
    
    if max(image.size) > max_size and image.size[0] == image.size[1]:
        image.thumbnail((max_size, max_size))
    
    output_file = os.path.splitext(bmp_file)[0] + ".jpg"
    # Convert and save the image as JPG
    image.save(output_file, "JPEG")
    return output_file

def replace_texture_file(text_files, find_text, replace_text):
    for txt_file in text_files:
        with open(txt_file, 'r') as file:
            filedata = file.read()
        filedata = filedata.replace(find_text, replace_text)
        with open(txt_file, 'w') as file:
            file.write(filedata)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert BMP texture files to JPG for Rvizweb.')
    parser.add_argument('--img_path', help='Path of texure files.')
    parser.add_argument('--max_size', type=int, help='Image maxium size, default value: 512.')
    args = parser.parse_args()
    IMG_DIR = args.img_path or "/home/workspace/ros/src/iai_maps/iai_apartment/meshes/"
    MAX_SIZE = args.max_size or 512

    bmp_files = list_files(IMG_DIR, ".bmp")
    dae_files = list_files(IMG_DIR, ".dae")
    mtl_files = list_files(IMG_DIR, ".mtl")
    obj_files = list_files(IMG_DIR, ".obj")

    for i in tqdm(bmp_files):
        new_img = os.path.basename(convert_bmp_to_jpg(i, MAX_SIZE))
        old_img = os.path.basename(i)
        replace_texture_file(dae_files, old_img, new_img)
        replace_texture_file(mtl_files, old_img, new_img)
        replace_texture_file(obj_files, old_img, new_img)
        os.remove(i)
