import nncf
import openvino.runtime as ov
import torch
from torchvision import datasets, transforms

import os
from PIL import Image
import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import yaml

class CustomDataset(Dataset):
    def __init__(self, data_path, split='train', transform=None):
        self.data_path = data_path
        self.split = split
        self.transform = transform
        self.image_folder = os.path.join(data_path, f'images/{split}')
        self.labels_folder = os.path.join(data_path, f'labels/{split}')
        w=os.walk(self.image_folder)
        data_list = []
        for _, _, files in w:
            for file in files:
                name=file.split('.')[0]
                if os.path.exists(os.path.join(self.labels_folder, name+'.txt')):
                    data_list.append(name)
        self.data_list=data_list[:300]


    def __len__(self):
        return len(self.data_list)

    def __getitem__(self, idx):
        img_name = os.path.join(self.image_folder, self.data_list[idx] + '.jpg')
        image = Image.open(img_name).convert('RGB')
        label_name = os.path.join(self.labels_folder, self.data_list[idx] + '.txt')
        label = []
        with open(label_name) as f:
            for line in f:
                label.append(list(map(float, line.split())))
        keypoints = torch.tensor(label, dtype=torch.float32)
        if self.transform:
            image = self.transform(image)
        
        # resize to 640*640
        image = transforms.Resize((640, 640))(image)

        return image, keypoints


# Instantiate your uncompressed model
model = ov.Core().read_model("./best_100b_openvino_model/best_100b.xml")

# Provide validation part of the dataset to collect statistics needed for the compression algorithm
data_path = './datasets/buff_format'
val_dataset = CustomDataset(data_path,split='val',transform=transforms.Compose([transforms.ToTensor()]))
dataset_loader = torch.utils.data.DataLoader(val_dataset, batch_size=1, shuffle=False)

# Step 1: Initialize transformation function


def transform_fn(data_item):
    images, _ = data_item
    return images


# Step 2: Initialize NNCF Dataset
calibration_dataset = nncf.Dataset(dataset_loader, transform_fn)
# Step 3: Run the quantization pipeline
quantized_model = nncf.quantize(model, calibration_dataset)

ov.save_model(quantized_model, "./best_100b_openvino_model/best_100b_quantized.xml")
