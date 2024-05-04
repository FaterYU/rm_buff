import os
import argparse
import shutil
import random
import tqdm


def split_dataset(dataset_path, output_path, train_ratio=0.8):
    train_path = os.path.join(output_path, 'train')
    test_path = os.path.join(output_path, 'test')
    if not os.path.exists(output_path):
        os.makedirs(output_path)
        if not os.path.exists(train_path):
            os.makedirs(train_path)
        if not os.path.exists(test_path):
            os.makedirs(test_path)

    for class_name in os.listdir(dataset_path):
        print('class: %s' % class_name)
        class_path = os.path.join(dataset_path, class_name)
        train_class_path = os.path.join(train_path, class_name)
        test_class_path = os.path.join(test_path, class_name)
        if not os.path.exists(train_class_path):
            os.makedirs(train_class_path)
        if not os.path.exists(test_class_path):
            os.makedirs(test_class_path)

        img_list = os.listdir(class_path)
        train_img_list = random.sample(
            img_list, int(len(img_list) * train_ratio))
        test_img_list = list(set(img_list) - set(train_img_list))

        print('working at train...')
        for img_name in tqdm.tqdm(train_img_list):
            shutil.copy(os.path.join(class_path, img_name),
                        os.path.join(train_class_path, img_name))
        print('working at test...')
        for img_name in tqdm.tqdm(test_img_list):
            shutil.copy(os.path.join(class_path, img_name),
                        os.path.join(test_class_path, img_name))

        print('class: %s, train: %d, test: %d' %
              (class_name, len(train_img_list), len(test_img_list)))


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default='./datasets/buff_data',
                        required=False, help='dataset path which contains images and labels')
    parser.add_argument('--ratio', type=float, default=0.8,
                        required=False, help='train ratio')
    parser.add_argument('--output', type=str, required=False,
                        default='./datasets/buff_format/', help='output path')
    args = parser.parse_args()

    split_dataset(args.data, args.output, args.ratio)
