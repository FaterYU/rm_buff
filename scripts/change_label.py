import cv2
import numpy as np
import os
import tqdm

dataset_path = 'datasets/buff_data/'
new_dataset_path = 'datasets/buff_segmentation_data/'
train_test_ratio = 0.8

if not os.path.exists(new_dataset_path):
    os.makedirs(new_dataset_path)
    os.makedirs(new_dataset_path+'images')
    os.makedirs(new_dataset_path+'labels')


def get_keypoints(srcImg, label):
    P = [[label[i], label[1+i]] for i in range(5, 15, 2)]
    P.remove(P[2])
    for i in range(len(P)):
        P[i][0] = round(float(P[i][0])*srcImg.shape[1])
        P[i][1] = round(float(P[i][1])*srcImg.shape[0])
    # print(P)
    img = np.zeros((712, 372), np.uint8)
    cpl = []
    # cpl.append([0, 138])
    # cpl.append([28,25])
    # cpl.append([125,12])
    # cpl.append([235,5])
    # cpl.append([345,0])
    # cpl.append([381, 155])
    # cpl.append([711, 155])
    cpl.append([138, 0])
    cpl.append([25, 28])
    cpl.append([12, 125])
    cpl.append([5, 235])
    cpl.append([0, 345])
    cpl.append([155, 381])
    cpl.append([155, 711])
    for i in range(len(cpl)-1, -1, -1):
        cpl.append([371-cpl[i][0], cpl[i][1]])
    points = np.array(cpl, np.int32)
    img = cv2.fillConvexPoly(img, points, 255)
    kpt = []
    kpt.append([26, 131])
    kpt.append([20, 234])
    for i in range(len(kpt)-1, -1, -1):
        kpt.append([371-kpt[i][0], kpt[i][1]])
    # print(kpt)
    srcPoints = np.array(kpt, np.float32)
    dstPoints = np.array(P, np.float32)
    M = cv2.getPerspectiveTransform(srcPoints, dstPoints)

    # cal piont in dst
    dst_cpl = []
    for i in range(len(cpl)):
        p = np.array([cpl[i][0], cpl[i][1], 1])
        p = np.dot(M, p)
        p = p/p[2]
        p[0] /= srcImg.shape[1]
        p[1] /= srcImg.shape[0]
        dst_cpl.append([p[0], p[1]])

    # dst = cv2.warpPerspective(img, M, (srcImg.shape[1], srcImg.shape[0]))
    # not_dst = cv2.bitwise_not(dst)
    # mask_img = cv2.bitwise_and(srcImg, srcImg, mask=dst)
    # cv2.imshow('mask_img', mask_img)
    # cv2.imshow('src', srcImg)
    # cv2.imshow('image', dst)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return dst_cpl

print('change label')
name_list = []
for name in tqdm.tqdm(os.listdir(dataset_path+'images')):
    label_path = dataset_path+'labels/'+name.split('.')[0]+'.txt'
    if not os.path.exists(label_path):
        continue
    try:
        labels = []
        with open(label_path, 'r') as f:
            for line in f:
                labels.append(line.strip().split())
        image = cv2.imread(dataset_path+'images/'+name)
        coordinates = []
        for label in labels:
            if int(label[0]) % 2 == 1:
                continue
            coordinate = [int(label[0])]
            for i in get_keypoints(image, label):
                coordinate.append(i[0])
                coordinate.append(i[1])
            coordinates.append(coordinate)
    except:
        print('\nerror and skip:', name)
        continue
    new_label_path = new_dataset_path+'labels/'+name.split('.')[0]+'.txt'
    with open(new_label_path, 'w') as f:
        for coordinate in coordinates:
            f.write(' '.join(map(str, coordinate))+'\n')
    cv2.imwrite(new_dataset_path+'images/'+name, image)
    name_list.append(name.split('.')[0])

# split train and test
print('split train and test')
train_images_path = new_dataset_path+'images/train/'
test_images_path = new_dataset_path+'images/test/'
train_labels_path = new_dataset_path+'labels/train/'
test_labels_path = new_dataset_path+'labels/test/'
if not os.path.exists(train_images_path):
    os.makedirs(train_images_path)
    os.makedirs(test_images_path)
    os.makedirs(train_labels_path)
    os.makedirs(test_labels_path)

for name in tqdm.tqdm(name_list):
    try:
        if np.random.rand() < train_test_ratio:
            os.rename(new_dataset_path+'images/'+name+'.jpg', train_images_path+name+'.jpg')
            os.rename(new_dataset_path+'labels/'+name+'.txt', train_labels_path+name+'.txt')
        else:
            os.rename(new_dataset_path+'images/'+name+'.jpg', test_images_path+name+'.jpg')
            os.rename(new_dataset_path+'labels/'+name+'.txt', test_labels_path+name+'.txt')
    except:
        print('\nerror and skip:', name)
        continue

# for i in label:
#     x, y, w, h = map(float, i[1:5])
#     cv2.rectangle(image, (int((x - w/2)*image.shape[1]), int((y - h/2)*image.shape[0])), (int(
#         (x + w/2)*image.shape[1]), int((y + h/2)*image.shape[0])), (0, 255, 0), 1)
#     for j in range(5, len(i), 2):
#         cv2.circle(image, (int(float(
#             i[j])*image.shape[1]), int(float(i[j+1])*image.shape[0])), 2, (0, 255, 0), -1)
#         # cv2.putText(image, str((j-5)//2), (int(float(i[j])*image.shape[1]), int(
#         #     float(i[j+1])*image.shape[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

# cv2.imshow('image', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# standard_img(image,label)
