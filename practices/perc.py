# import camera
from cortano import RemoteInterface
from torchvision.models.detection import maskrcnn_resnet50_fpn
from torchvision import transforms
from PIL import Image
import torch
import cv2
import numpy as np

# labels are from here (COCO): https://gist.github.com/tersekmatija/9d00c4683d52d94cf348acae29e8db1a

def get_XYZ(depth_image):
    fx = 460.92495728
    fy = 460.85058594
    cx = 315.10949707
    cy = 176.72598267
    h, w = (360, 640)
    U = np.tile(np.arange(w).reshape((1, w)), (h, 1))
    V = np.tile(np.arange(h).reshape((h, 1)), (1, w))
    U = (U - cx) / fx
    V = (V - cy) / fy

    Z = depth_image
    X = U * Z
    Y = V * Z
    # formatting magic
    XYZ = np.concatenate((
        X.reshape((-1, 1)),
        Y.reshape((-1, 1)),
        Z.reshape((-1, 1))
    ), axis=-1)
    return XYZ

if __name__ == "__main__":
    robot = RemoteInterface(Config.ip)
    # cam = camera.RealsenseCamera() # because this is a locally run camera, but you don't need

    # object detection model
    model = maskrcnn_resnet50_fpn(pretrained=True, pretrained_backbone=True)
    model.eval()
    model.to('cuda')


    while True:
        robot.update()
        color, depth, sensors = robot.read()
        # color, depth = robot.read()

        preprocess = transforms.Compose([ transforms.ToTensor(), ])
        input_tensor = preprocess(Image.fromarray(color))
        input_batch = input_tensor.unsqueeze(0).to('cuda')
        with torch.no_grad():
            output = model(input_batch)[0]
        output = {l: output[l].to('cpu').numpy() for l in output}
        # print("-----------------")
        # print(output["scores"].shape)
        # print(output["labels"].shape)
        # print(output["masks"].shape)

        object_index = 37
        indeces_found = np.logical_and(output["labels"]==object_index, output["scores"] > 0.25)
        scores = output["scores"][indeces_found] # between 0 and 1, 1 being most confident
        labels = output["labels"][indeces_found] # integer id of the object found, refer to COCO classes
        masks  = output["masks"][indeces_found] # mask in the image (N, 1, height, width), between 0 and 1

        print("-----------------")
        print(scores.shape)
        print(scores.shape)
        print(masks.shape)

        # get a single mask's centered XYZ coordinates, ball's location
        single_mask = np.zeros((360, 640), dtype=np.uint8)
        # if len(masks) > 0:
            # single_mask = masks[0].reshape((360, 640))
        for idx in range(len(masks)):
            reshaped_mask = masks[idx].reshape((360, 640))
            indecies = reshaped_mask>0.25
            single_mask[indecies] = 255
            
        ball_depth = depth * (single_mask > 0)
        xyz = get_XYZ(ball_depth)
        num_pixels = np.sum(ball_depth > 0)
        if num_pixels > 0:
            average_xyz = np.sum(xyz, axis=0) / num_pixels

        if num_pixels > 0:
            print(average_xyz)
        
        cv2.imshow("color", single_mask)
        cv2.waitKey(1)
