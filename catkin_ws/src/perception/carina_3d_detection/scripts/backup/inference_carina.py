import mmcv
import torch
from copy import deepcopy
from mmcv.parallel import collate, scatter
from mmcv.runner import load_checkpoint
from os import path as osp

from mmdet3d.core import Box3DMode, show_result
from mmdet3d.core.bbox import get_box_type
from mmdet3d.datasets.pipelines import Compose
from mmdet3d.models import build_detector


from mmdet3d.core.bbox.structures.lidar_box3d import LiDARInstance3DBoxes
# from mmdet3d.core.bbox import Box3DMode

import torch
import numpy as np
def init_detector(config, checkpoint=None, device='cuda:0'):
    """Initialize a detector from config file.

    Args:
        config (str or :obj:`mmcv.Config`): Config file path or the config
            object.
        checkpoint (str, optional): Checkpoint path. If left as None, the model
            will not load any weights.
        device (str): Device to use.

    Returns:
        nn.Module: The constructed detector.
    """
    if isinstance(config, str):
        config = mmcv.Config.fromfile(config)
    elif not isinstance(config, mmcv.Config):
        raise TypeError('config must be a filename or Config object, '
                        f'but got {type(config)}')
    config.model.pretrained = None
    config.model.train_cfg = None
    model = build_detector(config.model, test_cfg=config.get('test_cfg'))
    if checkpoint is not None:
        checkpoint = load_checkpoint(model, checkpoint)
        if 'CLASSES' in checkpoint['meta']:
            model.CLASSES = checkpoint['meta']['CLASSES']
        else:
            model.CLASSES = config.class_names
    model.cfg = config  # save the config in the model for convenience
    model.to(device)
    model.eval()
    return model


def inference_detector(model, pcd):


    """Inference point cloud with the detector.

    Args:
        model (nn.Module): The loaded detector.
        pcd (str): Point cloud files.

    Returns:
        tuple: Predicted results and data from pipeline.
    """
    cfg = model.cfg
    # device = next(model.parameters()).device  # model device

    # # build the data pipeline
    # test_pipeline = deepcopy(cfg.data.test.pipeline)
    # test_pipeline = Compose(test_pipeline)
    # box_type_3d, box_mode_3d = get_box_type(cfg.data.test.box_type_3d)
    # data = dict(
    #     pts_filename=pcd,
    #     box_type_3d=box_type_3d,
    #     box_mode_3d=box_mode_3d,
    #     img_fields=[],
    #     bbox3d_fields=[],
    #     pts_mask_fields=[],
    #     pts_seg_fields=[],
    #     bbox_fields=[],
    #     mask_fields=[],
    #     seg_fields=[])
    # data = test_pipeline(data)
    # data = collate([data], samples_per_gpu=1)

    # if next(model.parameters()).is_cuda:
    #     # scatter to specified GPU
    #     data = scatter(data, [device.index])[0]
    # else:
    #     # this is a workaround to avoid the bug of MMDataParallel
    #     data['img_metas'] = data['img_metas'][0].data
    #     data['points'] = data['points'][0].data
    # forward the model
    # box_type_3d= LiDARInstance3DBoxes(torch.tensor([[ ]]))
    data={'img_metas': [[{'flip': False, 'pcd_horizontal_flip': False, 'pcd_vertical_flip': False, 'box_mode_3d': Box3DMode.LIDAR, 
    'box_type_3d': LiDARInstance3DBoxes, 
    'pcd_trans': np.array([0., 0., 0.]), 'pcd_scale_factor': 1.0, 'pts_filename': ''}]], 
    # 'points': [[torch.tensor([[ 2.1554e+01,  2.8000e-02,  9.3800e-01,  3.4000e-01],
    #     [ 2.1240e+01,  9.4000e-02,  9.2700e-01,  2.4000e-01],
    #     [ 2.1056e+01,  1.5900e-01,  9.2100e-01,  5.3000e-01],
    #     [ 6.3150e+00, -3.1000e-02, -1.6490e+00,  2.9000e-01],
    #     [ 6.3090e+00, -2.1000e-02, -1.6470e+00,  2.9000e-01],
    #     [ 6.3110e+00, -1.0000e-03, -1.6480e+00,  3.2000e-01]], device='cuda:0')]]}
    'points': [[torch.tensor(pcd.astype(np.float32), device='cuda:0')]]}


    print(data)
    with torch.no_grad():
        result = model(return_loss=False, rescale=True, **data)
    return result, data


def show_result_meshlab(data, result, out_dir):
    """Show result by meshlab.

    Args:
        data (dict): Contain data from pipeline.
        result (dict): Predicted result from model.
        out_dir (str): Directory to save visualized result.
    """
    points = data['points'][0][0].cpu().numpy()
    pts_filename = data['img_metas'][0][0]['pts_filename']
    file_name = osp.split(pts_filename)[-1].split('.')[0]

    assert out_dir is not None, 'Expect out_dir, got none.'

    pred_bboxes = result[0]['boxes_3d'].tensor.numpy()
    # for now we convert points into depth mode
    if data['img_metas'][0][0]['box_mode_3d'] != Box3DMode.DEPTH:
        points = points[..., [1, 0, 2]]
        points[..., 0] *= -1
        pred_bboxes = Box3DMode.convert(pred_bboxes,
                                        data['img_metas'][0][0]['box_mode_3d'],
                                        Box3DMode.DEPTH)
        pred_bboxes[..., 2] += pred_bboxes[..., 5] / 2
    else:
        pred_bboxes[..., 2] += pred_bboxes[..., 5] / 2
    show_result(points, None, pred_bboxes, out_dir, file_name)
