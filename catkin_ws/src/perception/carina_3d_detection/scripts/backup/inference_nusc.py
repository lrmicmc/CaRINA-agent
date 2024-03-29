import mmcv
import numpy as np
import torch
from copy import deepcopy
from mmcv.parallel import collate, scatter
from mmcv.runner import load_checkpoint
from os import path as osp

from mmdet3d.core import Box3DMode, show_result
from mmdet3d.core.bbox import get_box_type
from mmdet3d.datasets import NuScenesDataset
from mmdet3d.datasets.pipelines import Compose
from mmdet3d.models import build_detector

import time

def convert_SyncBN(config):
    if isinstance(config, dict):
        for item in config:
            if item == 'norm_cfg':
                config[item]['type'] = config[item]['type']. \
                                       replace('naiveSyncBN', 'BN')
            else:
                convert_SyncBN(config[item])
    return


def get_nuscenes_lidar_top_data(root_path, samples_num, pkl_name, pipeline):
    nus_dataset = NuScenesDataset(
        pkl_name, pipeline, root_path, test_mode=True)
    data = nus_dataset[samples_num]
    return data


def init_detector_nusc(config, checkpoint=None, device='cuda:0'):
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
    convert_SyncBN(config.model)
    model = build_detector(config.model, test_cfg=config.get('test_cfg'))
    # model = build_detector(config.model, test_cfg=config.test_cfg)
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


def inference_detector_nusc(model, pcd=None, data_root=None, version=None):
    """Inference point cloud with the detector.

    Args:
        model (nn.Module): The loaded detector.
        pcd (str): Point cloud files.

    Returns:
        tuple: Predicted results and data from pipeline.
    """
    cfg = model.cfg
    device = next(model.parameters()).device  # model device
    # build the data pipeline
    test_pipeline = deepcopy(cfg.data.test.pipeline)
    test_pipeline = Compose(test_pipeline)
    box_type_3d, box_mode_3d = get_box_type(cfg.data.test.box_type_3d)

    if pcd is None:
        if version is None:
            # for unittest or modified nuscenes dataset structure
            pkl_name = 'nus_info.pkl'
        else:
            pkl_name = 'nuscenes-' + version + '_infos_train.pkl'
        pkl_name = osp.join(data_root, pkl_name)
        sample_num = 1
        data = get_nuscenes_lidar_top_data(data_root, sample_num, pkl_name,
                                           cfg.data.test.pipeline)
    else:
        data = dict(
            pts_filename=pcd,
            box_type_3d=box_type_3d,
            box_mode_3d=box_mode_3d,
            img_fields=[],
            bbox3d_fields=[],
            pts_mask_fields=[],
            pts_seg_fields=[],
            bbox_fields=[],
            mask_fields=[],
            seg_fields=[])
        data = test_pipeline(data)
    # print(data)
    data = collate([data], samples_per_gpu=1)
    # print(data)
    if next(model.parameters()).is_cuda:
        # scatter to specified GPU
        data = scatter(data, [device.index])[0]
    else:
        # this is a workaround to avoid the bug of MMDataParallel
        data['img_metas'] = data['img_metas'][0].data
        data['points'] = data['points'][0].data
    # forward the model
    # print(data)
    time_ini=time.time()
    with torch.no_grad():
        result = model(return_loss=False, rescale=True, **data)
    time_fin=time.time()
    print('inference time ',time_fin-time_ini)
    return result, data


def show_result_meshlab_nusc(data, result, out_dir):
    """Show result by meshlab.

    Args:
        data (dict): Contain data from pipeline.
        result (dict): Predicted result from model.
        out_dir (str): Directory to save visualized result.
    """

    points = data['points'][0][0].cpu().numpy()
    # print('show points',points)
    if isinstance(data['img_metas'][0][0]['pts_filename'], np.ndarray):
        pts_filename = "ros"#data['img_metas'][0][0]['pts_filename']
        file_name = "ros"
    else:
        pts_filename = data['img_metas'][0][0]['pts_filename']
        file_name = osp.split(pts_filename)[-1].split('.')[0]
    assert out_dir is not None, 'Expect out_dir, got none.'
    if 'pts_bbox' in result[0].keys():
        pred_bboxes = result[0]['pts_bbox']['boxes_3d'].tensor.numpy()
        # we should visualise the origin pointcloud
        origin_points_indices = np.equal(points[:, 3], 0)
        points = points[origin_points_indices]
    else:
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
    return points, pred_bboxes, out_dir, file_name
