norm_cfg = dict(type='SyncBN', requires_grad=True)
model = dict(
    type='EncoderDecoder',
    pretrained='open-mmlab://resnet50_v1c',
    backbone=dict(
        type='ResNetV1c',
        depth=18,
        in_channels=15,
        num_stages=4,
        out_indices=(0, 1, 2, 3),
        dilations=(1, 1, 1, 1),
        strides=(1, 2, 2, 2),
        norm_cfg=dict(type='SyncBN', requires_grad=True),
        norm_eval=False,
        style='pytorch',
        contract_dilation=True),
    neck=dict(
        type='FPN',
        in_channels=[64, 128, 256, 512],
        out_channels=256,
        num_outs=4),
    decode_head=dict(
        type='FPNHead',
        in_channels=[256, 256, 256, 256],
        in_index=[0, 1, 2, 3],
        feature_strides=[4, 8, 16, 32],
        channels=128,
        dropout_ratio=0.1,
        num_classes=23,
        norm_cfg=dict(type='SyncBN', requires_grad=True),
        align_corners=False,
        loss_decode=dict(
            type='CrossEntropyLoss', use_sigmoid=False, loss_weight=1.0)),
    regression_head=dict(
        type='LinearRegHead',
        num_points=200,
        in_channels=512,
        loss=dict(type='MSELoss', loss_weight=1.0)),
    train_cfg=dict(),
    test_cfg=dict(mode='whole'))
dataset_type = 'CarlaSegBEVFusionDataset'
data_root = '/mnt/Data1/carla_dataset/'
img_norm_cfg = dict(
    mean=[0.0, 0.0, 0.0], std=[255.0, 255.0, 255.0], to_rgb=False)
CLASSES = ('Building', 'Fence', 'Pedestrian', 'RoadLine', 'Road', 'SideWalk',
           'Vegetation', 'Vehicles', 'Wall', 'Ground', 'GuardRail', 'Static',
           'Dynamic', 'Terrain')
PALETTE = [[255, 0, 0], [2, 0, 0], [4, 0, 0], [6, 0, 0], [7, 0, 0], [8, 0, 0],
           [9, 0, 0], [10, 0, 0], [11, 0, 0], [14, 0, 0], [17, 0, 0],
           [19, 0, 0], [20, 0, 0], [22, 0, 0]]
PALETTE2 = [[255, 0, 0], [2, 0, 0], [4, 0, 0], [6, 0, 0], [7, 255,
                                                           0], [8, 0, 0],
            [9, 0, 0], [10, 0, 0], [11, 0, 0], [14, 0, 0], [17, 0, 0],
            [19, 0, 0], [20, 0, 0], [22, 0, 0]]
train_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='LoadAnnotations'),
    dict(type='RandomRotate', prob=0.5, degree=45),
    dict(
        type='Normalize',
        mean=[0.0, 0.0, 0.0],
        std=[255.0, 255.0, 255.0],
        to_rgb=False),
    dict(type='DefaultFormatBundle'),
    dict(type='Collect', keys=['img', 'gt_semantic_seg', 'path_gt'])
]
test_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(
        type='MultiScaleFlipAug',
        img_scale=(650, 750),
        flip=False,
        transforms=[
            dict(
                type='Normalize',
                mean=[0.0, 0.0, 0.0],
                std=[255.0, 255.0, 255.0],
                to_rgb=False),
            dict(type='TestFormatBundle'),
            dict(type='Collect', keys=['img'])
        ])
]
data = dict(
    samples_per_gpu=2,
    workers_per_gpu=2,
    train=dict(
        points_regression=200,
        type='CarlaSegBEVFusionDataset',
        classes=('Building', 'Fence', 'Pedestrian', 'RoadLine', 'Road',
                 'SideWalk', 'Vegetation', 'Vehicles', 'Wall', 'Ground',
                 'GuardRail', 'Static', 'Dynamic', 'Terrain'),
        palette=[[255, 0, 0], [2, 0, 0], [4, 0, 0], [6, 0, 0], [7, 0, 0],
                 [8, 0, 0], [9, 0, 0], [10, 0, 0], [11, 0, 0], [14, 0, 0],
                 [17, 0, 0], [19, 0, 0], [20, 0, 0], [22, 0, 0]],
        data_root='/mnt/Data1/carla_dataset/',
        img_dir_stereo='/mnt/Data1/carla_dataset/train/inputs/stereo_bev',
        img_dir_lidar='/mnt/Data1/carla_dataset/train/inputs/lidar_bev_color',
        img_dir_global_plan_p=
        '/mnt/Data1/carla_dataset/train/inputs/global_plan/points',
        img_dir_global_plan_l=
        '/mnt/Data1/carla_dataset/train/inputs/global_plan/img_line',
        img_dir_gps_back=
        '/mnt/Data1/carla_dataset/train/inputs/gps_backward/img_line',
        ann_path_csv_file=
        '/mnt/Data1/carla_dataset/train/gt/path_pred/path_pred.txt',
        ann_dir='/mnt/Data1/carla_dataset/train/gt/segmentation_aerial_gray',
        ann_route_dir='/mnt/Data1/carla_dataset/train/gt/path_pred/img_line',
        pipeline=[
            dict(type='LoadImageFromFile'),
            dict(type='LoadAnnotations'),
            dict(type='RandomRotate', prob=0.5, degree=45),
            dict(
                type='Normalize',
                mean=[0.0, 0.0, 0.0],
                std=[255.0, 255.0, 255.0],
                to_rgb=False),
            dict(type='DefaultFormatBundle'),
            dict(type='Collect', keys=['img', 'gt_semantic_seg', 'path_gt'])
        ]),
    val=dict(
        points_regression=200,
        type='CarlaSegBEVFusionDataset',
        classes=('Building', 'Fence', 'Pedestrian', 'RoadLine', 'Road',
                 'SideWalk', 'Vegetation', 'Vehicles', 'Wall', 'Ground',
                 'GuardRail', 'Static', 'Dynamic', 'Terrain'),
        palette=[[255, 0, 0], [2, 0, 0], [4, 0, 0], [6, 0, 0], [7, 255, 0],
                 [8, 0, 0], [9, 0, 0], [10, 0, 0], [11, 0, 0], [14, 0, 0],
                 [17, 0, 0], [19, 0, 0], [20, 0, 0], [22, 0, 0]],
        data_root='/mnt/Data1/carla_dataset/',
        img_dir_stereo='/mnt/Data1/carla_dataset/validation/inputs/stereo_bev',
        img_dir_lidar=
        '/mnt/Data1/carla_dataset/validation/inputs/lidar_bev_color',
        img_dir_global_plan_p=
        '/mnt/Data1/carla_dataset/validation/inputs/global_plan/points',
        img_dir_global_plan_l=
        '/mnt/Data1/carla_dataset/validation/inputs/global_plan/img_line',
        img_dir_gps_back=
        '/mnt/Data1/carla_dataset/validation/inputs/gps_backward/img_line',
        ann_path_csv_file=
        '/mnt/Data1/carla_dataset/validation/gt/path_pred/path_pred.txt',
        ann_dir=
        '/mnt/Data1/carla_dataset/validation/gt/segmentation_aerial_gray',
        ann_route_dir=
        '/mnt/Data1/carla_dataset/validation/gt/path_pred/img_line',
        pipeline=[
            dict(type='LoadImageFromFile'),
            dict(
                type='MultiScaleFlipAug',
                img_scale=(650, 750),
                flip=False,
                transforms=[
                    dict(
                        type='Normalize',
                        mean=[0.0, 0.0, 0.0],
                        std=[255.0, 255.0, 255.0],
                        to_rgb=False),
                    dict(type='TestFormatBundle'),
                    dict(type='Collect', keys=['img'])
                ])
        ]),
    test=dict(
        points_regression=200,
        type='CarlaSegBEVFusionDataset',
        classes=('Building', 'Fence', 'Pedestrian', 'RoadLine', 'Road',
                 'SideWalk', 'Vegetation', 'Vehicles', 'Wall', 'Ground',
                 'GuardRail', 'Static', 'Dynamic', 'Terrain'),
        palette=[[255, 0, 0], [2, 0, 0], [4, 0, 0], [6, 0, 0], [7, 255, 0],
                 [8, 0, 0], [9, 0, 0], [10, 0, 0], [11, 0, 0], [14, 0, 0],
                 [17, 0, 0], [19, 0, 0], [20, 0, 0], [22, 0, 0]],
        data_root='/mnt/Data1/carla_dataset/',
        img_dir_stereo='/mnt/Data1/carla_dataset/validation/inputs/stereo_bev',
        img_dir_lidar=
        '/mnt/Data1/carla_dataset/validation/inputs/lidar_bev_color',
        img_dir_global_plan_p=
        '/mnt/Data1/carla_dataset/validation/inputs/global_plan/points',
        img_dir_global_plan_l=
        '/mnt/Data1/carla_dataset/validation/inputs/global_plan/img_line',
        img_dir_gps_back=
        '/mnt/Data1/carla_dataset/validation/inputs/gps_backward/img_line',
        ann_path_csv_file=
        '/mnt/Data1/carla_dataset/validation/gt/path_pred/path_pred.txt',
        ann_dir=
        '/mnt/Data1/carla_dataset/validation/gt/segmentation_aerial_gray',
        ann_route_dir=
        '/mnt/Data1/carla_dataset/validation/gt/path_pred/img_line',
        pipeline=[
            dict(type='LoadImageFromFile'),
            dict(
                type='MultiScaleFlipAug',
                img_scale=(650, 750),
                flip=False,
                transforms=[
                    dict(
                        type='Normalize',
                        mean=[0.0, 0.0, 0.0],
                        std=[255.0, 255.0, 255.0],
                        to_rgb=False),
                    dict(type='TestFormatBundle'),
                    dict(type='Collect', keys=['img'])
                ])
        ]))
log_config = dict(
    interval=50, hooks=[dict(type='TextLoggerHook', by_epoch=False)])
dist_params = dict(backend='nccl')
log_level = 'INFO'
load_from = None
resume_from = None
workflow = [('train', 1)]
cudnn_benchmark = True
optimizer = dict(type='SGD', lr=0.01, momentum=0.9, weight_decay=0.0005)
optimizer_config = dict()
lr_config = dict(policy='poly', power=0.9, min_lr=1e-05, by_epoch=False)
runner = dict(type='IterBasedRunner', max_iters=2500000)
checkpoint_config = dict(by_epoch=False, interval=8000)
evaluation = dict(interval=8000, metric='mIoU', pre_eval=True)
work_dir = './work_dirs/fpn_r50_700X700_'
gpu_ids = range(0, 2)
auto_resume = False
