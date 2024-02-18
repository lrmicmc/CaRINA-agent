voxel_size = [0.32, 0.32, 6]
model = dict(
    type='MVXFasterRCNN',
    data_preprocessor=dict(
        type='Det3DDataPreprocessor',
        voxel=True,
        voxel_layer=dict(
            max_num_points=10,
            point_cloud_range=[-74.88, -74.88, -3, 74.88, 74.88, 3],
            voxel_size=[0.32, 0.32, 6],
            max_voxels=(32000, 32000))),
    pts_voxel_encoder=dict(
        type='HardVFE',
        in_channels=4,
        feat_channels=[64],
        with_distance=False,
        voxel_size=[0.32, 0.32, 6],
        with_cluster_center=True,
        with_voxel_center=True,
        point_cloud_range=[-74.88, -74.88, -3, 74.88, 74.88, 3],
        norm_cfg=dict(type='naiveSyncBN1d', eps=0.001, momentum=0.01)),
    pts_middle_encoder=dict(
        type='PointPillarsScatter', in_channels=64, output_shape=[468, 468]),
    pts_backbone=dict(
        type='SECOND',
        in_channels=64,
        norm_cfg=dict(type='naiveSyncBN2d', eps=0.001, momentum=0.01),
        layer_nums=[3, 5, 5],
        layer_strides=[1, 2, 2],
        out_channels=[64, 128, 256]),
    pts_neck=dict(
        type='SECONDFPN',
        norm_cfg=dict(type='naiveSyncBN2d', eps=0.001, momentum=0.01),
        in_channels=[64, 128, 256],
        upsample_strides=[1, 2, 4],
        out_channels=[128, 128, 128]),
    pts_bbox_head=dict(
        type='Anchor3DHead',
        num_classes=6,
        in_channels=384,
        feat_channels=384,
        use_direction_classifier=True,
        anchor_generator=dict(
            type='AlignedAnchor3DRangeGenerator',
            ranges=[[-74.88, -74.88, -0.0345, 74.88, 74.88, -0.0345],
                    [-74.88, -74.88, 0, 74.88, 74.88, 0],
                    [-74.88, -74.88, -0.1188, 74.88, 74.88, -0.1188]],
            sizes=[[4.73, 2.08, 1.77], [0.91, 0.84, 1.74], [1.81, 0.84, 1.77]],
            rotations=[0, 1.57],
            reshape_out=False),
        diff_rad_by_sin=True,
        dir_offset=-0.7854,
        bbox_coder=dict(type='DeltaXYZWLHRBBoxCoder', code_size=7),
        loss_cls=dict(
            type='mmdet.FocalLoss',
            use_sigmoid=True,
            gamma=2.0,
            alpha=0.25,
            loss_weight=1.0),
        loss_bbox=dict(
            type='mmdet.SmoothL1Loss',
            beta=0.1111111111111111,
            loss_weight=1.0),
        loss_dir=dict(
            type='mmdet.CrossEntropyLoss', use_sigmoid=False,
            loss_weight=0.2)),
    train_cfg=dict(
        pts=dict(
            assigner=[
                dict(
                    type='Max3DIoUAssigner',
                    iou_calculator=dict(type='BboxOverlapsNearest3D'),
                    pos_iou_thr=0.55,
                    neg_iou_thr=0.4,
                    min_pos_iou=0.4,
                    ignore_iof_thr=-1),
                dict(
                    type='Max3DIoUAssigner',
                    iou_calculator=dict(type='BboxOverlapsNearest3D'),
                    pos_iou_thr=0.5,
                    neg_iou_thr=0.3,
                    min_pos_iou=0.3,
                    ignore_iof_thr=-1),
                dict(
                    type='Max3DIoUAssigner',
                    iou_calculator=dict(type='BboxOverlapsNearest3D'),
                    pos_iou_thr=0.5,
                    neg_iou_thr=0.3,
                    min_pos_iou=0.3,
                    ignore_iof_thr=-1)
            ],
            allowed_border=0,
            code_weight=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            pos_weight=-1,
            debug=False)),
    test_cfg=dict(
        pts=dict(
            use_rotate_nms=True,
            nms_across_levels=False,
            nms_pre=4096,
            nms_thr=0.25,
            score_thr=0.1,
            min_bbox_size=0,
            max_num=500)))
dataset_type = 'CarlaTrackingDataset'
data_root = '/mnt/Data1/dataset_carla_tracking/'
backend_args = None
class_names = ['Car', 'Cyclist', 'Pedestrian']
metainfo = dict(classes=['Car', 'Cyclist', 'Pedestrian'])
point_cloud_range = [-74.88, -74.88, -3, 74.88, 74.88, 3]
input_modality = dict(use_lidar=True, use_camera=False)
train_pipeline = [
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4,
        backend_args=None),
    dict(type='LoadAnnotations3D', with_bbox_3d=True, with_label_3d=True),
    dict(
        type='RandomFlip3D',
        sync_2d=False,
        flip_ratio_bev_horizontal=0.5,
        flip_ratio_bev_vertical=0.5),
    dict(
        type='GlobalRotScaleTrans',
        rot_range=[-0.78539816, 0.78539816],
        scale_ratio_range=[0.95, 1.05]),
    dict(
        type='PointsRangeFilter',
        point_cloud_range=[-74.88, -74.88, -3, 74.88, 74.88, 3]),
    dict(
        type='ObjectRangeFilter',
        point_cloud_range=[-74.88, -74.88, -3, 74.88, 74.88, 3]),
    dict(type='PointShuffle'),
    dict(
        type='Pack3DDetInputs',
        keys=['points', 'gt_bboxes_3d', 'gt_labels_3d'])
]
test_pipeline = [
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4,
        backend_args=None),
    dict(
        type='MultiScaleFlipAug3D',
        img_scale=(1333, 800),
        pts_scale_ratio=1,
        flip=False,
        transforms=[
            dict(
                type='GlobalRotScaleTrans',
                rot_range=[0, 0],
                scale_ratio_range=[1.0, 1.0],
                translation_std=[0, 0, 0]),
            dict(type='RandomFlip3D'),
            dict(
                type='PointsRangeFilter',
                point_cloud_range=[-74.88, -74.88, -3, 74.88, 74.88, 3])
        ]),
    dict(type='Pack3DDetInputs', keys=['points'])
]
eval_pipeline = [
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4,
        backend_args=None),
    dict(type='Pack3DDetInputs', keys=['points'])
]
train_dataloader = dict(
    batch_size=2,
    num_workers=2,
    persistent_workers=True,
    sampler=dict(type='DefaultSampler', shuffle=True),
    dataset=dict(
        type='RepeatDataset',
        times=2,
        dataset=dict(
            type='CarlaTrackingDataset',
            data_root='/mnt/Data1/dataset_carla_tracking/',
            ann_file='annotation_velodyne_train.pkl',
            data_prefix=dict(pts=''),
            pipeline=[
                dict(
                    type='LoadPointsFromFile',
                    coord_type='LIDAR',
                    load_dim=4,
                    use_dim=4,
                    backend_args=None),
                dict(
                    type='LoadAnnotations3D',
                    with_bbox_3d=True,
                    with_label_3d=True),
                dict(
                    type='RandomFlip3D',
                    sync_2d=False,
                    flip_ratio_bev_horizontal=0.5,
                    flip_ratio_bev_vertical=0.5),
                dict(
                    type='GlobalRotScaleTrans',
                    rot_range=[-0.78539816, 0.78539816],
                    scale_ratio_range=[0.95, 1.05]),
                dict(
                    type='PointsRangeFilter',
                    point_cloud_range=[-74.88, -74.88, -3, 74.88, 74.88, 3]),
                dict(
                    type='ObjectRangeFilter',
                    point_cloud_range=[-74.88, -74.88, -3, 74.88, 74.88, 3]),
                dict(type='PointShuffle'),
                dict(
                    type='Pack3DDetInputs',
                    keys=['points', 'gt_bboxes_3d', 'gt_labels_3d'])
            ],
            modality=dict(use_lidar=True, use_camera=False),
            test_mode=False,
            metainfo=dict(classes=['Car', 'Cyclist', 'Pedestrian']),
            box_type_3d='LiDAR',
            backend_args=None)))
val_dataloader = dict(
    batch_size=1,
    num_workers=1,
    persistent_workers=True,
    drop_last=False,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type='CarlaTrackingDataset',
        data_root='/mnt/Data1/dataset_carla_tracking/',
        data_prefix=dict(pts=''),
        ann_file='annotation_velodyne_train.pkl',
        pipeline=[
            dict(
                type='LoadPointsFromFile',
                coord_type='LIDAR',
                load_dim=4,
                use_dim=4,
                backend_args=None),
            dict(type='Pack3DDetInputs', keys=['points'])
        ],
        modality=dict(use_lidar=True, use_camera=False),
        test_mode=True,
        metainfo=dict(classes=['Car', 'Cyclist', 'Pedestrian']),
        box_type_3d='LiDAR',
        backend_args=None))
test_dataloader = dict(
    batch_size=1,
    num_workers=1,
    persistent_workers=True,
    drop_last=False,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        type='CarlaTrackingDataset',
        data_root='/mnt/Data1/dataset_carla_tracking/',
        data_prefix=dict(pts=''),
        ann_file='annotation_velodyne_train.pkl',
        pipeline=[
            dict(
                type='LoadPointsFromFile',
                coord_type='LIDAR',
                load_dim=4,
                use_dim=4,
                backend_args=None),
            dict(type='Pack3DDetInputs', keys=['points'])
        ],
        modality=dict(use_lidar=True, use_camera=False),
        test_mode=True,
        metainfo=dict(classes=['Car', 'Cyclist', 'Pedestrian']),
        box_type_3d='LiDAR',
        backend_args=None))
val_evaluator = dict(
    type='KittiMetric',
    ann_file='/mnt/Data1/dataset_carla_tracking/annotation_velodyne_train.pkl',
    metric='bbox')
test_evaluator = dict(
    type='KittiMetric',
    ann_file='/mnt/Data1/dataset_carla_tracking/annotation_velodyne_train.pkl',
    metric='bbox')
lr = 0.0018
optim_wrapper = dict(
    type='OptimWrapper',
    optimizer=dict(
        type='AdamW', lr=0.0018, betas=(0.95, 0.99), weight_decay=0.01),
    clip_grad=dict(max_norm=10, norm_type=2))
param_scheduler = [
    dict(
        type='CosineAnnealingLR',
        T_max=16,
        eta_min=0.018,
        begin=0,
        end=16,
        by_epoch=True,
        convert_to_iter_based=True),
    dict(
        type='CosineAnnealingLR',
        T_max=24,
        eta_min=1.8e-07,
        begin=16,
        end=40,
        by_epoch=True,
        convert_to_iter_based=True),
    dict(
        type='CosineAnnealingMomentum',
        T_max=16,
        eta_min=0.8947368421052632,
        begin=0,
        end=16,
        by_epoch=True,
        convert_to_iter_based=True),
    dict(
        type='CosineAnnealingMomentum',
        T_max=24,
        eta_min=1,
        begin=16,
        end=40,
        by_epoch=True,
        convert_to_iter_based=True)
]
train_cfg = dict(by_epoch=True, max_epochs=40, val_interval=10)
val_cfg = dict()
test_cfg = dict()
auto_scale_lr = dict(enable=False, base_batch_size=48)
default_scope = 'mmdet3d'
default_hooks = dict(
    timer=dict(type='IterTimerHook'),
    logger=dict(type='LoggerHook', interval=50),
    param_scheduler=dict(type='ParamSchedulerHook'),
    checkpoint=dict(type='CheckpointHook', interval=1),
    sampler_seed=dict(type='DistSamplerSeedHook'),
    visualization=dict(type='Det3DVisualizationHook'))
env_cfg = dict(
    cudnn_benchmark=False,
    mp_cfg=dict(mp_start_method='fork', opencv_num_threads=0),
    dist_cfg=dict(backend='nccl'))
log_processor = dict(type='LogProcessor', window_size=50, by_epoch=True)
log_level = 'INFO'
load_from = None
resume = True
launcher = 'pytorch'
work_dir = './work_dirs/hv_pointpillars_secfpn_6x8_160e_carla-3d-2class'
