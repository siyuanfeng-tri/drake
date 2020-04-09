To generate the potato models, please refer to (it has a readme.)
drake/manipulation/models/potato_generator.py

To compile the data generator:
$ bazel build //examples/manipulation_station:pdc_record_data

I recommend running from the python driver script drake/batch_run.py
For example, I run with:
$ python3 batch_run.py --model_dir ~/tmp/potato_models/ --output_dir ~/tmp/mar_5 --num_runs 80
where ~/tmp/potato_models/ holds a bunch of potato models, and is typically an 
output from potato_generator.py
The arguments for the batch driver should be pretty self explanatory. 

The output should looks like this:
sfeng@sfeng0 ~/tmp $ tree mar_5
mar_5
├── 2020-03-05-15-22-43
│   └── processed
│       ├── image_masks
│       │   ├── 000000_mask.png
│       │   ├── 000001_mask.png
│       │   ├── 000002_mask.png
│       │   ├── 000003_mask.png
│       │   ├── 000004_mask.png
│       │   ├── 000005_mask.png
│       │   ├── 000006_mask.png
│       │   ├── 000007_mask.png
│       │   ├── 000008_mask.png
│       │   ├── 000009_mask.png
│       │   ├── 000010_mask.png
│       │   ├── 000011_mask.png
│       │   └── 000012_mask.png
│       ├── images
│       │   ├── 000000_rgb.png
│       │   ├── 000001_rgb.png
│       │   ├── 000002_rgb.png
│       │   ├── 000003_rgb.png
│       │   ├── 000004_rgb.png
│       │   ├── 000005_rgb.png
│       │   ├── 000006_rgb.png
│       │   ├── 000007_rgb.png
│       │   ├── 000008_rgb.png
│       │   ├── 000009_rgb.png
│       │   ├── 000010_rgb.png
│       │   ├── 000011_rgb.png
│       │   ├── 000012_rgb.png
│       │   ├── camera_info.yaml
│       │   └── pose_data.yaml
│       └── rendered_images
│           ├── 000000_depth.png
│           ├── 000001_depth.png
│           ├── 000002_depth.png
│           ├── 000003_depth.png
│           ├── 000004_depth.png
│           ├── 000005_depth.png
│           ├── 000006_depth.png
│           ├── 000007_depth.png
│           ├── 000008_depth.png
│           ├── 000009_depth.png
│           ├── 000010_depth.png
│           ├── 000011_depth.png
│           └── 000012_depth.png

