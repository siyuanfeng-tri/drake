'''
This thing takes two inputs:
- a folder for obj models
- a folder of folders for texture files
and for every combination of obj X texture, it generates a folder that
contains a sdf, obj, texture files and mtl file. The sdf has bogus inertia
and collision, and the mtl are bogus as well, but good enough for now.
Feel free to modify the obj and mtl templates below to fit your use case.

Note, for texture inputs, I am assuming there is a binary mask for each texture,
and the binary mask is a 8 bit 3 channel image, and 255 means peeled, 0 means
skin. If the mask isn't present, a zero mask is assumed (all skin).

For the obj input, mine looks like:
sfeng@sfeng0 ~/Downloads $ tree potato_sdf
potato_sdf
├── scale_diffpotato0_31.obj
├── scale_diffpotato0_36.obj
├── yukon_gold_potato.obj
└── yukon_mix_potato.obj

For the texture input, mine looks like:
sfeng@sfeng0 ~/Downloads $ tree potato_texture/
potato_texture/
├── peeled_color_chage_yukon
│   ├── peel_potato_0_mask.png
│   ├── peel_potato_0.png
│   ├── peel_potato_10_mask.png
│   ├── peel_potato_10.png
│   ├── peel_potato_11_mask.png
│   ├── peel_potato_11.png

'''

import os
import shutil
import fileinput

import argparse
import cv2
import numpy as np


def sdf_generator(model_name, obj_file, roll_rad, scale, texture_file):
    sdf_template = f"""<?xml version='1.0' ?>
<sdf version='1.6'>

  <!--
  Axes:
    +X - Pointing toward Handle.
    +Y - Right side of cup, if handle is facing toward you.
    +Z - towards the top of the cup.
  Origin:
    (0, 0, 0) at the center of the mug cylinder.
  Dimensions (approximate):
    Cylinder:
      Radius: 4cm
      Height: 10cm
    Handle:
      Max X-Dist. from Cylinder: 3cm
        Z-Dist of above point: 3.2cm
      Width: 2cm
  -->

  <model name='{model_name}'>
    <link name='{model_name}'>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <inertial>
        <pose frame=''>0.01 0 0 0 -0 0</pose>
        <mass>0.094</mass>
        <inertia>
          <ixx>0.000156</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000156</iyy>
          <iyz>0</iyz>
          <izz>0.00015</izz>
        </inertia>
      </inertial>

      <collision name='base_collision_1'>
        <pose frame=''>0.04 0.08 -0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <collision name='base_collision_2'>
        <pose frame=''>-0.04 0.08 -0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <collision name='base_collision_3'>
        <pose frame=''>0.04 -0.08 -0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <collision name='base_collision_4'>
        <pose frame=''>-0.04 -0.08 -0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <collision name='base_collision_5'>
        <pose frame=''>0.04 0.08 0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <collision name='base_collision_6'>
        <pose frame=''>-0.04 0.08 0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <collision name='base_collision_7'>
        <pose frame=''>0.04 -0.08 0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <collision name='base_collision_8'>
        <pose frame=''>-0.04 -0.08 0.04 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.0025</radius>
          </sphere>
        </geometry>
        <surface>
	  <friction>
	    <ode>
	      <mu>0.9</mu>
	      <mu2>0.5</mu2>
	    </ode>
	  </friction>
	</surface>
      </collision>

      <visual name='base_visual'>
        <!-- See *.model.yaml file -->
        <pose frame=''>0 0 0 {roll_rad} 0 0</pose>
        <geometry>
          <mesh>
            <scale>{scale} {scale} {scale}</scale>
            <uri>{obj_file}</uri>
            <!-- <uri>scale_diffpotato0_36.obj</uri> -->
          </mesh>
        </geometry>
        <material>
          <drake:diffuse_map>{texture_file}</drake:diffuse_map>
	  <!-- <drake:diffuse_map>3.png</drake:diffuse_map> -->
          <!-- <diffuse>0.0 0.75 1.0 1.0</diffuse> -->
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
    return sdf_template


def mtl_generator(texture_file):
    template = f"""newmtl Material
Ns 0.000000
Ka 1.000000 1.000000 1.000000
Kd 0.800000 0.800000 0.800000
Ks 0.500000 0.500000 0.500000
Ke 0.000000 0.000000 0.000000
Ni 1.450000
d 1.000000
illum 2
map_Kd {texture_file}
"""
    return template


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--obj_dir', type=str,
        default='/home/sfeng/Downloads/potato_sdf', help='Dir to obj files')
    parser.add_argument('--texture_root_dir', type=str,
        default='/home/sfeng/Downloads/potato_texture',
        help='''Dir to png texture and masks. The mask should be a 3 channel
                8 bit binary image, where 255 means peeled part, 0 means
                skin.''')
    parser.add_argument('--output_dir', type=str,
        default='/home/sfeng/code/drake/manipulation/models',
        help='Output dir.')
    args = parser.parse_args()

    obj_dir = args.obj_dir
    texture_root_dir = args.texture_root_dir
    output_dir = args.output_dir

    obj_files = os.listdir(obj_dir)
    potato_dirs = os.listdir(texture_root_dir)

    # Note: This is also hardcoded in the training code.
    PEELED_MASK = 2
    SKIN_MASK = 1

    for obj_file in obj_files:
        for potato_name in potato_dirs:
            texture_dir = os.path.join(texture_root_dir, potato_name)
            texture_files = os.listdir(texture_dir)
            texture_files = [x for x in texture_files if 'mask' not in x]
            for texture_file in texture_files:
                texture_path = os.path.join(texture_dir, texture_file)
                obj_path = os.path.join(obj_dir, obj_file)

                model_name = f'auto--{potato_name}--{obj_file}--{texture_file}'

                # TODO(Taku): For some reason, the deepsdf generated model, I need
                # to scale them down by this much. I don't know the correct scale.
                # This is eye balled.
                if 'scale_diffpotato' in obj_file:
                    roll_rad = np.pi / 2
                    scale = 0.04
                else:
                    roll_rad = 0
                    scale = 1

                sdf_template = sdf_generator(
                    model_name, obj_file, roll_rad, scale, texture_file)

                # Make dir
                out_dir = os.path.join(output_dir, model_name)
                if not os.path.isdir(out_dir):
                    os.makedirs(out_dir)
                print(f'Writing to {out_dir}...')

                # Write sdf
                f = open(os.path.join(out_dir, f'{model_name}.sdf'), 'w')
                f.write(sdf_template)
                f.close()

                # Write mtl, this is probably bogus.
                obj_file_no_ext = os.path.splitext(obj_file)[0]
                mtl_template = mtl_generator(texture_file)
                f = open(os.path.join(out_dir, f'{obj_file_no_ext}.mtl'), 'w')
                f.write(mtl_template)
                f.close()

                # Modify obj with the right mtl.
                with open(obj_path, 'r') as file:
                    obj_data = file.read()
                lines = obj_data.splitlines()
                with open(os.path.join(out_dir, obj_file), 'w') as file:
                    for line in lines:
                        if 'mtllib' in line:
                            line = f'mtllib {obj_file_no_ext}.mtl'
                        if 'usemtl' in line:
                            # This is hard coded in mtl_generator().
                            line = 'usemtl Material'
                        file.write(line + '\n')

                # Copy texture and sdf.
                shutil.copyfile(texture_path, os.path.join(out_dir, texture_file))

                # Modify the binary mask s.t. 0 in taku's mask -> SKIN_MASK in
                # sfeng mask, and 1 -> PEELED_MASK
                texture_mask_file = os.path.splitext(texture_file)[0] + "_mask.png"
                texture_mask_path = os.path.join(texture_dir, texture_mask_file)
                if os.path.isfile(texture_mask_path):
                    mask = cv2.imread(texture_mask_path)
                else:
                    texture = cv2.imread(texture_path)
                    H, W, _ = texture.shape
                    mask = np.zeros([H, W], dtype=np.uint8)
                modified_mask = np.where(mask == 0, SKIN_MASK, PEELED_MASK)
                cv2.imwrite(os.path.join(out_dir, texture_mask_file), modified_mask)

if __name__ == "__main__":
    main()
