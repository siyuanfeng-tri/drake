import os
import subprocess
import argparse
import random


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_runs', type=int, default=1)
    parser.add_argument('--model_dir', type=str, default="")
    parser.add_argument('--output_dir', type=str,
        default="/home/sfeng/tmp/pdc_img/")
    args = parser.parse_args()

    output_dir = os.path.expanduser(args.output_dir)
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    drake_path = os.path.dirname(os.path.realpath(__file__))
    model_dir = args.model_dir
    if not model_dir:
        model_dir = os.path.join(drake_path, 'manipulation/models')
    else:
        model_dir = os.path.expanduser(model_dir)
    model_names = os.listdir(model_dir)
    model_names = [x for x in model_names if 'auto--' in x]

    random.seed()

    all_dirs = set(os.listdir(output_dir))

    i = 0
    while i < args.num_runs:
        if len(model_names) <= 1:
            name_idx = 0
        else:
            name_idx = random.randrange(len(model_names))
        model_name = model_names[name_idx]
        min_dist = random.uniform(0.3, 1.)
        max_dist = min_dist + 0.05
        print(f'\n\n\n EXECUTING {i + 1} out of {args.num_runs} w. {model_name}\n\n\n')
        stats = subprocess.run([
#            os.path.join(drake_path, 'bazel-bin/examples/manipulation_station/potato_demo'),
#            '--target_realtime_rate', '1.0',
#            '--model_dir', model_dir,
#            '--model_name', model_name,
            os.path.join(drake_path, 'bazel-bin/examples/manipulation_station/pdc_record_data'),
            '--target_realtime_rate', '.0',
            '--min_z', f'{min_dist}', '--max_z', f'{max_dist}',
            '--model_dir', model_dir,
            '--model_name', model_name,
            '--record_period', f'{1/5}',
            '--record_start', f'{2}',
            '--output_dir', output_dir,
            ])

        # Success
        if stats.returncode == 0:
            i += 1

    # The c++ prog will remove ones that failed.
    new_alldirs = list(set(os.listdir(output_dir)) - all_dirs)
    new_alldirs.sort()
    print('\n\n\n\n\n=================================================')
    print('Newly generated:')
    for d in new_alldirs:
        print(f'- \"{d}\"')


if __name__ == "__main__":
    main()
