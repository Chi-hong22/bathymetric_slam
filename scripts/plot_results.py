#!/usr/bin/python

import matplotlib.pyplot as plot
import numpy
import sys
from optparse import OptionParser

parser = OptionParser()
parser.add_option("--initial_poses",
                  dest="initial_poses",
                  default="",
                  help="The filename that contains the original poses.")
parser.add_option("--corrupted_poses",
                  dest="corrupted_poses",
                  default="",
                  help="The filename that contains the optimized poses.")
parser.add_option("--optimized_poses",
                  dest="optimized_poses",
                  default="",
                  help="The filename that contains the optimized poses.")
parser.add_option("-e",
                  "--axes_equal",
                  action="store_true",
                  dest="axes_equal",
                  default="",
                  help="Make the plot axes equal.")
parser.add_option("--output_file",
                  dest="outputFile",
                  default="",
                  help="The output file.")

(options, args) = parser.parse_args()

# Read the original and optimized poses files.
poses_original = None
if options.initial_poses != '':
    poses_original = numpy.genfromtxt(options.initial_poses, usecols=(1, 2, 3))

poses_corrupted = None
if options.corrupted_poses != '':
    poses_corrupted = numpy.genfromtxt(options.corrupted_poses,
                                       usecols=(1, 2, 3))

poses_optimized = None
if options.optimized_poses != '':
    poses_optimized = numpy.genfromtxt(options.optimized_poses,
                                       usecols=(1, 2, 3))

sum_opt = 0.0
sum_corr = 0.0
for i in range(0, len(poses_optimized)):
    sum_opt += numpy.linalg.norm(poses_original[i, :] - poses_optimized[i, :])
    sum_corr += numpy.linalg.norm(poses_original[i, :] - poses_corrupted[i, :])

sum_opt /= len(poses_optimized)
sum_corr /= len(poses_optimized)

print("Diff original and corrupted", sum_corr)
print("Diff original and optimized", sum_opt)

#  with open(options.outputFile, "a") as text_file:
#  text_file.write("%s" % sum_corr)
#  text_file.write(" %s" % sum_opt)
#  text_file.close()
#
# Plots the results for the specified poses.
figure = plot.figure()
axes = figure.add_subplot(111, projection='3d')

if poses_original is not None:
    plot.plot(poses_original[:, 0],
              poses_original[:, 1],
              poses_original[:, 2],
              '-',
              alpha=0.5,
              color="green",
              linewidth=2.0,
              label='Ground Truth')

if poses_corrupted is not None:
    plot.plot(poses_corrupted[:, 0],
              poses_corrupted[:, 1],
              poses_corrupted[:, 2],
              '-',
              alpha=0.5,
              color="red",
              linewidth=2.0,
              label='Corrupted')

if poses_optimized is not None:
    plot.plot(poses_optimized[:, 0],
              poses_optimized[:, 1],
              poses_optimized[:, 2],
              '-',
              alpha=0.5,
              color="blue",
              linewidth=2.0,
              label='Optimized')

# Set aspect ratio to be based on data ranges for a true-to-scale representation
all_poses = []
if poses_original is not None:
    all_poses.append(poses_original)
if poses_corrupted is not None:
    all_poses.append(poses_corrupted)
if poses_optimized is not None:
    all_poses.append(poses_optimized)

if all_poses:
    combined_poses = numpy.vstack(all_poses)
    min_coords = numpy.min(combined_poses, axis=0)
    max_coords = numpy.max(combined_poses, axis=0)
    ranges = max_coords - min_coords
    # To avoid division by zero or tiny ranges, set a minimum range
    ranges[ranges < 1e-6] = 1e-6
    axes.set_box_aspect(ranges)

plot.legend()
plot.title('Trajectories Comparison')
plot.xlabel('X')
plot.ylabel('Y')
axes.set_zlabel('Z')

plot.show()
