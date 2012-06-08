#!/bin/bash
/data/svn/pcl/trunk/build/bin/people_pcd_prob -numTrees 3 -tree0 ~/Data/results/forest1/tree_20.txt -tree1 ~/Data/results/forest2/tree_20.txt -tree2 ~/Data/results/forest3/tree_20.txt -tree3 ~/Data/results/forest3/tree_20.txt -mask 0 -FG 0 -pcd $1

