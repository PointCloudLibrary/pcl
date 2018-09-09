#! /usr/bin/env python3

"""
Software License Agreement (BSD License)

 Point Cloud Library (PCL) - www.pointclouds.org
 Copyright (c) 2018-, Open Perception, Inc.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holder(s) nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

"""

import argparse
from argparse import ArgumentParser
from collections import defaultdict
import getpass
import json
import os
import pathlib
import re
import subprocess
import sys

import requests


def find_pcl_folder():
    folder = os.path.dirname(os.path.abspath(__file__))
    folder = pathlib.Path(folder).parent
    return str(folder)


def find_pr_list(start: str, end: str):
    """Returns all PR ids from a certain commit range. Inspired in
    http://joey.aghion.com/find-the-github-pull-request-for-a-commit/
    https://stackoverflow.com/questions/36433572/how-does-ancestry-path-work-with-git-log#36437843
    """

    # Let git generate the proper pr history
    cmd = "git log --oneline " + start + ".." + end
    cmd = cmd.split()
    output = subprocess.run(cmd, cwd=FOLDER, stdout=subprocess.PIPE)
    pr_commits = output.stdout.split(b"\n")

    # Fetch ids for all merge requests from PRS
    merge_re = re.compile("\S+ Merge pull request #(\d+) from \S+")
    squash_re = re.compile("\(#(\d+)\)")

    ids = []
    for pr in pr_commits:

        pr_s = str(pr)

        # Match agains usual pattern
        uid = None
        match = merge_re.fullmatch(pr_s)

        # Match agains squash pattern
        if not match:
            match = squash_re.search(pr_s)

            # Abort
            if not match:
                continue

        # Extract PR uid
        uid = int(match.group(1))
        ids.append(uid)

    return ids


def fetch_pr_info(pr_ids, auth):

    prs_url = "https://api.github.com/repos/PointCloudLibrary/pcl/pulls/"
    pr_info = []

    sys.stdout.write("Fetching PR Info: {}%".format(0))
    sys.stdout.flush()

    for i, pr_id in enumerate(pr_ids):

        # Fetch GitHub info
        response = requests.get(prs_url + str(pr_id), auth=auth)
        data = response.json()

        if response.status_code != 200:
            print(
                "\nError: Failed to fetch PR info. Server reported '"
                + data["message"]
                + "'",
                file=sys.stderr,
            )
            exit(code=1)

        d = {"id": pr_id, "title": data["title"], "labels": data["labels"]}
        pr_info.append(d)

        # import pdb; pdb.set_trace()
        sys.stdout.write(
            "\rFetching PR Info: {:0.2f}%".format(100 * (i + 1) / len(pr_ids))
        )
        sys.stdout.flush()

    print("")
    return pr_info


def extract_version(tag):
    """Finds the corresponding version from a provided tag.
    If the tag does not correspond to a suitable version tag, the original tag
    is returned
    """
    version_re = re.compile("pcl-\S+")
    res = version_re.fullmatch(tag)

    # Not a usual version tag
    if not res:
        return tag

    return tag[4:]


def generate_text_content(tag, pr_info):

    module_order = (
        None,
        "cmake",
        "2d",
        "common",
        "cuda",
        "features",
        "filters",
        "geometry",
        "gpu",
        "io",
        "kdtree",
        "keypoints",
        "ml",
        "octree",
        "outofcore",
        "people",
        "recognition",
        "registration",
        "sample_consensus",
        "search",
        "segmentation",
        "simulation",
        "stereo",
        "surface",
        "apps",
        "docs",
        "tutorials",
        "examples",
        "tests",
        "tools",
        "ci",
    )

    module_titles = {
        None: "Uncategorized",
        "2d": "libpcl_2d",
        "apps": "PCL Apps",
        "cmake": "CMake",
        "ci": "CI",
        "common": "libpcl_common",
        "cuda": "libpcl_cuda",
        "docs": "PCL Docs",
        "examples": "PCL Examples",
        "features": "libpcl_features",
        "filters": "libpcl_filters",
        "gpu": "libpcl_gpu",
        "io": "libpcl_io",
        "kdtree": "libpcl_kdtree",
        "keypoints": "libpcl_keypoints",
        "ml": "libpcl_ml",
        "octree": "libpcl_octree",
        "outofcore": "libpcl_outofcore",
        "people": "libpcl_people",
        "recognition": "libpcl_recognition",
        "registration": "libpcl_registration",
        "sample_consensus": "libpcl_sample_consensus",
        "search": "libpcl_search",
        "segmentation": "libpcl_segmentation",
        "simulation": "libpcl_simulation",
        "stereo": "libpcl_stereo",
        "surface": "libpcl_surface",
        "tests": "PCL Tests",
        "tools": "PCL Tools",
        "tutorials": "PCL Tutorials",
        "visualization": "libpcl_visualization",
    }

    changes_order = ("new-feature", "deprecation", "removal", "behavior", "api", "abi")

    changes_titles = {
        "new-feature": "New Features",
        "deprecation": "Deprecated",
        "removal": "Removed",
        "behavior": "Behavioral changes",
        "api": "API changes",
        "abi": "ABI changes",
    }

    changes_description = {
        "new-feature": "Newly added functionalities.",
        "deprecation": "Deprecated code scheduled to be removed after two minor releases.",
        "removal": "Removal of deprecated code.",
        "behavior": "Changes in the expected default behavior.",
        "api": "Changes to the API which didn't went through the proper deprecation and removal cycle.",
        "abi": "Changes that cause ABI incompatibility but are still API compatible.",
    }

    changes_labels = {
        "breaks API": "api",
        "breaks ABI": "abi",
        "behavior": "behavior",
        "deprecation": "deprecation",
        "removal": "removal",
    }

    # change_log content
    clog = []

    # Infer version from tag
    version = extract_version(tag)

    # Find the commit date for writting the Title
    cmd = ("git log -1 --format=%ai " + tag).split()
    output = subprocess.run(cmd, cwd=FOLDER, stdout=subprocess.PIPE)
    date = output.stdout.split()[0].decode()
    tokens = date.split("-")
    clog += [
        "## *= "
        + version
        + " ("
        + tokens[2]
        + "."
        + tokens[1]
        + "."
        + tokens[0]
        + ") =*"
    ]

    # Map each PR into the approriate module and changes
    modules = defaultdict(list)
    changes = defaultdict(list)
    module_re = re.compile("module: \S+")
    changes_re = re.compile("changes: ")
    feature_re = re.compile("new feature")

    for pr in pr_info:

        pr["modules"] = []
        pr["changes"] = []

        for label in pr["labels"]:
            if module_re.fullmatch(label["name"]):
                module = label["name"][8:]
                pr["modules"].append(module)
                modules[module].append(pr)

            elif changes_re.match(label["name"]):
                key = changes_labels[label["name"][9:]]
                pr["changes"].append(key)
                changes[key].append(pr)

            elif feature_re.fullmatch(label["name"]):
                pr["changes"].append("new-feature")
                changes["new-feature"].append(pr)

        # No labels defaults to section None
        if not pr["modules"]:
            modules[None].append(pr)
            continue

    # Generate Changes Summary
    for key in changes_order:

        # Skip empty sections
        if not changes[key]:
            continue

        clog += ["\n### `" + changes_titles[key] + ":`\n"]

        clog += ["*" + changes_description[key] + "*\n"]

        for pr in changes[key]:
            prefix = "".join(["[" + k + "]" for k in pr["modules"]])
            if prefix:
                prefix = "**" + prefix + "** "
            clog += [
                "* "
                + prefix
                + pr["title"]
                + " [[#"
                + str(pr["id"])
                + "]]"
                + "(https://github.com/PointCloudLibrary/pcl/pull/"
                + str(pr["id"])
                + ")"
            ]

    # Traverse Modules and generate each section's content
    clog += ["\n### `Modules:`"]
    for key in module_order:

        # Skip empty sections
        if not modules[key]:
            continue

        # if key:
        clog += ["\n#### `" + module_titles[key] + ":`\n"]

        for pr in modules[key]:
            prefix = "".join(["[" + k + "]" for k in pr["changes"]])
            if prefix:
                prefix = "**" + prefix + "** "
            clog += [
                "* "
                + prefix
                + pr["title"]
                + " [[#"
                + str(pr["id"])
                + "]]"
                + "(https://github.com/PointCloudLibrary/pcl/pull/"
                + str(pr["id"])
                + ")"
            ]

    return clog


def parse_arguments():

    parser = ArgumentParser(
        description="Generate a change log between two "
        "revisions.\n\nCheck https://github.com/PointCloudLibrary/pcl/wiki/Preparing-Releases#creating-the-change-log "
        "for some additional examples on how to use the tool."
    )
    parser.add_argument(
        "start",
        help="The start (exclusive) " "revision/commit/tag to generate the log.",
    )
    parser.add_argument(
        "end",
        nargs="?",
        default="HEAD",
        help="The end "
        "(inclusive) revision/commit/tag to generate the log. "
        "(Defaults to HEAD)",
    )
    parser.add_argument(
        "--username",
        help="GitHub Account user name. If "
        "specified it will perform requests with the provided credentials. "
        "This is often required in order to overcome GitHub API's request "
        "limits.",
    )
    meg = parser.add_mutually_exclusive_group()
    meg.add_argument(
        "--cache",
        nargs="?",
        const="pr_info.json",
        metavar="FILE",
        help="Caches the PR info extracted from GitHub into a JSON file. "
        "(Defaults to 'pr_info.json')",
    )
    meg.add_argument(
        "--from-cache",
        nargs="?",
        const="pr_info.json",
        metavar="FILE",
        help="Uses a previously generated PR info JSON cache "
        "file to generate the change log. (Defaults to 'pr_info.json')",
    )

    # Parse arguments
    args = parser.parse_args()
    args.auth = None

    if args.username:
        password = getpass.getpass(prompt="Password for " + args.username + ": ")
        args.auth = (args.username, password)

    return args


##
##  'main'
##

FOLDER = find_pcl_folder()

# Parse arguments
args = parse_arguments()

pr_info = None
if not args.from_cache:

    # Find all PRs since tag
    prs = find_pr_list(start=args.start, end=args.end)

    # Generate pr objects with title, labels from ids
    pr_info = fetch_pr_info(prs, auth=args.auth)
    if args.cache:
        with open(args.cache, "w") as fp:
            d = {"start": args.start, "end": args.end, "pr_info": pr_info}
            fp.write(json.dumps(d))
else:
    # Load previously cached info
    with open(args.from_cache) as fp:
        d = json.loads(fp.read())
        pr_info = d["pr_info"]
        args.start = d["start"]
        args.end = d["start"]


# Generate text content based on changes
clog = generate_text_content(tag=args.end, pr_info=pr_info)
print("\n".join(clog))
