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

import json
import argparse
from pathlib import Path

import requests
import re


CATEGORIES = {
    "new feature": ("New features", "added to PCL"),
    "deprecation": (
        "Deprecation",
        "of public APIs, scheduled to be removed after two minor releases",
    ),
    "removal": ("Removal", "of the public APIs deprecated in previous releases"),
    "behavior change": ("Behavior changes", "in classes, apps, or tools",),
    "API break": (
        "API changes",
        "that did not go through the proper deprecation and removal cycle",
    ),
    "ABI break": ("ABI changes", "that are still API compatible",),
    "fix": (None, None),
    "enhancement": (None, None),
}


MODULES = {
    "cmake": "CMake",
    "2d": "libpcl_2d",
    "common": "libpcl_common",
    "cuda": "libpcl_cuda",
    "features": "libpcl_features",
    "filters": "libpcl_filters",
    "geometry": "libpcl_geometry",
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
    "visualization": "libpcl_visualization",
    "apps": "PCL Apps",
    "docs": "PCL Docs",
    "tutorials": "PCL Tutorials",
    "examples": "PCL Examples",
    "tests": "PCL Tests",
    "tools": "PCL Tools",
    "ci": "CI",
    None: "Uncategorized",
}


def fetch_latest_release_date():
    """
    Fetch the date when the latest release was created.
    """
    url = "https://api.github.com/repos/PointCloudLibrary/pcl/releases/latest"
    response = requests.get(url)
    response.raise_for_status()
    return response.json()["created_at"]


def fetch_pr_since(start_date):
    """
    Fetch data of PRs merged after a given start date.
    """
    url = f"https://api.github.com/search/issues?q=is:pr+repo:PointCloudLibrary/pcl+merged:>={start_date}"
    pr_data = list()
    page = 1
    while True:
        response = requests.get(f"{url}&page={page}&per_page=100")
        response.raise_for_status()
        data = response.json()
        total_count = data["total_count"]
        pr_data.extend(data["items"])
        page += 1
        if len(pr_data) == total_count:
            break
    return pr_data


def filter_labels(labels, prefix):
    """
    Filter a given list of PR labels, keeping only those starting with a given prefix.
    The prefix is stripped from the labels.
    """
    return [
        label["name"][len(prefix) :]
        for label in labels
        if label["name"].startswith(prefix)
    ]


def strip_leading_tag(text):
    """
    >>> strip_leading_tag("[text] larger text")
    'larger text'
    >>> strip_leading_tag("no tag text")
    'no tag text'
    """
    if len(text) == 0 or text[0] != '[':
        return text
    pattern = re.compile('\[.*\]\s*')
    match = pattern.match(text)
    return text[match.end():] if match else text


def make_pr_bullet_point(pr, prefix=None):
    ref = "[#{0}](https://github.com/PointCloudLibrary/pcl/pull/{0})".format(
        pr["number"]
    )

    tags = ""
    if prefix in ("modules", "categories"):
        tags = "".join(["[" + k + "]" for k in pr[prefix]])
    if tags:
        tags = "**" + tags + "** "

    return f"* {tags}{pr['title']} [{ref}]"


def generate_category_section(key, prs):
    section = list()
    filtered_prs = [pr for pr in prs if key in pr["categories"]]
    title, description = CATEGORIES[key]
    if filtered_prs and title:
        section += [f"\n**{title}** *{description}*\n"]
        section += [make_pr_bullet_point(pr, "modules") for pr in filtered_prs]
    return section


def generate_module_section(key, prs):
    section = list()
    filtered_prs = [pr for pr in prs if key in pr["modules"]]
    title = MODULES[key]
    if filtered_prs and title:
        section += [f"\n#### {title}:\n"]
        section += [make_pr_bullet_point(pr, "categories") for pr in filtered_prs]
    return section


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="""
    Generate changelog from pull requests merged after the latest release.
    """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    cache_grp = parser.add_mutually_exclusive_group()
    cache_grp.add_argument("--save", type=Path, help="Save PR data fetched from GitHub")
    cache_grp.add_argument("--load", type=Path, help="Load PR data from a file")
    parser.add_argument(
        "--with-misc",
        "-m",
        action="store_true",
        help=(
            "Add section with miscellaneous PRs that are not important enough to be "
            "included in the official changelog"
        ),
    )
    args = parser.parse_args()

    if args.load:
        with open(args.load) as fp:
            pr_data = json.loads(fp.read())
    else:
        date = fetch_latest_release_date()
        pr_data = fetch_pr_since(date)
        if args.save:
            with open(args.save, "w") as fp:
                fp.write(json.dumps(pr_data))

    selected_prs = list()
    excluded_prs = list()
    for pr in sorted(pr_data, key=lambda d: d["closed_at"]):
        categories = filter_labels(pr["labels"], "changelog: ")
        title = strip_leading_tag(pr["title"])
        pr_info = {
            "number": pr["number"],
            "title": title,
            "modules": filter_labels(pr["labels"], "module: "),
            "categories": [g for g in categories if g not in ["fix", "enhancement"]],
        }
        if categories:
            selected_prs.append(pr_info)
        else:  # exclude PRs not tagged with any changelog label
            excluded_prs.append(pr_info)

    clog = list()

    clog += ["\n### Notable changes"]
    for k in CATEGORIES.keys():
        clog.extend(generate_category_section(k, selected_prs))

    clog += ["\n### Changes grouped by module"]
    for k in MODULES.keys():
        clog.extend(generate_module_section(k, selected_prs))

    if args.with_misc:
        clog += ["\n### Miscellaneous PRs excluded from changelog\n"]
        for pr in excluded_prs:
            clog += [make_pr_bullet_point(pr)]

    print("\n".join(clog))
