#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Release script usable by shmdata and switcher libraries.

Usage: ./release_version [OPTION]

Arguments:
  -l, --lib[=LIBRARY]    (Mandatory) Indicate which library will be released (must be shmdata or switcher)
"""
import sys
import getopt
import os
import shutil
import subprocess
import multiprocessing
import atexit
import re
import datetime

from enum import IntEnum, unique
from typing import List

libs_root_path = os.path.join(os.path.expanduser("~"), "src", "releases")
libraries = ["shmdata", "switcher"]
version_file = "CMakeLists.txt"

version_pattern = "set\({}_VERSION_(\S+)\s+(\d+)\)"
shmdata_require_pattern = "set\(SHMDATA_REQUIRED_VERSION (\d+\.\d+)\)"
git_path = "git@gitlab.com:nicobou"
remote_repo = "origin"
bringup_branch = "master"
working_branch = "develop"
release_branch = "release"
changelog_file = "changelog"
success = True


@unique
class VersionIncrease(IntEnum):
    """
    Version increase type
    """
    NONE = 0
    MAJOR = 1
    MINOR = 2
    PATCH = 3


def parse_version_number(lib: str, regex_pattern: str) -> List[int]:
    config_file = os.path.join(libs_root_path, lib, version_file)
    version_number = [-1, -1, -1]
    with open(config_file) as file:
        major = minor = patch = -1
        for line in file:
            version_line = re.search(regex_pattern, line)
            if version_line and len(version_line.groups()) == 2:
                type = version_line.group(1)
                version = version_line.group(2)
                if type == "MAJOR":
                    major = int(version)
                elif type == "MINOR":
                    minor = int(version)
                elif type == "PATCH":
                    patch = int(version)
                    break
            elif version_line and len(version_line.groups()) == 1:
                version = version_line.group(1).split(".")
                if len(version) == 2:
                    major = int(version[0])
                    minor = int(version[1])
                    patch = 0
                    break

    if major != -1 and minor != -1 and patch != -1:
        version_number = [major, minor, patch]
    else:
        printerr(f"Current version number not found in {config_file}")

    return version_number


def increase_version_number(version: List[int], version_increase: VersionIncrease) -> None:
    if version:
        if version_increase == VersionIncrease.MAJOR:
            version[1] = 0
            version[2] = 0
        elif version_increase == VersionIncrease.MINOR:
            version[2] = 0
        version[version_increase - 1] += 1
    else:
        printerr("Invalid version number, cannot proceed to increase it.")


def update_switcher_shmdata_version() -> None:
    os.chdir(os.path.join(libs_root_path, "shmdata"))
    assert git_checkout(
        bringup_branch) == 0, f"Failed to checkout {bringup_branch} branch of shmdata."
    shmdata_version = parse_version_number("shmdata", version_pattern.format("SHMDATA"))
    assert shmdata_version != [-1, -1, -1]
    print(f"found shmdata version {shmdata_version[0]}.{shmdata_version[1]}")
    build_dir = os.path.join(libs_root_path, "shmdata", "build")
    if not os.path.isdir(build_dir):
        os.mkdir(build_dir)
    os.chdir(build_dir)
    if subprocess.call(f"cmake -DENABLE_GPL=ON -DCMAKE_BUILD_TYPE=Release .. && make -j {multiprocessing.cpu_count()} && sudo make install", shell=True) != 0:
        printerr(f"{lib} build failed, stopping the release.")
    lib_repo = f"{git_path}/{lib}.git"
    os.chdir(os.path.join(libs_root_path))
    assert git_clone(lib_repo) == 0, f"Could not fetch codebase for library {lib} at {lib_repo}"
    os.chdir(os.path.join(libs_root_path, lib))
    switcher_shmdata_version = parse_version_number(lib, shmdata_require_pattern)
    if switcher_shmdata_version[0] != shmdata_version[0] or switcher_shmdata_version[1] != shmdata_version[1]:
        assert git_checkout(
            working_branch) == 0, f"Failed to checkout {working_branch} branch of switcher."
        commit_version_number(lib, shmdata_version, shmdata_require_pattern)


def commit_version_number(lib: str, new_version: List[int], regex_pattern: str) -> None:
    config_file = os.path.join(libs_root_path, lib, version_file)
    changed_file = config_file + ".tmp"
    shmdata_required_version = False
    with open(config_file) as old_file:
        with open(changed_file, "w") as new_file:
            for line in old_file:
                version_line = re.search(regex_pattern, line)
                if version_line and len(version_line.groups()) == 2:
                    type = version_line.group(1)
                    if type == "MAJOR":
                        line = re.sub(r"\d+", f"{new_version[0]}", line)
                    elif type == "MINOR":
                        line = re.sub(r"\d+", f"{new_version[1]}", line)
                    elif type == "PATCH":
                        line = re.sub(r"\d+", f"{new_version[2]}", line)
                elif version_line and len(version_line.groups()) == 1:
                    version = version_line.group(1).split(".")
                    if len(version) == 2:
                        line = re.sub(
                            r"\d+\.\d+", f"{new_version[0]}.{new_version[1]}", line)
                        shmdata_required_version = True
                new_file.write(line)

    os.rename(changed_file, config_file)
    git_add([config_file])
    if shmdata_required_version:
        assert git_commit(f"🔖 Shmdata version change to {new_version[0]}.{new_version[1]}") == 0, \
            "Failed to commit shmdata version number."
    else:
        assert git_commit(
            f"🔖 Version {new_version[0]}.{new_version[1]}.{new_version[2]}") == 0, "Failed to commit version number."


def git_push(remote_repo: str, remote_branch: str) -> int:
    return subprocess.call(f"git push {remote_repo} {remote_branch} --tags", shell=True)


def git_checkout(branch_name: str, is_new: bool = False) -> int:
    if is_new:
        return subprocess.call(f"git checkout -b {branch_name}", shell=True)
    else:
        return subprocess.call(f"git checkout {branch_name}", shell=True)


def git_commit(message: str) -> int:
    return subprocess.call(f"git commit -m \"{message}\"", shell=True)


def git_merge(branch_name: str, force: bool = False) -> int:
    if force:
        return subprocess.call(f"git merge -X theirs {branch_name}", shell=True)
    else:
        return subprocess.call(f"git merge {branch_name}", shell=True)


def git_tag(tag_name: str) -> int:
    return subprocess.call(f"git tag {tag_name}", shell=True)


def git_add(file_list: List[str]) -> None:
    for file in file_list:
        subprocess.call(f"git add {file}", shell=True)


def git_clone(repo_url: str) -> int:
    return subprocess.call(f"git clone {repo_url}", shell=True)


def git_pull() -> int:
    return subprocess.call("git pull", shell=True)


def git_submodule_update() -> int:
    return subprocess.call("git submodule update --init --recursive", shell=True)


def get_git_config(property: str, default_value: str) -> str:
    config_property = default_value
    git_config_full = subprocess.check_output(
        "git config --list", shell=True, encoding="utf-8").strip().split("\n")
    for config in git_config_full:
        prop = config.split("=")
        if len(prop) < 2:
            break
        if prop[0] == property:
            config_property = prop[1]
    return config_property


def update_changelog(lib: str, version: List[int]) -> None:
    print("Generating release notes")
    # NEWS.md contain the full history of releases
    orig_file_name = "NEWS.md"
    new_file_name = "NEWS.md.new"
    authors_file_name = "AUTHORS.md"
    # LAST_RELEASE.md contains only the last release details
    # It is used to describe release with gitlab's release system
    orig_release_file_name = "LAST_RELEASE.md"
    new_release_file_name = "LAST_RELEASE.md.new"

    git_checkout(working_branch)
    latest_tag = subprocess.check_output(
        "git describe --tags --abbrev=0", shell=True, encoding="utf-8")
    tag_date = subprocess.check_output(
        f"git log -1 --format=%ai {latest_tag}", shell=True, encoding="utf-8")
    commits = re.split(
        r"[a-z0-9]{40} ",
        subprocess.check_output(
            f"git log --pretty=oneline --since=\"{tag_date}\" | tr -d '\n'", shell=True, encoding="utf-8").strip()
    )
    commits = commits[1:-1]

    # interactive write of the "last release" file
    with open(new_release_file_name, "w") as new_file:
        new_file.write(
            f"{lib} {version[0]}.{version[1]}.{version[2]} ({datetime.date.today()})\n---------------------------\n\n")
        for commit in commits:
            new_file.write(f"* {commit}\n")
        new_file.write("\n")
    subprocess.call([get_git_config("core.editor", "vim"), new_release_file_name])
    os.rename(new_release_file_name, orig_release_file_name)

    # copy content of "last release" file into the changelog file
    with open(new_file_name, "w") as new_file:
        with open(orig_file_name, "r") as old_file:
            for i, line in enumerate(old_file.readlines()):
                if i != 3:
                    new_file.write(line)
                    continue
                with open(orig_release_file_name, 'r') as release_file:
                    new_file.write("\n")
                    new_file.write(release_file.read())
    os.rename(new_file_name, orig_file_name)
    subprocess.call(os.path.join(sys.path[0], "make_authors_from_git.sh"), shell=True)
    git_add([orig_file_name, orig_release_file_name])
    if subprocess.call(os.path.join(sys.path[0], "make_authors_from_git.sh"), shell=True) == 0:
        git_add([authors_file_name])
    git_commit(f"📝 Updated changelog for version {version[0]}.{version[1]}.{version[2]}.")


@ atexit.register  # Only clean on exit if the release was successful.
def cleanup_folder() -> None:
    global success
    if os.path.exists(libs_root_path) and success:
        shutil.rmtree(libs_root_path)
    success = False


def printerr(err: str) -> None:
    sys.stderr.write(err + "\n")
    exit(2)


def usage() -> None:
    print(__doc__)


if __name__ == "__main__":
    assert(sys.version_info[0] == 3 and sys.version_info[1] >
           6), f"This script must be ran with at least Python 3.7, detected Python {sys.version_info[0]}.{sys.version_info[1]}"

    release_version = []
    version_increase = VersionIncrease.NONE
    lib = ""

    try:
        opts, args = getopt.getopt(sys.argv[1:], "l:", ["lib="])
    except getopt.GetoptError as err:
        print(err)
        usage()
        exit(2)

    for o, a in opts:
        if o in ["-l", "--lib"]:
            if a in libraries:
                lib = a
            else:
                usage()
                printerr(f"Unmanaged library {a} provided")
        else:
            usage()
            printerr("Unhandled option")

    if not lib:
        usage()
        printerr("Target library not provided")

    cleanup_folder()  # Always remove the folder when launching the script.
    os.mkdir(libs_root_path)
    os.chdir(libs_root_path)

    choice = input(f"Is it a: 1/ Major release 2/ Minor release 3/ Bugfix release,"
                   f" of the {lib} library ? This will impact the new version number "
                   f"(x.y.z matches the choices 1.2.3.): ")
    if choice == "1":
        version_increase = VersionIncrease.MAJOR
    elif choice == "2":
        version_increase = VersionIncrease.MINOR
    elif choice == "3":
        version_increase = VersionIncrease.PATCH
    else:
        printerr("Wrong choice. Aborting the release.")

    # Always clone shmdata even when releasing switcher to synchronize versions.
    lib_repo = f"{git_path}/shmdata.git"
    assert git_clone(lib_repo) == 0, f"Could not fetch codebase for library {lib} at {lib_repo}"

    if lib == "switcher":
        update_switcher_shmdata_version()

    os.chdir(os.path.join(libs_root_path, lib))
    git_checkout(working_branch)
    git_submodule_update()

    release_version = parse_version_number(lib, version_pattern.format(lib.upper()))
    assert release_version != [-1, -1, -1]
    increase_version_number(release_version, version_increase)

    print("Version number found for all libraries, now executing unit tests.")

    build_dir = os.path.join(libs_root_path, lib, "build")

    if not os.path.isdir(build_dir):
        os.mkdir(build_dir)
    os.chdir(build_dir)

    if subprocess.call(f"cmake -DCMAKE_BUILD_TYPE=Release .. && make -j {multiprocessing.cpu_count()} package_source_test", shell=True) != 0:
        printerr(f"{lib} unit tests failed, stopping the release.")

    os.chdir(os.path.join(libs_root_path, lib))

    print("All unit tests passed successfully, now creating new branches for release.")

    quidfile = "../doc/quiddity_types.txt"
    subprocess.call(os.path.join(
        sys.path[0], f"echo \`\`\` > {quidfile} && switcher -K >> {quidfile} && echo \`\`\` >> {quidfile}"), shell=True)

    update_changelog(lib, release_version)
    new_branch = f"{release_branch}/version-{release_version[0]}.{release_version[1]}.{release_version[2]}"
    print(f"Creating branch {new_branch} for release of {lib} library.")
    git_checkout(new_branch, True)
    commit_version_number(lib, release_version, version_pattern.format(lib.upper()))
    assert git_checkout(bringup_branch) == 0, f"Could not checkout branch {bringup_branch}"
    assert git_merge(
        new_branch, True) == 0, f"Merge from branch {new_branch} into {bringup_branch} did not work."
    git_tag(f"{release_version[0]}.{release_version[1]}.{release_version[2]}")
    # If it's a bugfix release, we keep an odd number for develop and even for master.
    assert git_checkout(working_branch) == 0, f"Could not checkout branch {working_branch}"
    if version_increase == VersionIncrease.PATCH:
        release_version[2] += 1
    else:
        release_version[2] = 1
    commit_version_number(lib, release_version, version_pattern.format(lib.upper()))
    git_tag(f"{release_version[0]}.{release_version[1]}.{release_version[2]}")

    print("Pushing all branches and tags to remote.")
    do_push = input(
        f"Do you want to push {bringup_branch} and {working_branch} branches to {remote_repo}? [y/N]")
    if do_push == "y":
        assert git_push(
            remote_repo, bringup_branch) == 0, f"Failed to push branch {bringup_branch} into {remote_repo}/{bringup_branch}"
        assert git_push(
            remote_repo, working_branch) == 0, "Failed to push branch {working_branch} into {remote_repo}/{working_branch}"

    print("Cleaning up temporary files")
    success = True
