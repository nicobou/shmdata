#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ubuntu packaging script. This script does the following:
1 - Clone Shmdata in a new directory
2 - Merge master branch into debian/master
3 - Update debian/changelog with new package version
4 - Commit changes in the debian/master branch
5 - Ask the user if the updated debian/master branch needs to be pushed. 
    If yes, the Shmdata CI will build and publish the new package.

Usage: ./package_ubuntu.py

"""

import atexit
import getopt
import os
import shutil
import subprocess
import sys
import re

from typing import List

libs_root_path = os.path.join(os.path.expanduser("~"), "src", "releases")
version_file = "CMakeLists.txt"

version_pattern = "set\({}_VERSION_(\S+)\s+(\d+)\)"
git_path = "git@gitlab.com:sat-mtl/tools"
remote_repo = "origin"
bringup_branch = "master"
debian_branch_prefix = "debian"
changelog_file = "changelog"
success = True


def git_clean() -> None:
    """
    Clean the git repository of all temporary and untracked files
    """
    subprocess.call("git clean -d -X -f", shell=True)
    subprocess.call("git clean -d -f", shell=True)


def git_push(remote_repo: str, remote_branch: str) -> int:
    """
    Push the given branch to the given repository

    :param remote_repo: Remote repository URL
    :param remote_branch: Remote branch

    :return: Return the exit code of the command
    """
    return subprocess.call(f"git push {remote_repo} {remote_branch} --tags", shell=True)


def git_checkout(branch_name: str, is_new: bool = False) -> int:
    """
    Checkout the given branch

    :param branch_name: Branch to checkout
    :param is_new: If True, a new branch will be created

    :return: Return the exit code of the command
    """
    if is_new:
        return subprocess.call(f"git checkout -b {branch_name}", shell=True)
    else:
        return subprocess.call(f"git checkout {branch_name}", shell=True)


def git_commit(message: str) -> int:
    """
    Commit the staged changes

    :param message: Commit message

    :return: Return the exit code of the command
    """
    return subprocess.call(f"git commit -m \"{message}\"", shell=True)


def git_merge(branch_name: str, force: bool = False) -> int:
    """
    Merge the current branch

    :param branch_name: Branch to merge from
    :param force: If True, force merging

    :return: Return the exit code of the command
    """
    if force:
        return subprocess.call(f"git merge -X theirs {branch_name}", shell=True)
    else:
        return subprocess.call(f"git merge {branch_name}", shell=True)


def git_tag(tag_name: str) -> int:
    """
    Tag the current commit

    :param tag_name: Tag name

    :return: Return the exit code of the command
    """
    return subprocess.call(f"git tag {tag_name}", shell=True)


def git_add(file_list: List[str]) -> None:
    """
    Add the given files to be staged

    :param file_list: File list

    :return: Return the exit code of the command
    """
    for file in file_list:
        subprocess.call(f"git add {file}", shell=True)


def git_clone(repo_url: str) -> int:
    """
    Clone the given repository

    :param repo_url: Repository URL

    :return: Return the exit code of the command
    """
    return subprocess.call(f"git clone {repo_url}", shell=True)


def git_pull() -> int:
    """
    Pull changes from the remote repository

    :return: Return the exit code of the command
    """
    return subprocess.call("git pull", shell=True)


def get_git_config(property: str, default_value: str) -> str:
    """
    Get the value for the given configuration key

    :param property: Property to get from the git configuration
    :param default_value: Default value if the property is not set

    :return: Return the exit code of the command
    """
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


def date() -> str:
    """
    Get the current date in the Debian package standard format

    :return: The current date and time as a string
    """
    process = subprocess.Popen(['date', '-R'], stdout=subprocess.PIPE)
    output, err = process.communicate()
    return output.decode("utf-8")[0:-1]


def update_changelog(lib: str, version: List[int]) -> None:
    print("Updating changelog")
    orig_file_path = "debian/changelog"
    new_file_path = "debian/changelog.new"
    version_str = f"{version[0]}.{version[1]}.{version[2]}"

    author_name = get_git_config("user.name", "")
    author_email = get_git_config("user.email", "")

    new_name = input(f"Enter the name of the submitter (default: {author_name}): ")
    if new_name:
        author_name = new_name
    new_email = input(f"Enter the email of the submitter (default: {author_email}): ")
    if new_email:
        author_email = new_email

    with open(new_file_path, "w") as new_file:
        with open(orig_file_path, "r") as old_file:
            current_date = date()
            new_file.write(f"{lib} ({version_str}-1) focal; urgency=low\n\n"
                           f"  * New release for version {version_str}\n\n"
                           f" -- {author_name} <{author_email}>  {current_date}\n\n")

            for line in old_file.readlines():
                new_file.write(line)

    subprocess.call([get_git_config("core.editor", "vim"), new_file_path])
    os.rename(new_file_path, orig_file_path)
    git_add([orig_file_path])


@atexit.register  # Only clean on exit if the release was successful.
def cleanup_folder() -> None:
    global success
    if os.path.exists(libs_root_path) and success:
        shutil.rmtree(libs_root_path)
    success = False


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


def printerr(err: str) -> None:
    sys.stderr.write(err + "\n")
    exit(2)


def usage() -> None:
    print(__doc__)


if __name__ == "__main__":
    assert(sys.version_info[0] == 3 and sys.version_info[1] >
           6), f"This script must be ran with at least Python 3.7, detected Python {sys.version_info[0]}.{sys.version_info[1]}"

    cleanup_folder()  # Always remove the folder when launching the script.

    lib = "shmdata"
    debian_master_branch = f"{debian_branch_prefix}/master"

    os.mkdir(libs_root_path)
    os.chdir(libs_root_path)

    assert git_clone(
        f"{git_path}/{lib}.git") == 0, f"Could not fetch codebase for library {lib} at {libs_root_path}"
    os.chdir(os.path.join(libs_root_path, lib))

    git_checkout(bringup_branch)

    release_version = parse_version_number(lib, version_pattern.format(lib.upper()))

    git_checkout(debian_master_branch)

    print(f"Merging the {bringup_branch} into {debian_master_branch}")
    git_checkout(debian_master_branch)
    git_merge(bringup_branch)
    git_add(["debian/patches"])
    git_commit("Updated patches")

    update_changelog(f"lib{lib}", release_version)

    print("Committing the modifications to the source repository.")
    do_commit = input(
        f"Do you want to push updates to {lib} on branch {debian_master_branch} to {remote_repo}? [y/N]")
    if do_commit == "y":
        git_commit(f"🔖 Updated package from upstream to version {release_version}")
        git_tag(f"debian/{release_version[0]}.{release_version[1]}.{release_version[2]}")
        assert git_push(
            remote_repo, debian_master_branch) == 0, f"Failed to push branch {debian_master_branch} to {remote_repo}"

    print("Cleaning up temporary files")
    success = True
