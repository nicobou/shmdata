#!/usr/bin/python
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
import atexit
import re
import datetime

libs_root_path  = os.path.join(os.path.expanduser("~"), 'src', 'releases')
libraries       = ['shmdata', 'switcher']
version_file    = 'configure.ac'
version_pattern = 'AC_INIT\(.+(\d+\.\d+\.\d+).+\)'
github_path     = 'git@github.com:sat-metalab'
remote_repo     = 'origin'
bringup_branch  = 'master'
working_branch  = 'develop'
release_branch  = 'release'
changelog_file  = 'changelog'
check_success   = False


def parse_version_number(lib, version_regex):
    version = ''

    config_file = os.path.join(libs_root_path, lib, version_file)
    with open(config_file) as file:
        for line in file:
            version_line = version_regex.search(line)
            if version_line:
                version = version_line.group(1).split('.')
                for i,subversion in enumerate(version):
                    version[i] = int(subversion)
                break

    if version:
        print 'Current version found for library {} major: {}, minor: {}, hotfix: {}'.format(lib, version[0], version[1], version[2])
    else:
        printerr('Current version number not found in {}'.format(config_file))

    return version


def increase_version_number(version, version_increase, force_bugfix_version=0):
    if version:
        if version_increase == 1:
            version[1] = 0
            version[2] = force_bugfix_version
        if version_increase == 2:
            version[2] = force_bugfix_version
        version[version_increase - 1] += 1
    else:
        printerr('Invalid version number, cannot proceed to increase it.')
    return version


def commit_version_number(lib, new_version, version_regex):
    config_file = os.path.join(libs_root_path, lib, version_file)
    changed_file = config_file + ".tmp"
    with open(config_file) as old_file:
        with open(changed_file, 'w') as new_file:
            for line in old_file:
                version_line = version_regex.search(line)
                if version_line:
                    line = re.sub(r'\d+.\d+.\d+', '{}.{}.{}'.format(new_version[0], new_version[1], new_version[2]), line)
                new_file.write(line)
    os.rename(changed_file, config_file)
    git_add([config_file])
    assert git_commit('Version {}.{}.{}'.format(new_version[0], new_version[1], new_version[2])) == 0, 'Failed to commit version number.'


def git_push(remote_repo, remote_branch):
    return subprocess.call('git push {} {} --tags'.format(remote_repo, remote_branch), shell=True)


def git_checkout(branch_name, is_new=False):
    if is_new:
        return subprocess.call('git checkout -b {}'.format(branch_name), shell=True)
    else:
        return subprocess.call('git checkout {}'.format(branch_name), shell=True)


def git_commit(message):
    return subprocess.call('git commit -m "{}"'.format(message), shell=True)


def git_merge(branch_name, force=False):
    if force:
        return subprocess.call('git merge -X theirs {}'.format(branch_name), shell=True)
    else:
        return subprocess.call('git merge {}'.format(branch_name), shell=True)


def git_tag(tag_name):
    return subprocess.call('git tag {}'.format(tag_name), shell=True)


def git_add(file_list):
    for file in file_list:
        subprocess.call('git add {}'.format(file), shell=True)


def git_clone(repo_url):
    return subprocess.call('git clone {}'.format(repo_url), shell=True)


def git_pull():
    return subprocess.call('git pull', shell=True)


def git_submodule(recursive=False):
    recursive_str = ''
    if recursive:
        recursive_str = '--recursive'
    return subprocess.call('git submodule update --init {}'.format(recursive_str), shell=True)


def get_git_config(property, default_value):
    config_property = default_value
    git_config_full = subprocess.check_output('git config --list', shell=True).strip().split('\n')
    for config in git_config_full:
        prop = config.split('=')
        if len(prop) < 2:
            break;
        if prop[0] == property:
            config_property = prop[1]
    return config_property


def update_changelog(lib, version):
    print 'Generating release notes'
    commit_messages = []
    orig_file_name = 'NEWS.md'
    new_file_name = 'NEWS.md.new'
    authors_file_name = 'AUTHORS.md'

    git_checkout(working_branch)
    latest_tag = subprocess.check_output('git describe --tags --abbrev=0', shell=True)
    tag_date = subprocess.check_output('git log -1 --format=%ai {}'.format(latest_tag), shell=True)
    commits = re.split(r'commit [a-z0-9]+' , subprocess.check_output('git log --first-parent --since="{}"'.format(tag_date), shell=True).strip())
    with open(new_file_name, 'w') as new_file:
        with open(orig_file_name, 'r') as old_file:
            for i, line in enumerate(old_file.readlines()):
                if i != 3:
                    new_file.write(line)
                    continue
                new_file.write('\n{} {}.{}.{} ({})\n---------------------------\n'
                               'This is an official release in the {}.{} stable series.\n\n'
                               .format(lib, version[0], version[1], version[2], datetime.date.today(), version[0], version[1]))
                for commit in commits:
                    elements = commit.split('\n    ')
                    if len(elements) < 2:
                        continue
                    commit_message = ''
                    for message in elements[1:]:
                        commit_message += message + ' '
                    if commit_message:
                        commit_message = re.sub(r'reviewer\s*:\s*[a-z]+', r'', commit_message, flags=re.IGNORECASE)
                        new_file.write('* {}\n'.format(commit_message.strip()))

                new_file.write('\n')
    subprocess.call([get_git_config('core.editor', 'vim'), new_file_name])
    os.rename(new_file_name, orig_file_name)
    subprocess.call(os.path.join(sys.path[0], 'make_authors_from_git.sh'), shell=True)
    git_add([orig_file_name, authors_file_name])
    git_commit('Updated changelog for version {}.{}.{}.'.format(version[0], version[1], version[2]))


@atexit.register
def cleanup_folder():
    if os.path.exists(libs_root_path) and check_success:
        shutil.rmtree(libs_root_path)


def printerr(err):
    sys.stderr.write(err + '\n')
    exit(2)


def usage():
    print __doc__


if __name__ == '__main__':
    version_regex = re.compile(version_pattern)
    version_release = []
    version_increase = 0
    lib = ''

    try:
        opts, args = getopt.getopt(sys.argv[1:], "l", ["lib="])
    except getopt.GetoptError as err:
        print err
        usage()
        exit(2)

    for o, a in opts:
        if o in ["-l", "--lib"]:
            if a in libraries:
                lib = a
            else:
                usage()
                printerr('Unmanaged library {} provided'.format(a))
        else:
            usage()
            printerr('Unhandled option')

    if not lib:
        usage()
        printerr('Target library not provided')

    cleanup_folder()
    os.mkdir(libs_root_path)
    os.chdir(libs_root_path)

    choice = raw_input('Is it a: 1/ Major release 2/ Minor release 3/ Bugfix release,'
                       ' of the {} library ? This will impact the new version number '
                       '(x.y.z matches the choices 1.2.3.): '.format(lib))
    if choice == '1':
        version_increase = 1
    elif choice == '2':
        version_increase = 2
    elif choice == '3':
        version_increase = 3
    else:
        printerr('Wrong choice. Aborting the release.')

    lib_repo = '{}/{}.git'.format(github_path, lib)

    assert(git_clone(lib_repo) == 0, 'Could not fetch code base for library {} at {}'.format(lib, lib_repo))

    os.chdir(os.path.join(libs_root_path, lib))
    git_checkout(working_branch)

    assert(git_submodule(True) == 0, 'Could not fetch summodules')

    version_release = parse_version_number(lib, version_regex)
    version_release = increase_version_number(version_release, version_increase)

    print 'Version number found for all libraries, now executing unit tests.'

    if subprocess.call('./autogen.sh && ./configure && make -j check', shell=True) != 0:
        printerr('{} unit tests failed, stopping the release.'.format(lib))

    check_success = True
    print 'All unit tests passed successfully, now creating new branches for release.'

    update_changelog(lib, version_release)
    new_branch = '{}/version-{}.{}.{}'.format(release_branch, version_release[0], version_release[1], version_release[2])
    print 'Creating branch {} for release of {} library.'.format(new_branch, lib)
    git_checkout(new_branch, True)
    commit_version_number(lib, version_release, version_regex)
    assert git_checkout(bringup_branch) == 0, 'Could not checkout branch {}'.format(bringup_branch)
    assert git_merge(new_branch, True) == 0, 'Merge from branch {} into {} did not work.'.format(new_branch, bringup_branch)
    git_tag('{}.{}.{}'.format(version_release[0], version_release[1], version_release[2]))
    assert git_push(remote_repo, bringup_branch) == 0, 'Failed to push branch {} into {}/{}'.format(bringup_branch, remote_repo, bringup_branch)
    # If it's a bugfix release, we keep an odd number for develop and even for master.
    assert git_checkout(working_branch) == 0, 'Could not checkout branch {}'.format(working_branch)
    version_develop = list(version_release)
    if version_increase == 3:
        version_develop = increase_version_number(version_develop, version_increase, 1)
    commit_version_number(lib, version_develop, version_regex)
    git_tag('{}.{}.{}'.format(version_develop[0], version_develop[1], version_develop[2]))
    assert git_push(remote_repo, working_branch) == 0, 'Failed to push branch {} into {}/{}'.format(working_branch, remote_repo, working_branch)
